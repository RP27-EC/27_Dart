/**
 * host_check：link24g_frame.h 帧编解码与字节流解析检查。
 * 只验证纯逻辑，不代表 DL20 链路或实机通过。
 */
#include <stdio.h>
#include "link24g_frame.h"

static int s_fail = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (cond) {                                                   \
            printf("  [OK]   %s\n", msg);                             \
        } else {                                                      \
            printf("  [FAIL] %s (line %d)\n", msg, __LINE__);         \
            s_fail++;                                                 \
        }                                                             \
    } while (0)

static int Feed_All(link24g_parser_t *p, const uint8_t *buf, uint16_t len)
{
    int frames = 0;
    uint16_t i;

    for (i = 0U; i < len; i++) {
        if (Link24g_Parser_Feed(p, buf[i])) {
            frames++;
        }
    }
    return frames;
}

static uint16_t Make_Cmd(uint8_t *frame, uint8_t seq, uint8_t cmd, int16_t a0, int16_t a1, uint16_t age)
{
    link24g_cmd_t c = { 0x5C, seq, cmd, a0, a1, age };
    uint8_t pl[LINK24G_CMD_PAYLOAD_LEN];

    Link24g_Cmd_Pack(&c, pl);
    return Link24g_Frame_Encode((uint8_t)LINK24G_MSG_CMD, seq, pl, LINK24G_CMD_PAYLOAD_LEN, frame);
}

int main(void)
{
    uint8_t frame[LINK24G_FRAME_MAX];
    uint8_t buf[128];
    uint16_t n;
    link24g_parser_t p;
    link24g_cmd_t c;
    link24g_status_t s_in, s_out;
    uint8_t pl[LINK24G_STAT_PAYLOAD_LEN];
    const uint8_t check_str[] = "123456789";

    printf("1. CRC16-CCITT(0xFFFF) 标准校验值\n");
    CHECK(Link24g_Crc16(check_str, 9U) == 0x29B1U, "\"123456789\" 应为 0x29B1");

    printf("2. 命令往返\n");
    n = Make_Cmd(frame, 7U, LINK24G_CMD_SET_MODE, 3, -250, 1234U);
    CHECK(n == LINK24G_HEAD_LEN + LINK24G_CMD_PAYLOAD_LEN + 2U, "命令帧长度 15 B");
    Link24g_Parser_Init(&p);
    CHECK(Feed_All(&p, frame, n) == 1, "完整帧应解析出 1 帧");
    CHECK(Link24g_Cmd_Unpack(&p, &c) && (c.seq == 7U) && (c.session == 0x5CU) &&
          (c.cmd == LINK24G_CMD_SET_MODE) && (c.arg0 == 3) && (c.arg1 == -250) && (c.age_ms == 1234U),
          "字段原样还原(含负数)");
    CHECK(!Link24g_Status_Unpack(&p, &s_out), "命令帧不能按状态帧解包");

    printf("3. 状态往返\n");
    memset(&s_in, 0, sizeof(s_in));
    s_in.ack_session = 0x5C; s_in.ack_seq = 7U; s_in.state = LINK24G_BASE_MOVING;
    s_in.mode = LINK24G_MODE_END_MOVE; s_in.flags = 0x0BU; s_in.fault = 2U;
    s_in.pos_mm = -279; s_in.target_mm = 133; s_in.last_move_ms = 587U;
    s_in.start_lag_ms = -3; s_in.moves = 42U;
    Link24g_Status_Pack(&s_in, pl);
    n = Link24g_Frame_Encode((uint8_t)LINK24G_MSG_STATUS, 0U, pl, LINK24G_STAT_PAYLOAD_LEN, frame);
    Link24g_Parser_Init(&p);
    CHECK((Feed_All(&p, frame, n) == 1) && Link24g_Status_Unpack(&p, &s_out) &&
          (memcmp(&s_in, &s_out, sizeof(s_in)) == 0), "状态全部字段原样还原");

    printf("4. 字节流：前置垃圾、拆包、粘包\n");
    Link24g_Parser_Init(&p);
    buf[0] = 0x00; buf[1] = 0xA5; buf[2] = 0x13; buf[3] = 0xA5;
    n = Make_Cmd(&buf[4], 1U, LINK24G_CMD_STOP, 0, 0, 0U);
    n = (uint16_t)(n + 4U);
    n = (uint16_t)(n + Make_Cmd(&buf[n], 2U, LINK24G_CMD_GOTO_MM, 100, 0, 0U));
    CHECK((Feed_All(&p, buf, 9U) == 0) && (Feed_All(&p, &buf[9], (uint16_t)(n - 9U)) == 2),
          "垃圾后两帧粘连、任意处拆开，仍解析出 2 帧");
    CHECK(Link24g_Cmd_Unpack(&p, &c) && (c.seq == 2U) && (c.arg0 == 100), "最后一帧为 GOTO 100");

    printf("5. 0xA5 0xA5 0x5A 重同步\n");
    Link24g_Parser_Init(&p);
    buf[0] = 0xA5;
    n = Make_Cmd(&buf[1], 3U, LINK24G_CMD_ZERO, 0, 0, 0U);
    CHECK(Feed_All(&p, buf, (uint16_t)(n + 1U)) == 1, "多一个 0xA5 不丢帧");

    printf("6. 错误帧丢弃\n");
    Link24g_Parser_Init(&p);
    n = Make_Cmd(frame, 4U, LINK24G_CMD_RELOCATE, 0, 0, 0U);
    frame[7] ^= 0x01U;
    CHECK((Feed_All(&p, frame, n) == 0) && (p.crc_err == 1U), "载荷翻转 1 位应判 CRC 错");
    frame[7] ^= 0x01U;
    CHECK(Feed_All(&p, frame, n) == 1, "CRC 错后下一帧正常");
    Link24g_Parser_Init(&p);
    buf[0] = 0xA5; buf[1] = 0x5A; buf[2] = 0x10; buf[3] = 0; buf[4] = 200;
    n = Make_Cmd(&buf[5], 5U, LINK24G_CMD_STOP, 0, 0, 0U);
    CHECK((Feed_All(&p, buf, (uint16_t)(n + 5U)) == 1) && (p.len_err == 1U),
          "长度 200 应丢弃并继续解析后续帧");

    printf("7. 非法命令号\n");
    Link24g_Parser_Init(&p);
    n = Make_Cmd(frame, 6U, (uint8_t)LINK24G_CMD_NUM, 0, 0, 0U);
    CHECK((Feed_All(&p, frame, n) == 1) && !Link24g_Cmd_Unpack(&p, &c), "超范围命令号应拒绝");

    printf("\n%s：%d 项失败\n", (s_fail == 0) ? "全部通过" : "存在失败", s_fail);
    return (s_fail == 0) ? 0 : 1;
}
