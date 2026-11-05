/**
 ******************************************************************************
 * @file    link24g_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   镖架端 2.4G 测试基地同步实现
 * @version V1.0
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  线程划分见 link24g_protocol.h。接收缓冲为单生产者(中断)/单消费者(Poll)。
 *  命令入队可能来自 ControlTask(出镖同步)与 CommunityTask(调试请求)，
 *  入队、出队在临界区内完成。STOP 只由 CommunityTask 入队，与 Poll 同一任务，
 *  因此 Poll 读取队首期间队首不会被其它任务改写。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "link24g_protocol.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define LINK24G_RING_SIZE   (256U)   /* 接收环形缓冲 B，2 的幂 */
#define LINK24G_QUEUE_LEN   (8U)     /* 待回执命令上限 */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  待回执命令
 */
typedef struct {
    uint8_t  cmd;          /* link24g_cmd_e */
    uint8_t  seq;          /* 分配的序号 */
    int16_t  arg0;         /* 参数 0 */
    int16_t  arg1;         /* 参数 1 */
    uint32_t create_tick;  /* 入队时刻 ms */
    uint32_t last_tx;      /* 最近发送时刻 ms */
    uint8_t  tries;        /* 已发送次数 */
} link24g_pending_t;

/* Exported variables --------------------------------------------------------*/
link24g_ctrl_t g_link24g_ctrl = {
    .req         = LINK24G_REQ_NONE,
    .mode        = LINK24G_MODE_FIXED,
    .fixed_mm    = 0,
    .goto_mm     = 0,
    .sync_enable = 1U,
};
link24g_base_view_t g_link24g_base;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

static uint8_t           s_ring[LINK24G_RING_SIZE];
static volatile uint16_t s_ring_head;      /* 中断写 */
static volatile uint16_t s_ring_tail;      /* Poll 读 */
static link24g_parser_t  s_parser;

static link24g_pending_t s_queue[LINK24G_QUEUE_LEN];
static uint8_t           s_q_head;         /* 队首下标 */
static uint8_t           s_q_count;        /* 队列长度 */
static uint8_t           s_next_seq;       /* 下一个序号 */
static uint32_t          s_last_tx_tick;   /* 最近一次发送 ms */

/* Private functions ---------------------------------------------------------*/
#if DART_LINK24G_ENABLE
static uint8_t Link24g_Make_Session(void)
{
    const uint32_t *uid = (const uint32_t *)UID_BASE;
    uint32_t mix = uid[0] ^ uid[1] ^ uid[2] ^ HAL_GetTick() ^ SysTick->VAL;

    mix ^= mix >> 16;
    mix ^= mix >> 8;
    return ((mix & 0xFFU) == 0U) ? 1U : (uint8_t)mix;
}

/**
 * @brief  发送队首命令一次
 */
static void Link24g_Tx_Head(uint32_t now)
{
    link24g_pending_t *p = &s_queue[s_q_head];
    link24g_cmd_t c;
    uint8_t  payload[LINK24G_CMD_PAYLOAD_LEN];
    uint8_t  frame[LINK24G_FRAME_MAX];
    uint16_t len;
    uint32_t age = now - p->create_tick;

    c.session = g_link24g_base.session;
    c.seq     = p->seq;
    c.cmd     = p->cmd;
    c.arg0    = p->arg0;
    c.arg1    = p->arg1;
    c.age_ms  = (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;
    Link24g_Cmd_Pack(&c, payload);
    len = Link24g_Frame_Encode((uint8_t)LINK24G_MSG_CMD, c.seq, payload,
                               LINK24G_CMD_PAYLOAD_LEN, frame);
    /* NOTE: 阻塞发送，23 B 约 2 ms，CommunityTask 周期 10 ms 内可接受 */
    (void)HAL_UART_Transmit(&huart1, frame, len, LINK24G_TX_TIMEOUT_MS);
    p->last_tx = now;
    p->tries++;
    s_last_tx_tick = now;
    g_link24g_base.tx_frames++;
}

/**
 * @brief  空闲心跳，让基地知道镖架在线
 */
static void Link24g_Tx_Ping(uint32_t now)
{
    uint8_t  frame[LINK24G_FRAME_MAX];
    uint16_t len = Link24g_Frame_Encode((uint8_t)LINK24G_MSG_PING, 0U, NULL, 0U, frame);

    (void)HAL_UART_Transmit(&huart1, frame, len, LINK24G_TX_TIMEOUT_MS);
    s_last_tx_tick = now;
}

static void Link24g_Pop_Head(void)
{
    taskENTER_CRITICAL();
    if (s_q_count > 0U) {
        s_q_head = (uint8_t)((s_q_head + 1U) % LINK24G_QUEUE_LEN);
        s_q_count--;
    }
    taskEXIT_CRITICAL();
}

static void Link24g_Handle_Status(const link24g_status_t *st, uint32_t now)
{
    g_link24g_base.st      = *st;
    g_link24g_base.rx_tick = now;
    if ((s_q_count > 0U) &&
        (st->ack_session == g_link24g_base.session) &&
        (st->ack_seq == s_queue[s_q_head].seq)) {
        Link24g_Pop_Head();
        g_link24g_base.acked++;
    }
}

static void Link24g_Drain_Rx(uint32_t now)
{
    link24g_status_t st;
    uint8_t byte;

    while (s_ring_tail != s_ring_head) {
        byte = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) & (LINK24G_RING_SIZE - 1U));
        if (Link24g_Parser_Feed(&s_parser, byte) && Link24g_Status_Unpack(&s_parser, &st)) {
            Link24g_Handle_Status(&st, now);
        }
    }
    g_link24g_base.crc_err = s_parser.crc_err;
}

/**
 * @brief  执行调试器请求
 */
static void Link24g_Handle_Req(void)
{
    uint8_t req = g_link24g_ctrl.req;

    switch (req) {
        case LINK24G_REQ_SET_MODE:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_SET_MODE, (int16_t)g_link24g_ctrl.mode,
                                        g_link24g_ctrl.fixed_mm);
            break;
        case LINK24G_REQ_ZERO:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_ZERO, 0, 0);
            break;
        case LINK24G_REQ_GOTO:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_GOTO_MM, g_link24g_ctrl.goto_mm, 0);
            break;
        case LINK24G_REQ_RELOCATE:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_RELOCATE, 0, 0);
            break;
        case LINK24G_REQ_STOP:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_STOP, 0, 0);
            break;
        case LINK24G_REQ_SYNC_TEST:
            (void)Link24g_Send_Base_Cmd(LINK24G_CMD_SYNC_SHOT, 0, 0);
            break;
        default:
            break;
    }
    g_link24g_ctrl.req = LINK24G_REQ_NONE;
}
#endif /* DART_LINK24G_ENABLE */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化：生成会话号，清空队列与解析器
 */
void Link24g_Init(void)
{
    memset(&g_link24g_base, 0, sizeof(g_link24g_base));
    Link24g_Parser_Init(&s_parser);
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_q_head    = 0U;
    s_q_count   = 0U;
    s_next_seq  = 0U;
    s_last_tx_tick = 0U;
#if DART_LINK24G_ENABLE
    g_link24g_base.session = Link24g_Make_Session();
#endif
}

/**
 * @brief  USART1 接收入口(空闲中断上下文，只拷贝)
 * @param  buf 接收缓冲
 * @param  len 本次实际收到字节数
 */
void Link24g_Rx(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    for (i = 0U; i < len; i++) {
        next = (uint16_t)((s_ring_head + 1U) & (LINK24G_RING_SIZE - 1U));
        if (next == s_ring_tail) {
            g_link24g_base.ring_overflow++;
            continue;
        }
        s_ring[s_ring_head] = buf[i];
        s_ring_head = next;
    }
}

/**
 * @brief  命令入队(不发送)
 * @param  cmd  命令
 * @param  arg0 参数 0，含义见 link24g_cmd_e
 * @param  arg1 参数 1
 * @retval true 已入队；未启用、队列满时返回 false
 * @note   STOP 会清空队列并插到队首，保证停止优先。
 */
bool Link24g_Send_Base_Cmd(link24g_cmd_e cmd, int16_t arg0, int16_t arg1)
{
#if DART_LINK24G_ENABLE
    link24g_pending_t *p;
    uint8_t slot;
    bool ok = true;

    if ((uint8_t)cmd >= (uint8_t)LINK24G_CMD_NUM) {
        return false;
    }
    taskENTER_CRITICAL();
    if (cmd == LINK24G_CMD_STOP) {
        s_q_count = 0U;   /* 丢弃未回执命令 */
    }
    if (s_q_count >= LINK24G_QUEUE_LEN) {
        ok = false;
    } else {
        slot = (uint8_t)((s_q_head + s_q_count) % LINK24G_QUEUE_LEN);
        p = &s_queue[slot];
        p->cmd         = (uint8_t)cmd;
        p->seq         = s_next_seq++;
        p->arg0        = arg0;
        p->arg1        = arg1;
        p->create_tick = HAL_GetTick();
        p->last_tx     = 0U;
        p->tries       = 0U;
        s_q_count++;
        if (cmd == LINK24G_CMD_SYNC_SHOT) {
            g_link24g_base.last_sync_tick = p->create_tick;
        }
    }
    taskEXIT_CRITICAL();
    if (!ok) {
        g_link24g_base.queue_full++;
    }
    return ok;
#else
    (void)cmd;
    (void)arg0;
    (void)arg1;
    return false;
#endif
}

/**
 * @brief  释放事件钩子，由 dart_fsm 在发出释放命令时调用
 * @param  shot_no 发序 1~4
 */
void Link24g_Notify_Shot(uint8_t shot_no)
{
    if (g_link24g_ctrl.sync_enable != 0U) {
        (void)Link24g_Send_Base_Cmd(LINK24G_CMD_SYNC_SHOT, (int16_t)shot_no, 0);
    }
}

/**
 * @brief  周期处理，CommunityTask 每 10ms 调用：收状态、处理请求、发送/重发
 * @param  now 当前时刻 ms
 */
void Link24g_Poll(uint32_t now)
{
#if DART_LINK24G_ENABLE
    link24g_pending_t *p;

    Link24g_Drain_Rx(now);
    Link24g_Handle_Req();

    g_link24g_base.online = (g_link24g_base.rx_tick != 0U) &&
                            !Dart_Time_Elapsed(now, g_link24g_base.rx_tick, LINK24G_ONLINE_TIMEOUT_MS);

    if (s_q_count > 0U) {
        p = &s_queue[s_q_head];
        if ((p->tries == 0U) || Dart_Time_Elapsed(now, p->last_tx, LINK24G_RESEND_MS)) {
            if (p->tries >= LINK24G_MAX_TRIES) {
                Link24g_Pop_Head();
                g_link24g_base.dropped++;
            } else {
                Link24g_Tx_Head(now);
            }
        }
    } else if (Dart_Time_Elapsed(now, s_last_tx_tick, LINK24G_PING_PERIOD_MS)) {
        Link24g_Tx_Ping(now);
    }
    g_link24g_base.queue_len = s_q_count;
#else
    (void)now;
#endif
}

/**
 * @brief  基地是否在线(LINK24G_ONLINE_TIMEOUT_MS 内收到过状态)
 */
bool Link24g_Is_Online(void)
{
    return g_link24g_base.online;
}
