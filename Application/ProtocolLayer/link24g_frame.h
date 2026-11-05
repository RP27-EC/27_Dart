/**
 ******************************************************************************
 * @file    link24g_frame.h
 * @author  RobotPilots 镖架电控
 * @brief   镖架 <-> 测试基地 2.4G 帧格式与编解码(纯头文件，无 HAL 依赖)
 * @version V1.0
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 镖架工程(RP27_Dart)与基地工程(RP27_DartBase)使用同一份本文件，
 *     修改时两边必须同步，并同时修改 LINK24G_PROTO_VERSION。
 *  2. 链路：DL20 点对点透传，UART 115200 8N1。DL20 会把串口数据打包后再无线发送，
 *     到达时刻不规则，所以本协议按字节流解析，不依赖串口空闲中断分帧。
 *  3. 帧格式(小端)：
 *       [0]=0xA5 [1]=0x5A [2]=消息类型 [3]=序号 [4]=载荷长度 n(<=16)
 *       [5..4+n]=载荷 [5+n..6+n]=CRC16，覆盖 [2..4+n]
 *     CRC16-CCITT：多项式 0x1021，初值 0xFFFF，不反射，无最终异或。
 *  4. 命令以 (会话号, 序号) 去重：同一命令重发不会被重复执行。
 *     会话号由镖架上电时生成，避免镖架复位后序号与基地记录的旧序号相同而被丢弃。
 *  5. 位置单位 mm，以基地找零后的中位为 0，范围 ±280，正方向由基地配置决定。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __LINK24G_FRAME_H
#define __LINK24G_FRAME_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Exported constants --------------------------------------------------------*/
#define LINK24G_PROTO_VERSION    (1U)     /* 协议版本 */
#define LINK24G_SOF0             (0xA5U)  /* 帧头 1 */
#define LINK24G_SOF1             (0x5AU)  /* 帧头 2 */
#define LINK24G_HEAD_LEN         (5U)     /* 帧头到长度字段 B */
#define LINK24G_PAYLOAD_MAX      (16U)    /* 载荷上限 B */
#define LINK24G_FRAME_MAX        (LINK24G_HEAD_LEN + LINK24G_PAYLOAD_MAX + 2U)
#define LINK24G_CMD_PAYLOAD_LEN  (8U)     /* 命令载荷 B */
#define LINK24G_STAT_PAYLOAD_LEN (16U)    /* 状态载荷 B */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  消息类型
 */
typedef enum {
    LINK24G_MSG_CMD    = 0x10,   /* 镖架 -> 基地：命令 */
    LINK24G_MSG_PING   = 0x11,   /* 镖架 -> 基地：空闲心跳，无载荷 */
    LINK24G_MSG_STATUS = 0x20,   /* 基地 -> 镖架：状态与回执 */
} link24g_msg_e;

/**
 * @brief  基地命令；arg0/arg1 含义见各项
 */
typedef enum {
    LINK24G_CMD_STOP = 0,      /* 停止并原地保持，取消待执行移动；最高优先级 */
    LINK24G_CMD_GOTO_MM,       /* 移到指定位置，arg0=位置 mm */
    LINK24G_CMD_SYNC_SHOT,     /* 镖架已释放，arg0=发序 1~4 */
    LINK24G_CMD_SET_MODE,      /* 设置靶型，arg0=link24g_target_mode_e，arg1=固定位 mm */
    LINK24G_CMD_ZERO,          /* 堵转找零，完成后回到 0 mm */
    LINK24G_CMD_RELOCATE,      /* 随机换位(随机固定靶开闸/命中后) */
    LINK24G_CMD_NUM,           /* 命令总数 */
} link24g_cmd_e;

/**
 * @brief  靶型(对应 2027 规则四种基地检测模块行为)
 */
typedef enum {
    LINK24G_MODE_FIXED = 0,      /* 固定：停在固定位，出镖不动 */
    LINK24G_MODE_RANDOM_FIXED,   /* 随机固定：收到 RELOCATE 才换位 */
    LINK24G_MODE_RANDOM_MOVE,    /* 随机移动：出镖即换位 */
    LINK24G_MODE_END_MOVE,       /* 末端移动：出镖 1.2 s 后换位 */
    LINK24G_MODE_NUM,            /* 靶型总数 */
} link24g_target_mode_e;

/**
 * @brief  基地运行状态
 */
typedef enum {
    LINK24G_BASE_UNZEROED = 0,   /* 未找零，不响应移动 */
    LINK24G_BASE_ZEROING,        /* 找零中 */
    LINK24G_BASE_IDLE,           /* 静止保持 */
    LINK24G_BASE_WAIT_START,     /* 已收到出镖，等待开始换位 */
    LINK24G_BASE_MOVING,         /* 换位中 */
    LINK24G_BASE_FAULT,          /* 故障，见 fault */
} link24g_base_state_e;

/**
 * @brief  基地状态标志位
 */
typedef enum {
    LINK24G_FLAG_ZEROED       = (1U << 0),  /* 零点有效 */
    LINK24G_FLAG_LAST_IN_TIME = (1U << 1),  /* 上次换位在时限内静止 */
    LINK24G_FLAG_PLAN_OVER    = (1U << 2),  /* 规划时间已超时限 */
    LINK24G_FLAG_SYNC_LATE    = (1U << 3),  /* 出镖同步到达已晚于开始时刻 */
} link24g_flag_e;

/**
 * @brief  命令内容(解码后)
 */
typedef struct {
    uint8_t  session;   /* 镖架会话号 1~255 */
    uint8_t  seq;       /* 命令序号 0~255 */
    uint8_t  cmd;       /* link24g_cmd_e */
    int16_t  arg0;      /* 参数 0 */
    int16_t  arg1;      /* 参数 1 */
    uint16_t age_ms;    /* 命令产生至本次发出 ms，最大 65535 */
} link24g_cmd_t;

/**
 * @brief  基地状态(解码后)
 */
typedef struct {
    uint8_t  ack_session;   /* 最近执行命令的会话号 */
    uint8_t  ack_seq;       /* 最近执行命令的序号 */
    uint8_t  state;         /* link24g_base_state_e */
    uint8_t  mode;          /* link24g_target_mode_e */
    uint8_t  flags;         /* link24g_flag_e 位图 */
    uint8_t  fault;         /* 基地故障码，0=无 */
    int16_t  pos_mm;        /* 当前位置 mm */
    int16_t  target_mm;     /* 当前/最近目标 mm */
    uint16_t last_move_ms;  /* 上次换位开始到静止 ms */
    int16_t  start_lag_ms;  /* 上次实际开始-应开始 ms */
    uint16_t moves;         /* 上电以来换位次数 */
} link24g_status_t;

/**
 * @brief  字节流解析器
 */
typedef struct {
    uint8_t  buf[LINK24G_FRAME_MAX]; /* 帧缓冲 */
    uint8_t  idx;                    /* 已收字节数 */
    uint8_t  need;                   /* 本帧总长，0=未知 */
    uint32_t crc_err;                /* CRC 错误帧数 */
    uint32_t len_err;                /* 长度非法帧数 */
    uint32_t ok;                     /* 有效帧数 */
} link24g_parser_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  CRC16-CCITT(0x1021，初值 0xFFFF)
 */
static inline uint16_t Link24g_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline void Link24g_Put_U16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t Link24g_Get_U16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief  组帧
 * @param  out 输出缓冲，至少 LINK24G_FRAME_MAX 字节
 * @retval 帧总长；载荷超长返回 0
 */
static inline uint16_t Link24g_Frame_Encode(uint8_t type, uint8_t seq,
                                            const uint8_t *payload, uint8_t len,
                                            uint8_t *out)
{
    uint16_t crc;

    if (len > LINK24G_PAYLOAD_MAX) {
        return 0U;
    }
    out[0] = LINK24G_SOF0;
    out[1] = LINK24G_SOF1;
    out[2] = type;
    out[3] = seq;
    out[4] = len;
    if (len > 0U) {
        memcpy(&out[LINK24G_HEAD_LEN], payload, len);
    }
    crc = Link24g_Crc16(&out[2], (uint16_t)(3U + len));
    Link24g_Put_U16(&out[LINK24G_HEAD_LEN + len], crc);
    return (uint16_t)(LINK24G_HEAD_LEN + len + 2U);
}

static inline void Link24g_Parser_Init(link24g_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/**
 * @brief  输入一个字节
 * @retval true 收到一帧有效帧，内容在 p->buf，长度 p->need
 * @note   CRC 或长度错误时丢弃，从下一字节重新找帧头。
 */
static inline bool Link24g_Parser_Feed(link24g_parser_t *p, uint8_t byte)
{
    uint16_t crc;

    if (p->idx == 0U) {
        if (byte == LINK24G_SOF0) {
            p->buf[p->idx++] = byte;
        }
        return false;
    }
    if (p->idx == 1U) {
        if (byte == LINK24G_SOF1) {
            p->buf[p->idx++] = byte;
        } else {
            p->idx = (byte == LINK24G_SOF0) ? 1U : 0U;   /* 0xA5 0xA5 0x5A 仍可同步 */
        }
        return false;
    }

    p->buf[p->idx++] = byte;
    if (p->idx == LINK24G_HEAD_LEN) {
        if (byte > LINK24G_PAYLOAD_MAX) {
            p->len_err++;
            p->idx = 0U;
            return false;
        }
        p->need = (uint8_t)(LINK24G_HEAD_LEN + byte + 2U);
        return false;
    }
    if ((p->idx < LINK24G_HEAD_LEN) || (p->idx < p->need)) {
        return false;
    }

    p->idx = 0U;
    crc = Link24g_Crc16(&p->buf[2], (uint16_t)(p->need - 4U));
    if (crc != Link24g_Get_U16(&p->buf[p->need - 2U])) {
        p->crc_err++;
        return false;
    }
    p->ok++;
    return true;
}

/** @brief 已解析帧的类型、序号、载荷 */
static inline uint8_t Link24g_Frame_Type(const link24g_parser_t *p)        { return p->buf[2]; }
static inline uint8_t Link24g_Frame_Seq(const link24g_parser_t *p)         { return p->buf[3]; }
static inline uint8_t Link24g_Frame_Len(const link24g_parser_t *p)         { return p->buf[4]; }
static inline const uint8_t *Link24g_Frame_Payload(const link24g_parser_t *p) { return &p->buf[LINK24G_HEAD_LEN]; }

/**
 * @brief  命令打包/解包；序号放在帧头，不在载荷中
 */
static inline void Link24g_Cmd_Pack(const link24g_cmd_t *c, uint8_t *pl)
{
    pl[0] = c->session;
    pl[1] = c->cmd;
    Link24g_Put_U16(&pl[2], (uint16_t)c->arg0);
    Link24g_Put_U16(&pl[4], (uint16_t)c->arg1);
    Link24g_Put_U16(&pl[6], c->age_ms);
}

static inline bool Link24g_Cmd_Unpack(const link24g_parser_t *p, link24g_cmd_t *c)
{
    const uint8_t *pl = Link24g_Frame_Payload(p);

    if ((Link24g_Frame_Type(p) != (uint8_t)LINK24G_MSG_CMD) ||
        (Link24g_Frame_Len(p) != LINK24G_CMD_PAYLOAD_LEN)) {
        return false;
    }
    c->seq     = Link24g_Frame_Seq(p);
    c->session = pl[0];
    c->cmd     = pl[1];
    c->arg0    = (int16_t)Link24g_Get_U16(&pl[2]);
    c->arg1    = (int16_t)Link24g_Get_U16(&pl[4]);
    c->age_ms  = Link24g_Get_U16(&pl[6]);
    return c->cmd < (uint8_t)LINK24G_CMD_NUM;
}

static inline void Link24g_Status_Pack(const link24g_status_t *s, uint8_t *pl)
{
    pl[0] = s->ack_session;
    pl[1] = s->ack_seq;
    pl[2] = s->state;
    pl[3] = s->mode;
    pl[4] = s->flags;
    pl[5] = s->fault;
    Link24g_Put_U16(&pl[6], (uint16_t)s->pos_mm);
    Link24g_Put_U16(&pl[8], (uint16_t)s->target_mm);
    Link24g_Put_U16(&pl[10], s->last_move_ms);
    Link24g_Put_U16(&pl[12], (uint16_t)s->start_lag_ms);
    Link24g_Put_U16(&pl[14], s->moves);
}

static inline bool Link24g_Status_Unpack(const link24g_parser_t *p, link24g_status_t *s)
{
    const uint8_t *pl = Link24g_Frame_Payload(p);

    if ((Link24g_Frame_Type(p) != (uint8_t)LINK24G_MSG_STATUS) ||
        (Link24g_Frame_Len(p) != LINK24G_STAT_PAYLOAD_LEN)) {
        return false;
    }
    s->ack_session  = pl[0];
    s->ack_seq      = pl[1];
    s->state        = pl[2];
    s->mode         = pl[3];
    s->flags        = pl[4];
    s->fault        = pl[5];
    s->pos_mm       = (int16_t)Link24g_Get_U16(&pl[6]);
    s->target_mm    = (int16_t)Link24g_Get_U16(&pl[8]);
    s->last_move_ms = Link24g_Get_U16(&pl[10]);
    s->start_lag_ms = (int16_t)Link24g_Get_U16(&pl[12]);
    s->moves        = Link24g_Get_U16(&pl[14]);
    return true;
}

#endif /* __LINK24G_FRAME_H */
