/**
 ******************************************************************************
 * @file    link24g_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   镖架端 2.4G 测试基地同步(DL20 无线串口，USART1)
 * @version V1.0
 * @date    2026-11-05
 ******************************************************************************
 * @attention
 *  1. 硬件：DL20 点对点 A/B 端，115200 8N1，接 USART1(PA9 TX / PB7 RX)，
 *     与串口屏分时复用：DART_LINK24G_ENABLE 与 DART_SCREEN_ENABLE 不能同时为 1。
 *     只在测试时使用，比赛拆除；DART_TEST_MODE=0 时禁止启用(dart_config.h 编译期检查)。
 *  2. 帧格式与命令见 link24g_frame.h，基地工程 RP27_DartBase 使用同一份。
 *  3. 线程：Link24g_Rx 在 USART1 空闲中断中只拷贝字节；
 *     Link24g_Send_Base_Cmd 可在 ControlTask 或 CommunityTask 调用，只入队不发送；
 *     实际收发、重发、回执在 CommunityTask 的 Link24g_Poll 中完成。
 *  4. 可靠性：命令带(会话号, 序号)，未收到基地回执则每 LINK24G_RESEND_MS 重发，
 *     最多 LINK24G_MAX_TRIES 次；基地按序号去重，不会重复执行。STOP 入队时清空队列并插到队首。
 *     出镖同步每次重发都更新 age_ms，基地据此反推释放时刻。
 *  5. 回执只表示基地已收到并接受命令，不代表已经移动到位；到位看 g_link24g_base。
 *  6. 调试：测试时用调试器改写 g_link24g_ctrl.req 下发命令(见 link24g_req_e)，
 *     执行后 req 自动清零；基地状态镜像在 g_link24g_base。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __LINK24G_PROTOCOL_H
#define __LINK24G_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "link24g_frame.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  调试器请求；写入 g_link24g_ctrl.req 后由 Poll 执行并清零
 */
typedef enum {
    LINK24G_REQ_NONE = 0,      /* 无请求 */
    LINK24G_REQ_SET_MODE,      /* 按 mode/fixed_mm 设置靶型 */
    LINK24G_REQ_ZERO,          /* 基地找零 */
    LINK24G_REQ_GOTO,          /* 移到 goto_mm */
    LINK24G_REQ_RELOCATE,      /* 随机换位(随机固定靶) */
    LINK24G_REQ_STOP,          /* 停止 */
    LINK24G_REQ_SYNC_TEST,     /* 不发射，模拟一次出镖同步 */
} link24g_req_e;

/**
 * @brief  调试器控制块(测试时用 Watch/MemRW 改写)
 */
typedef struct {
    volatile uint8_t req;      /* link24g_req_e，执行后清零 */
    uint8_t  mode;             /* link24g_target_mode_e */
    int16_t  fixed_mm;         /* 固定靶位置 mm，±280 */
    int16_t  goto_mm;          /* GOTO 目标 mm，±280 */
    uint8_t  sync_enable;      /* 1：释放时自动发出镖同步 */
} link24g_ctrl_t;

/**
 * @brief  基地状态镜像与链路统计
 */
typedef struct {
    link24g_status_t st;       /* 最近一次基地状态 */
    uint32_t rx_tick;          /* 收到状态的时刻 ms */
    bool     online;           /* 状态未超时 */
    uint8_t  session;          /* 本机会话号 */
    uint8_t  queue_len;        /* 待回执命令数 */
    uint32_t tx_frames;        /* 已发命令帧(含重发) */
    uint32_t acked;            /* 收到回执的命令数 */
    uint32_t dropped;          /* 重发超限丢弃的命令数 */
    uint32_t queue_full;       /* 入队失败次数 */
    uint32_t crc_err;          /* 接收 CRC 错误帧 */
    uint32_t ring_overflow;    /* 接收缓冲溢出字节 */
    uint32_t last_sync_tick;   /* 最近一次出镖同步入队 ms */
} link24g_base_view_t;

/* Exported variables --------------------------------------------------------*/
extern link24g_ctrl_t      g_link24g_ctrl;
extern link24g_base_view_t g_link24g_base;

/* Exported functions --------------------------------------------------------*/
void Link24g_Init(void);
void Link24g_Rx(const uint8_t *buf, uint16_t len);
bool Link24g_Send_Base_Cmd(link24g_cmd_e cmd, int16_t arg0, int16_t arg1);
void Link24g_Notify_Shot(uint8_t shot_no);
void Link24g_Poll(uint32_t now);
bool Link24g_Is_Online(void);

#endif /* __LINK24G_PROTOCOL_H */
