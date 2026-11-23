/*************************************************************************************************
 * App_CurShare.h  —  多机并联均流（与 CAN / 串口解耦）
 *
 * 职责边界：
 *   本模块：对端 0x41/0x42 解包、本机 0x41/0x42 组包、均流电流、下垂/trim → voltAdj。
 *   App_CanComm：何时发、用哪条 ID/缓冲发；收到 0x41 后回调本模块。
 *   App_UartProto：本机逻辑地址重赋值（写 uIndex + bCalIDCmd），地址变更时 ResetPeers。
 *
 * 协议对齐 A17E1：WATCHID 广播，源地址=LogADDR，CMD=0x41 电流态 / 0x42 电压态。
 * 0x42 接口保留；均压/主从 20ms 任务当前未接入调度。
 *************************************************************************************************/
#ifndef APP_CUR_SHARE_H
#define APP_CUR_SHARE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 均流补偿：1ms 发布 voltAdj，供 ISR 叠到 Ramp.v */
typedef struct
{
    float voltAdj;   /* = trimOut − droop，加到母线电压给定 */
    float droop;     /* 有符号下垂 [V] */
    float trim;      /* 200ms 积分目标 [V] */
    float trimOut;   /* 1ms 限速后的 trim [V] */
    uint16_t workCount; /* ★ 200ms 统计的同向均流工作台数，含本机 */
} ST_CUR_SHARE;

extern ST_CUR_SHARE CurShare;

/** ★ 对端 0x41：已更新对端状态返回 1，拒收返回 0。 */
uint16_t CurShare_Rx41(uint16_t uAdr, const uint16_t *dat);

/** 对端 0x42：运行态 + 电池电压 + 主机标志 → 写入 uMdlAddMes，可下发 canCmd */
void CurShare_Rx42(uint16_t uAdr, const uint16_t *dat);

/**
 * 组本机 0x41 载荷到 dat[8]。
 * @return 1=可发（地址已锁定），0=跳过
 */
uint16_t CurShare_FillTx41(uint16_t *dat);

/**
 * 组本机 0x42 载荷到 dat[8]。
 * @return 1=可发（地址已锁定），0=跳过
 */
uint16_t CurShare_FillTx42(uint16_t *dat);

/** 50ms 节拍：内部满 200ms 做一次并联均流电流 + trim 积分（令牌法，0x41 发送 81ms） */
void CurShare_Task_50ms(void);

/**
 * 1ms：下垂 + trim → CurShare.voltAdj
 * @param f32AdjMin  voltAdj 下限 = Vmin − Vref（通常为负）
 * @param f32AdjMax  voltAdj 上限 = Vmax − Vref（通常为正）
 */
void CurShare_Update1ms(float f32AdjMin, float f32AdjMax);

/** 清补偿与均流标志（停机/复位） */
void CurShare_ResetComp(void);

/** 本机地址变更：清对端票 + 补偿 */
void CurShare_ResetPeers(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CUR_SHARE_H */
