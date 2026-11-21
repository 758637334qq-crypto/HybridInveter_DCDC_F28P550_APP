/*********************************************************************************
* File:        App_BootProto.h
* Description: ARM 升级协议解析（原始工程 SCIB 内部通讯协议的 boot 部分，独立成文件）
*              与 UartProto（AA55 业务协议）共存；原 UartProto 0x2F 升级握手已移除，
*              本模块为唯一升级入口。
***********************************************************************************/

#ifndef APP_BOOTPROTO_H_
#define APP_BOOTPROTO_H_

#include <stdint.h>

/*------------------------------------------------------------------------------
* 原始工程协议参数（v102：Task_InterComm.h / Task_InterComm_NormalFunc.h）
*------------------------------------------------------------------------------*/
#define BOOT_PROTO_SLAVE_ADDR       (0xD2U)     /* SLAVE_DSP_COM_ADDR：DSP 从机地址 */
#define BOOT_PROTO_FUNC_CODE        (0x01U)     /* INTER_COM_FUCTIONCODE：功能码 */
#define BOOT_PROTO_RX_DATA_LEN      (90U)       /* INTER_COM_RX_DATA_LEN：数据域长度 */
#define BOOT_PROTO_FRAME_LEN        (BOOT_PROTO_RX_DATA_LEN + 6U)   /* 96 字节定长帧，无 CRC */
#define BOOT_PROTO_CMD_FLASH_OFFS   (9U)        /* 帧[9]：在线烧录命令字节 */
#define BOOT_PROTO_CMD_UPDATE       (3U)        /* S_DSP_UPDATE：允许跳 Boot */

#define BOOT_PROTO_SILENCE_TICKS    (4U)        /* 4×1ms=4ms 无字节 → 断帧（原 35×100us=3.5ms，RX 轮询移 1ms 任务后按 1ms 拍取整） */
#define BOOT_PROTO_READY_TICKS_1MS  (2000U)     /* FlashReady 持续 2000×1ms=2s 确认后才跳转（对齐 v102：原 PVBoostFlashMode 在 SuperTask 10ms 节拍内数 200×10ms=2s） */

typedef struct
{
    /* 升级状态标志（原始 stInterCommMessage 同名位，供其它模块查询） */
    volatile uint16_t u16FlashUpdateFlag;   /* ARM 已下发升级命令 */
    volatile uint16_t u16FlashReady;        /* 允许跳 Boot（S_DSP_UPDATE） */

    /* 1ms 帧状态机（只在本任务上下文读写） */
    volatile uint16_t u16State;             /* 0=等地址 1=等功能码 2=收帧体 */
    volatile uint16_t u16SilenceCnt;        /* 字节间隔计数（1ms 拍，>4ms 断帧） */
    volatile uint16_t u16ByteIdx;
    uint8_t  u8FlashCmd;                    /* 暂存帧[9]（全帧唯一被读取的字节） */

    /* 1ms 跳转确认 */
    volatile uint16_t u16ReadyCnt;          /* FlashReady 持续计数（1ms 拍） */
    volatile uint16_t u16PwmDisabled;       /* 升级态已封波标志 */

    /* 帧防误触发：原始协议无 CRC，与 AA55 协议共存时要求帧[9] 连续两帧一致才更新标志 */
    volatile uint16_t u16LastCmd;           /* 上一帧 [9] 值 */
    volatile uint16_t u16CmdRepeat;         /* [9] 连续一致帧数 */

    /* 诊断计数 */
    volatile uint16_t u16FrameCnt;          /* 收到合法帧数 */
    volatile uint16_t u16UpdateCmdCnt;      /* 烧录命令帧数 */
} BootProtoCtrl_t;

extern BootProtoCtrl_t g_BootProto;

void BootProto_Init(void);          /* 清状态机/标志（main 调一次） */
void BootProto_Task_Rx(void);       /* 静默断帧检查（1ms；必须排在 UartProto_RxTask 喂字节之前） */
void BootProto_RxByte(uint16_t byte);   /* 升级协议字节入口：0xD2 → 0x01 → 定长 96 收集 → 解析帧[9] */
void BootProto_Task_1ms(void);      /* 升级态边沿封波 + FlashReady 2s 确认 → 跳 Boot */

#endif /* APP_BOOTPROTO_H_ */
