/*************************************************************************************************
 * ConfigChip_CmpssInit.c  —  CMPSS1/2/4 硬件快保护
 *
 * 审查要点落实：
 *   1) DCAH 必须 select COMBINATION，否则 TRIPIN4 组合进不了 DCAH
 *      → Epwm_AttachTrip4Ost() 中新增 EPWM_selectDigitalCompareTripInput()
 *   2) OVP 只配 HP mux，不配 LP（避免 HP 编号误配 LP 表）
 *      → Cmpss_InitHighOnly() 删除 lpMuxSel 参数
 *   3) 滤波：OVP prd=100 窗32/门限24（≈16~22 µs）；
 *            OCP prd=11  窗10/门限6 （≈0.48~0.80 µs，抑开关噪声）
 *   4) CMPSS 跳闸源：DabCmpss_ReadTripSrc() 一次读回位图，Fault 侧共用置位
 *   5) INSRC_DAC|INV_INVERTED：本库 INSRC=0、INV=0x2，可 OR
 *************************************************************************************************/
#include "Includes.h"          // 工程总头文件（芯片寄存器、driverlib、类型定义）
#include "ConfigChip_CmpssInit.h"     // 对外声明 + DAC阈值宏 + 滤波分频宏 + 故障源位定义

/* =============================================================================================
 * 函数：Cmpss_ConfigFilterHighLocal
 * 作用：配置"高比较器"数字滤波器（driverlib cmpss.c 未链入，本地实现）
 *
 * 滤波原理（多数表决）：
 *   每隔 Ts = (prescale+1)/SYSCLK 采样一次比较器输出
 *   在最近 sampleWindow 次采样中，≥ threshold 次为"1" → CTRIPH 输出"1"
 *
 * 参数：
 *   base           CMPSS 模块基地址
 *   samplePrescale 采样时钟预分频（0=不分频；越大越慢越抗干扰）
 *   sampleWindow   窗口大小 1~32
 *   threshold      确认门限（必须 > window/2）
 * ============================================================================================= */
static void Cmpss_ConfigFilterHighLocal(uint32_t base, uint32_t samplePrescale,
                                        uint16_t sampleWindow, uint16_t threshold)
{
    uint16_t regValue;          // 拼好的"窗口+阈值"16位寄存器值

    // 寄存器存 N-1：窗口→SAMPWIN位段，阈值→THRESH位段
    regValue = (uint16_t)(((sampleWindow - 1U) << CMPSS_CTRIPHFILCTL_SAMPWIN_S) |
                          ((threshold - 1U) << CMPSS_CTRIPHFILCTL_THRESH_S));

    EALLOW;                     // 解除写保护

    // 读-改-写：&~掩码清旧值，| 新值，其它位（如 FILINIT）不变
    HWREGH(base + CMPSS_O_CTRIPHFILCTL) =
        (HWREGH(base + CMPSS_O_CTRIPHFILCTL) &
         ~(CMPSS_CTRIPHFILCTL_SAMPWIN_M | CMPSS_CTRIPHFILCTL_THRESH_M)) |
        regValue;

    // 分频低16位（采样周期 = (此值+1) 个 SYSCLK）
    HWREGH(base + CMPSS_O_CTRIPHFILCLKCTL)  = (uint16_t)samplePrescale;
    // 分频高16位（预分频位宽 >16 位，拆两个寄存器）
    HWREGH(base + CMPSS_O_CTRIPHFILCLKCTL2) = (uint16_t)(samplePrescale >> 16U);

    EDIS;                       // 恢复写保护
}

/* =============================================================================================
 * 函数：Cmpss_ConfigFilterLowLocal
 * 作用：配置"低比较器"数字滤波器（与 High 对称，寄存器地址不同）
 * 说明：双极性电流窗口中，低比较器也需要滤波，防止负向噪声误触发
 * ============================================================================================= */
static void Cmpss_ConfigFilterLowLocal(uint32_t base, uint32_t samplePrescale,
                                       uint16_t sampleWindow, uint16_t threshold)
{
    uint16_t regValue;

    // 注意用 CTRIPL（Low）的位段定义
    regValue = (uint16_t)(((sampleWindow - 1U) << CMPSS_CTRIPLFILCTL_SAMPWIN_S) |
                          ((threshold - 1U) << CMPSS_CTRIPLFILCTL_THRESH_S));

    EALLOW;

    HWREGH(base + CMPSS_O_CTRIPLFILCTL) =
        (HWREGH(base + CMPSS_O_CTRIPLFILCTL) &
         ~(CMPSS_CTRIPLFILCTL_SAMPWIN_M | CMPSS_CTRIPLFILCTL_THRESH_M)) |
        regValue;

    HWREGH(base + CMPSS_O_CTRIPLFILCLKCTL)  = (uint16_t)samplePrescale;
    HWREGH(base + CMPSS_O_CTRIPLFILCLKCTL2) = (uint16_t)(samplePrescale >> 16U);

    EDIS;
}

/* =============================================================================================
 * 函数：Cmpss_InitHighOnly
 * 作用：初始化"单向上限"比较器（仅高比较器参与保护）
 * 适用：母线过压（电压只往高跑，不需要下限）
 *
 * ★ 本版修正：删除 lpMuxSel 参数，不再调用 ASysCtl_selectCMPLPMux()
 *   原因：HP 和 LP 的通道映射表不同（如 HP5≠LP5），用 HP 编号配 LP 是错的
 *   低比较器保持复位默认状态即可，反正 DACL=0 不会触发
 *
 * 参数：
 *   base      CMPSS 基地址
 *   hpMuxSel  高比较器输入复用器（ASYSCTL_CMPHPMUX_SELECT_x）
 *   hpMuxVal  复用器通道号（HP5→5, HP4→4）
 *   dacHigh   高阈值 DAC 码（0~4095）
 *   filtPrd   滤波采样分频（VBUS 用 DAB_CMPSS_VBUS_OVP_FILT_PRD=100）
 * ============================================================================================= */
static void Cmpss_InitHighOnly(uint32_t base,
                               ASysCtl_CMPHPMuxSelect hpMuxSel,
                               uint32_t hpMuxVal,
                               Uint16 dacHigh,
                               uint32_t filtPrd)
{
    // ① 选输入引脚：高比较器正端接到第 hpMuxVal 号引脚（如 HP5 → B5）
    ASysCtl_selectCMPHPMux(hpMuxSel, hpMuxVal);
    // ★ 不再配 LP mux！避免 HP 编号误配 LP 映射表

    // ② 比较器负端 = 内部 DAC
    CMPSS_configHighComparator(base, CMPSS_INSRC_DAC);
    // 低比较器也设为 DAC 源（保持确定状态），但 DACL=0 不会触发
    CMPSS_configLowComparator(base, CMPSS_INSRC_DAC);

    // ③ DAC 配置：SYSCLK 同步 + 影子寄存器（在线改阈值无毛刺）
    CMPSS_configDACHigh(base, CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);
    CMPSS_configDACLow(base, CMPSS_DACSRC_SHDW);
    // 高阈值：Vin > dacHigh → 触发
    CMPSS_setDACValueHigh(base, dacHigh);
    // 低阈值 = 0：Vin < 0V 才触发，实际不可能 → 低比较器永远不动作
    CMPSS_setDACValueLow(base, 0U);

    // ④ 高比较器滤波：窗32/门限24（OVP 专用，偏稳）
    Cmpss_ConfigFilterHighLocal(base, filtPrd, 32U, 24U);
    CMPSS_initFilterHigh(base);     // 初始化滤波器状态

    // ⑤ 输出：高路用滤波结果送 XBAR；低路异步直通（反正不触发）
    CMPSS_configOutputsHigh(base, CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base, CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_ASYNC_COMP);
    CMPSS_setHysteresis(base, 0U);  // 迟滞=0（滤波器已抗抖动）

    // ⑥ 禁止 PWMSYNC 自动复位（故障锁存直到软件清除）
    CMPSS_disableLatchResetOnPWMSYNCHigh(base);
    CMPSS_disableLatchResetOnPWMSYNCLow(base);
    // 消隐关闭（保持实时检测）
    CMPSS_configBlanking(base, 1U);
    CMPSS_disableBlanking(base);

    // ⑦ 最后使能 + 延时等 DAC 稳定
    CMPSS_enableModule(base);
    DEVICE_DELAY_US(500);
}

/* =============================================================================================
 * 函数：Cmpss_InitBipolarOcp
 * 作用：初始化"双极性过流窗口"（高/低比较器同时参与保护）
 * 适用：电流 ±30 A（IDD1/IDD2），正负方向都要检测
 *
 * 工作原理：
 *   V_sense = V_offset + I × K
 *   高比较器：Vin > DACH(+30A) → CTRIPH = 1 → 正过流
 *   低比较器(取反)：Vin < DACL(-30A) → CTRIPL = 1 → 负过流
 *   XBAR 选 CTRIPH_OR_L：任一方向过流都触发
 *
 * 参数：
 *   dacPos   +30 A 阈值 DAC 码
 *   dacNeg   -30 A 阈值 DAC 码（数值 < dacPos）
 *   filtPrd  滤波分频（IDD=11）；窗/门限取 DAB_CMPSS_IDD_OCP_FILT_WIN/THR
 * ============================================================================================= */
static void Cmpss_InitBipolarOcp(uint32_t base,
                                 ASysCtl_CMPHPMuxSelect hpMuxSel,
                                 ASysCtl_CMPLPMuxSelect lpMuxSel,
                                 uint32_t muxVal,
                                 Uint16 dacPos,
                                 Uint16 dacNeg,
                                 uint32_t filtPrd)
{
    // ① 选输入引脚：高/低比较器正端接同一引脚（如 A5 同时给 HP5 和 LP5）
    ASysCtl_selectCMPHPMux(hpMuxSel, muxVal);
    // OCP 正负同脚：HP/LP 选同一模拟通道号（这里 HP 和 LP 的通道号碰巧相同）
    ASysCtl_selectCMPLPMux(lpMuxSel, muxVal);

    // ② 比较器负端 = 内部 DAC
    CMPSS_configHighComparator(base, CMPSS_INSRC_DAC);
    // ★ 低比较器取反：INSRC_DAC=0x0(bit0) | INV_INVERTED=0x2(bit1) = 0x2
    //   位段不冲突：bit[1:0]=10 → DAC 输入 + 输出取反
    //   效果：CTRIPL = (Vin < DACL) ? 1 : 0  →  负过流时触发
    CMPSS_configLowComparator(base, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);

    // ③ DAC 配置
    CMPSS_configDACHigh(base, CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);
    CMPSS_configDACLow(base, CMPSS_DACSRC_SHDW);
    CMPSS_setDACValueHigh(base, dacPos);   // +30 A 阈值
    CMPSS_setDACValueLow(base, dacNeg);    // -30 A 阈值

    // ④ 高/低都要滤波：窗10/门限6（10次中≥6次确认，≈0.48~0.80 µs @prd=11）
    //    防止开关噪声误触发正/负过流
    Cmpss_ConfigFilterHighLocal(base, filtPrd, DAB_CMPSS_IDD_OCP_FILT_WIN, DAB_CMPSS_IDD_OCP_FILT_THR);
    Cmpss_ConfigFilterLowLocal(base, filtPrd, DAB_CMPSS_IDD_OCP_FILT_WIN, DAB_CMPSS_IDD_OCP_FILT_THR);
    CMPSS_initFilterHigh(base);
    CMPSS_initFilterLow(base);

    // ⑤ 输出：高/低都用滤波结果送 XBAR（与 HighOnly 不同！低路也参与保护）
    CMPSS_configOutputsHigh(base, CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base, CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_setHysteresis(base, 0U);

    // ⑥ 禁止 PWMSYNC 自动复位 + 关闭消隐
    CMPSS_disableLatchResetOnPWMSYNCHigh(base);
    CMPSS_disableLatchResetOnPWMSYNCLow(base);
    CMPSS_configBlanking(base, 1U);
    CMPSS_disableBlanking(base);

    // ⑦ 使能 + 延时
    CMPSS_enableModule(base);
    DEVICE_DELAY_US(500);
}

/* =============================================================================================
 * 母线过压：CMPSS1，B5(HP5)，单向上限
 * 分频 DAB_CMPSS_VBUS_OVP_FILT_PRD=100 → 约 16~22 µs 确认（不传 lpMuxSel）
 * ============================================================================================= */
static void VmbusOvp_Cmpss1_Init(void)
{
    Cmpss_InitHighOnly(CMPSS1_BASE,
                       ASYSCTL_CMPHPMUX_SELECT_1,   // CMPSS1 高比较器复用器
                       5U,                          // HP5 → B5 脚
                       DAB_CMPSS_VBUS_OVP_DAC,      // 过压阈值 DAC 码
                       DAB_CMPSS_VBUS_OVP_FILT_PRD);// 滤波分频=100
}

/* 电流1 ±OCP：CMPSS2，A5；prd=11 + 窗10/门限6 → ≈0.48~0.80 µs */
static void Idd1Ocp_Cmpss2_Init(void)
{
    Cmpss_InitBipolarOcp(CMPSS2_BASE,
                         ASYSCTL_CMPHPMUX_SELECT_2,
                         ASYSCTL_CMPLPMUX_SELECT_2,
                         5U,                         // HP5/LP5 → A5
                         DAB_CMPSS_IDD_OCP_POS_DAC,  // +30 A
                         DAB_CMPSS_IDD_OCP_NEG_DAC,  // -30 A
                         DAB_CMPSS_IDD_OCP_FILT_PRD);// prd=11，窗/门限见 DAB_CMPSS_IDD_OCP_FILT_WIN/THR
}

/* 电流2 ±OCP：CMPSS4，A8；滤波同 IDD1 */
static void Idd2Ocp_Cmpss4_Init(void)
{
    Cmpss_InitBipolarOcp(CMPSS4_BASE,
                         ASYSCTL_CMPHPMUX_SELECT_4,
                         ASYSCTL_CMPLPMUX_SELECT_4,
                         4U,                         // HP4/LP4 → A8
                         DAB_CMPSS_IDD_OCP_POS_DAC,
                         DAB_CMPSS_IDD_OCP_NEG_DAC,
                         DAB_CMPSS_IDD_OCP_FILT_PRD);
}

/* =============================================================================================
 * 函数：DabCmpss_EpwmXbarInit
 * 作用：ePWM-XBAR TRIP4 配置——三路 CMPSS 信号"或"到 TRIP4
 *
 * MUX 编号与 CMPSS 对应（每个 CMPSS 占 2 个 MUX）：
 *   CMPSS1 → MUX0(CTRIPH) / MUX1(CTRIPL)
 *   CMPSS2 → MUX2(CTRIPH) / MUX3(CTRIPL)
 *   CMPSS4 → MUX6(CTRIPH) / MUX7(CTRIPL)
 *
 * MUX 值：0 = 仅 CTRIPH；1 = CTRIPH_OR_L
 *   OVP 单向上限 → 0（只需 CTRIPH）
 *   OCP 双极性   → 1（必须 OR_L，否则负过流检测不到）
 * ============================================================================================= */
static void DabCmpss_EpwmXbarInit(void)
{
    EALLOW;

    // --- 先清全部使能：软跳（Boot → App 不复位）时残留的 MUX 位会混进 TRIP4 ---
    EPwmXbarRegs.TRIP4MUXENABLE.all = 0U;

    // --- 信号源选择 ---
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0U;   // CMPSS1：仅 CTRIPH（OVP）
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 1U;   // CMPSS2：CTRIPH_OR_L（±OCP）
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX6 = 1U;   // CMPSS4：CTRIPH_OR_L（±OCP）

    // --- 使能参与 TRIP4 的"或"运算 ---
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0 = 1U;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2 = 1U;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX6 = 1U;

    EDIS;
}

/* =============================================================================================
 * 函数：Epwm_AttachTrip4Ost
 * 作用：让一个 ePWM 模块响应 TRIP4 → A/B 一次性锁死为低
 *
 * ★ 本版关键修正：新增 EPWM_selectDigitalCompareTripInput()
 *   原因：DCAH 默认输入源是 TRIPIN1（单个），不调 select 则 TRIPIN4 的组合输入进不来
 *   修正后信号流：
 *     TRIP4(XBAR) → TRIPIN4 → 组合输入 → DCAH → DCAEVT1 → OST → A/B=LOW
 * ============================================================================================= */
static void Epwm_AttachTrip4Ost(uint32_t base)
{
    // ★ ① 先选 DCAH 的输入源 = "组合输入"（COMBINATION）
    //   缺这步则 DCAH 默认取 TRIPIN1，TRIPIN4 的信号根本到不了 DCAH！
    EPWM_selectDigitalCompareTripInput(base,
                                       EPWM_DC_TRIP_COMBINATION,
                                       EPWM_DC_TYPE_DCAH);

    // ② 把 TRIPIN4（即 XBAR TRIP4）加入 DCAH 的组合输入（组合 = 多路"或"）
    EPWM_enableDigitalCompareTripCombinationInput(base,
                                                  EPWM_DC_COMBINATIONAL_TRIPIN4,
                                                  EPWM_DC_TYPE_DCAH);

    // ③ 事件条件：DCAH 为高 → 产生 DCAEVT1
    EPWM_setTripZoneDigitalCompareEventCondition(base,
                                                 EPWM_TZ_DC_OUTPUT_A1,
                                                 EPWM_TZ_EVENT_DCXH_HIGH);

    // ④ DCAEVT1 信号源 = 原始信号（不经消隐/滤波，异步直达，响应 ≈ 几十 ns）
    EPWM_setDigitalCompareEventSource(base,
                                      EPWM_DC_MODULE_A,
                                      EPWM_DC_EVENT_1,
                                      EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

    // ⑤ DCAEVT1 → 一次性跳闸(OST)：触发后锁住，直到软件清除
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCAEVT1);

    // ⑥ 跳闸动作：A/B 强制低（封锁上下管，安全态）
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);

    // ⑦ 只清挂接过程可能产生的 DCAEVT1 标志，保留 SetupBase() 已锁定的 OST 硬封波
    EPWM_clearTripZoneFlag(base, EPWM_TZ_FLAG_DCAEVT1);
}

/* 8 路 ePWM 全部挂接 TRIP4（双桥各 4 路） */
static void DabCmpss_AttachAllDabEpwm(void)
{
    Epwm_AttachTrip4Ost(EPWM10_BASE);
    Epwm_AttachTrip4Ost(EPWM12_BASE);
    Epwm_AttachTrip4Ost(EPWM11_BASE);
    Epwm_AttachTrip4Ost(EPWM3_BASE);
    Epwm_AttachTrip4Ost(EPWM7_BASE);
    Epwm_AttachTrip4Ost(EPWM9_BASE);
    Epwm_AttachTrip4Ost(EPWM2_BASE);
    Epwm_AttachTrip4Ost(EPWM4_BASE);
}

/**
 * @brief 一次读三路 CMPSS STS → DAB_CMPSS_SRC_* 位图（不清硬件）
 *
 * HI/LO 的 LATCHFILTOUT|FILTOUT 任一为 1 即记该源；调用方负责置 soft 位。
 */
Uint16 DabCmpss_ReadTripSrc(void)
{
    Uint16 s1 = CMPSS_getStatus(CMPSS1_BASE);
    Uint16 s2 = CMPSS_getStatus(CMPSS2_BASE);
    Uint16 s4 = CMPSS_getStatus(CMPSS4_BASE);
    Uint16 src = 0U;

    if ((s1 & (CMPSS_STS_HI_LATCHFILTOUT | CMPSS_STS_HI_FILTOUT)) != 0U)
    {
        src |= DAB_CMPSS_SRC_VBUS_OVP;
    }
    if ((s2 & (CMPSS_STS_HI_LATCHFILTOUT | CMPSS_STS_HI_FILTOUT)) != 0U)
    {
        src |= DAB_CMPSS_SRC_IDD1_POS;
    }
    if ((s2 & (CMPSS_STS_LO_LATCHFILTOUT | CMPSS_STS_LO_FILTOUT)) != 0U)
    {
        src |= DAB_CMPSS_SRC_IDD1_NEG;
    }
    if ((s4 & (CMPSS_STS_HI_LATCHFILTOUT | CMPSS_STS_HI_FILTOUT)) != 0U)
    {
        src |= DAB_CMPSS_SRC_IDD2_POS;
    }
    if ((s4 & (CMPSS_STS_LO_LATCHFILTOUT | CMPSS_STS_LO_FILTOUT)) != 0U)
    {
        src |= DAB_CMPSS_SRC_IDD2_NEG;
    }

    return src;
}

/* =============================================================================================
 * 函数：DabCmpss_ClearTrip（对外接口）
 * 作用：清 CMPSS 滤波锁存 + ePWM TZ；不读软锁存（避免恢复时二次置位）
 * 注意：跳闸源采集/上报只由 Fault_CmpssSrc5ms() 完成。
 * ============================================================================================= */
void DabCmpss_ClearFilterLatch(void)
{
    DabCmpss_ClearFilterLatchSrc((Uint16)(DAB_CMPSS_SRC_VBUS_OVP |
                                          DAB_CMPSS_SRC_IDD1_POS | DAB_CMPSS_SRC_IDD1_NEG |
                                          DAB_CMPSS_SRC_IDD2_POS | DAB_CMPSS_SRC_IDD2_NEG));
}

void DabCmpss_ClearFilterLatchSrc(Uint16 u16Src)
{
    if ((u16Src & DAB_CMPSS_SRC_VBUS_OVP) != 0U)
    {
        CMPSS_clearFilterLatchHigh(CMPSS1_BASE);
    }
    if ((u16Src & DAB_CMPSS_SRC_IDD1_POS) != 0U)
    {
        CMPSS_clearFilterLatchHigh(CMPSS2_BASE);
    }
    if ((u16Src & DAB_CMPSS_SRC_IDD1_NEG) != 0U)
    {
        CMPSS_clearFilterLatchLow(CMPSS2_BASE);
    }
    if ((u16Src & DAB_CMPSS_SRC_IDD2_POS) != 0U)
    {
        CMPSS_clearFilterLatchHigh(CMPSS4_BASE);
    }
    if ((u16Src & DAB_CMPSS_SRC_IDD2_NEG) != 0U)
    {
        CMPSS_clearFilterLatchLow(CMPSS4_BASE);
    }
}

void DabCmpss_ClearTrip(void)
{
    /* 只清硬件锁存与 ePWM TZ；跳闸源由 DabCmpss_ReadTripSrc() 按需读取。 */
    DabCmpss_ClearFilterLatch();

    EPWM_clearTripZoneFlag(EPWM10_BASE, EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM12_BASE, EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM11_BASE, EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM3_BASE,  EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM7_BASE,  EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM9_BASE,  EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM2_BASE,  EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM4_BASE,  EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
}

/* =============================================================================================
 * 函数：InitChip_CmpssRegs（总入口，系统初始化时调用一次）
 * 流程：清零 → 比较器 → XBAR → ePWM → 清标志 → 再清零（忽略上电噪声）
 * ============================================================================================= */
void InitChip_CmpssRegs(void)
{
    // ① 三路比较器
    VmbusOvp_Cmpss1_Init();     // 母线过压（prd=100，窗32/门限24）
    Idd1Ocp_Cmpss2_Init();      // 电流1 ±30 A（prd=11，窗10/门限6）
    Idd2Ocp_Cmpss4_Init();      // 电流2 ±30 A（同 IDD1）

    // ② XBAR + ePWM 挂接
    DabCmpss_EpwmXbarInit();    // 三路 → TRIP4
    DabCmpss_AttachAllDabEpwm();// 8 路 ePWM 响应 TRIP4

    // ③ 只清上电残留 CMPSS 滤波锁存，保留 SetupBase() 已锁定的 ePWM OST 硬封波
    DabCmpss_ClearFilterLatch();
}
