/*************************************************************************************************
 * ConfigChip_DabPwmMap.c  —  与 ConfigChip_DabPwmMap.h 配套：双桥影子镜像 / 分桥 GLD
 *************************************************************************************************/
#include "ConfigChip_DabPwmMap.h"

void DabPwm_MirrorShadowBridge2(void)
{
    DabEpwm1bRegs.CMPA.all = DabEpwm1Regs.CMPA.all;
    DabEpwm1bRegs.CMPB.all = DabEpwm1Regs.CMPB.all;
    DabEpwm2bRegs.CMPA.all = DabEpwm2Regs.CMPA.all;
    DabEpwm2bRegs.CMPB.all = DabEpwm2Regs.CMPB.all;
    DabEpwm3bRegs.CMPA.all = DabEpwm3Regs.CMPA.all;
    DabEpwm3bRegs.CMPB.all = DabEpwm3Regs.CMPB.all;
    DabEpwm4bRegs.CMPA.all = DabEpwm4Regs.CMPA.all;
    DabEpwm4bRegs.CMPB.all = DabEpwm4Regs.CMPB.all;

    DabEpwm1bRegs.AQCTLA.all = DabEpwm1Regs.AQCTLA.all;
    DabEpwm1bRegs.AQCTLB.all = DabEpwm1Regs.AQCTLB.all;
    DabEpwm2bRegs.AQCTLA.all = DabEpwm2Regs.AQCTLA.all;
    DabEpwm2bRegs.AQCTLB.all = DabEpwm2Regs.AQCTLB.all;
    DabEpwm3bRegs.AQCTLA.all = DabEpwm3Regs.AQCTLA.all;
    DabEpwm3bRegs.AQCTLB.all = DabEpwm3Regs.AQCTLB.all;
    DabEpwm4bRegs.AQCTLA.all = DabEpwm4Regs.AQCTLA.all;
    DabEpwm4bRegs.AQCTLB.all = DabEpwm4Regs.AQCTLB.all;
}

