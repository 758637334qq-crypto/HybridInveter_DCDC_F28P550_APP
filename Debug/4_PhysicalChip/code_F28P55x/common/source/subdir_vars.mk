################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_codestartbranch.asm \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_dbgier.asm \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_usdelay.asm 

C_SRCS += \
../4_PhysicalChip/code_F28P55x/common/source/device.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_adc.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_cputimers.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_defaultisr.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_dma.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_epwm.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_gpio.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_piectrl.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_pievect.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_sysctrl.c \
../4_PhysicalChip/code_F28P55x/common/source/f28p55x_tempsensorconv.c 

C_DEPS += \
./4_PhysicalChip/code_F28P55x/common/source/device.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_adc.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_cputimers.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_defaultisr.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_dma.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_epwm.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_gpio.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_piectrl.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_pievect.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_sysctrl.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_tempsensorconv.d 

OBJS += \
./4_PhysicalChip/code_F28P55x/common/source/device.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_adc.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_codestartbranch.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_cputimers.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_dbgier.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_defaultisr.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_dma.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_epwm.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_gpio.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_piectrl.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_pievect.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_sysctrl.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_tempsensorconv.obj \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_usdelay.obj 

ASM_DEPS += \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_codestartbranch.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_dbgier.d \
./4_PhysicalChip/code_F28P55x/common/source/f28p55x_usdelay.d 

OBJS__QUOTED += \
"4_PhysicalChip\code_F28P55x\common\source\device.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_adc.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_codestartbranch.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_cputimers.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_dbgier.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_defaultisr.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_dma.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_epwm.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_gpio.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_piectrl.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_pievect.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_sysctrl.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_tempsensorconv.obj" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_usdelay.obj" 

C_DEPS__QUOTED += \
"4_PhysicalChip\code_F28P55x\common\source\device.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_adc.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_cputimers.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_defaultisr.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_dma.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_epwm.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_gpio.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_piectrl.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_pievect.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_sysctrl.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_tempsensorconv.d" 

ASM_DEPS__QUOTED += \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_codestartbranch.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_dbgier.d" \
"4_PhysicalChip\code_F28P55x\common\source\f28p55x_usdelay.d" 

C_SRCS__QUOTED += \
"../4_PhysicalChip/code_F28P55x/common/source/device.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_adc.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_cputimers.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_defaultisr.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_dma.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_epwm.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_gpio.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_piectrl.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_pievect.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_sysctrl.c" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_tempsensorconv.c" 

ASM_SRCS__QUOTED += \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_codestartbranch.asm" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_dbgier.asm" \
"../4_PhysicalChip/code_F28P55x/common/source/f28p55x_usdelay.asm" 


