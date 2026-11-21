################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
1_TaskApp/source/%.obj: ../1_TaskApp/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu1 --vcu_support=vcrc -O0 --fp_mode=relaxed --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT" --include_path="D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/0_MainFrame/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/1_TaskApp/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/2_InterruptApp/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/3_UcosCode/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_F28P55x/cmd" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_ConfigSysInit/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_F28P55x/common/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_F28P55x/headers/include" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_F28P55x/driverlib" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/4_PhysicalChip/code_F28P55x/driverlib/inc" --include_path="E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/lib" --advice:performance=all --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="1_TaskApp/source/$(basename $(<F)).d_raw" --obj_directory="1_TaskApp/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


