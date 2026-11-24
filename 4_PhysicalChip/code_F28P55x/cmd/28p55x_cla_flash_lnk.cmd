MEMORY
{
   BEGIN            : origin = 0x080000, length = 0x000002  // Update the codestart location as needed

   BOOT_RSVD        : origin = 0x000002, length = 0x000126     /* Part of M0, BOOT rom will use this for stack */
   
   RAMM0_Flag       : origin = 0x00000128, length = 0x00000008
   RAMM01            : origin = 0x00000130, length = 0x000006C8
   
   //RAMM0            : origin = 0x000128, length = 0x0002D8
   //RAMM1            : origin = 0x000400, length = 0x000400

//   RAMLS0           : origin = 0x00008000, length = 0x00000800
//   RAMLS1           : origin = 0x00008800, length = 0x00000800
   RAMLS01           : origin = 0x00008000, length = 0x00001000

   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
//   RAMLS4           : origin = 0x00A000, length = 0x000800
//   RAMLS5           : origin = 0x00A800, length = 0x000800
//   RAMLS6           : origin = 0x00B000, length = 0x000800
//   RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMLS4567           : origin = 0x00A000, length = 0x002000   
//   RAMLS8           : origin = 0x014000, length = 0x002000  // When configured as CLA program use the address 0x4000
//   RAMLS9           : origin = 0x016000, length = 0x002000  // When configured as CLA program use the address 0x6000
//   RAMLS8_CLA       : origin = 0x004000, length = 0x002000  // Use only if configured as CLA program memory
//   RAMLS9_CLA       : origin = 0x006000, length = 0x002000  // Use only if configured as CLA program memory
   RAMLS89_CLA      : origin = 0x004000, length = 0x004000  // Combined RAMLS8 + RAMLS9 as CLA program memory (16KW, aliased to 0x014000)

   RAMGS0           : origin = 0x00C000, length = 0x002000
   RAMGS1           : origin = 0x00E000, length = 0x002000
   RAMGS2           : origin = 0x010000, length = 0x002000

   RAMGS3           : origin = 0x012000, length = 0x002000

   /* Flash Banks (128 sectors each) */
   FLASH_BOOTINFO    : origin = 0x080002, length = 0x00001E
   FLASH_BOOT  	   : origin = 0x080020, length = 0x003FE0
   
   FLASHSTART        : origin = 0x084000, length = 0x000008
   FLASH_APPINFO     : origin = 0x084008, length = 0x000020
   FLASHAPP		     : origin = 0x084028, length = 0x01BFD0
   FLASHEND		     : origin = 0x09FFF8, length = 0x000008
   
   //FLASH_BANK0     : origin = 0x080002, length = 0x1FFFE
   FLASH_BANK1     : origin = 0x0A0000, length = 0x20000
   FLASH_BANK2     : origin = 0x0C0000, length = 0x20000
   FLASH_BANK3     : origin = 0x0E0000, length = 0x20000
   FLASH_BANK4     : origin = 0x100000, length = 0x07FFF


   CLATOCPURAM      : origin = 0x001480,   length = 0x000080
   CPUTOCLARAM      : origin = 0x001500,   length = 0x000080
   CLATODMARAM      : origin = 0x001680,   length = 0x000080
   DMATOCLARAM      : origin = 0x001700,   length = 0x000080

   RESET            : origin = 0x3FFFC0, length = 0x000002
}


SECTIONS
{
/* ============================================================================
 * [恢复 Boot 布局步骤]（调试期为"无 Boot 直跑 APP"布局，恢复时按此操作）
 * 1) 本文件换回：注释掉下行 codestart : > BEGIN，恢复 codestart : > FLASHSTART(0x084000)
 * 2) 重新编译 APP -> F28P55x_APP.out（codestart@0x084000 / stAppInfo@0x084008 / .text@0x084400）
 * 3) 0x080000~0x084000 必须重烧 Boot，两种方式任选：
 *    a. 烧合并固件 DOhn_Boot_App_xx.bin（0x080000 整体烧，含 Boot+APP）
 *    b. 或先烧 Boot 工程（入口 0x080000），再烧 APP .out
 *    若调试期曾用"无 Boot"out 全片擦除过 Flash，Boot 区已为 0xFFFF，必须重烧，否则上电跑飞
 * 4) 烧 APP .out 时选"仅擦必要扇区"，禁止全片擦除（会抹掉 Boot）
 * 5) 恢复后行为：上电 ROM->0x080000 Boot->校验 stAppInfo("DOHN")->跳 0x084000->APP；
 *    ARM 0xD2 升级链路恢复有效（LB 0x080000 进 Boot）；0x080012 读到真实 Boot 版本；
 *    合并 bin 工具链恢复适用（APP 更新后需重新生成合并固件）
 * ============================================================================ */
   codestart        : > BEGIN
//   codestart        : > FLASHSTART
   
   Flash_BootInfo      : > FLASH_BOOTINFO, ALIGN(8)

   Flash_AppInfo       : > FLASH_APPINFO, ALIGN(8)
   
   .text            : > FLASHAPP, ALIGN(8)
   .cinit           : > FLASHAPP, ALIGN(8)
   .switch          : > FLASHAPP, ALIGN(8)
   .reset           : > RESET, TYPE = DSECT /* not used, */

   .stack           : > RAMM01
#if defined(__TI_EABI__)
   .bss             : > RAMGS0 | RAMGS1
   .bss:output      : > RAMGS0 | RAMGS1
   .init_array      : > FLASHAPP, ALIGN(8)
   .const           : > FLASHAPP, ALIGN(8)
   .data            : > RAMGS0
   .sysmem          : > RAMGS0
#else
   .pinit           : > FLASHAPP, ALIGN(8)
   .ebss            : > RAMGS0
   .econst          : > FLASHAPP, ALIGN(8)
   .esysmem         : > RAMGS0
#endif

//   ramgs0 : > RAMGS0
//   ramgs1 : > RAMGS1
//   ramgs2 : > RAMGS2

   #if defined(__TI_EABI__)
       .TI.ramfunc : {} LOAD = FLASHAPP,
			RUN = RAMLS4567,
			LOAD_START(RamfuncsLoadStart),
			LOAD_SIZE(RamfuncsLoadSize),
			LOAD_END(RamfuncsLoadEnd),
			RUN_START(RamfuncsRunStart),
			RUN_SIZE(RamfuncsRunSize),
			RUN_END(RamfuncsRunEnd),
			ALIGN(8)
   #else
       .TI.ramfunc : {} LOAD = FLASHAPP,
			RUN = RAMLS4567,
			LOAD_START(_RamfuncsLoadStart),
			LOAD_SIZE(_RamfuncsLoadSize),
			LOAD_END(_RamfuncsLoadEnd),
			RUN_START(_RamfuncsRunStart),
			RUN_SIZE(_RamfuncsRunSize),
			RUN_END(_RamfuncsRunEnd),
			ALIGN(8)
   #endif

/* CLA specific sections */

#if defined(__TI_EABI__)
    /* CLA specific sections */
    Cla1Prog        : LOAD = FLASHAPP,
		      RUN = RAMLS89_CLA,
		      LOAD_START(Cla1ProgLoadStart),
		      RUN_START(Cla1ProgRunStart),
		      LOAD_SIZE(Cla1ProgLoadSize),
		      ALIGN(4)
#else
    /* CLA specific sections */
    Cla1Prog        : LOAD = FLASHAPP,
		      RUN = RAMLS89_CLA,
		      LOAD_START(_Cla1ProgLoadStart),
		      RUN_START(_Cla1ProgRunStart),
		      LOAD_SIZE(_Cla1ProgLoadSize),
		      ALIGN(4)
#endif

   Cla1ToCpuMsgRAM  : > CLATOCPURAM
   CpuToCla1MsgRAM  : > CPUTOCLARAM

   .scratchpad      : > RAMLS2
   .bss_cla         : > RAMLS2

   Cla1DataRam      : > RAMLS3
   cla_shared       : > RAMLS3
   CLADataLS1       : > RAMLS2

#if defined(__TI_EABI__)
   .const_cla      : LOAD = FLASHAPP,
		      RUN = RAMLS2,
		      RUN_START(Cla1ConstRunStart),
		      LOAD_START(Cla1ConstLoadStart),
		      LOAD_SIZE(Cla1ConstLoadSize),
		      ALIGN(4)
#else
   .const_cla      : LOAD = FLASHAPP,
		      RUN = RAMLS2,
		      RUN_START(_Cla1ConstRunStart),
		      LOAD_START(_Cla1ConstLoadStart),
		      LOAD_SIZE(_Cla1ConstLoadSize),
		      ALIGN(4)
#endif

  {FromAppRequestFlag	= 0x00000128;}
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
