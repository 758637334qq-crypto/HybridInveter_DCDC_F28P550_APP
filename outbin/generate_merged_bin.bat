@echo off
:: ==============================================================================
:: This script merges Boot.hex and APP.hex into a single .bin and .hex file.
:: It uses hex_converter.exe to convert word-addressed HEX to byte-addressed HEX,
:: and srec_cat.exe to inject app metadata and file CRCs.
:: ==============================================================================

:: 1. Get current script directory path
set "SCRIPT_DIR=%~dp0"

:: [Merge switch] If outbin\MERGE_OFF.txt exists, skip the merge (post-build exits OK).
:: To re-enable merging, just delete outbin\MERGE_OFF.txt.
:: NOTE: in the "no-Boot direct-run" debug linker layout, the merged bin is invalid
:: (APP codestart @0x080000 is dropped; Boot jumps to empty 0x084000). Keep merge
:: OFF until the linker cmd is restored to the Boot layout (codestart > FLASHSTART).
if exist "%SCRIPT_DIR%MERGE_OFF.txt" (
    echo [SKIP] MERGE_OFF.txt found - firmware merge skipped. Delete outbin\MERGE_OFF.txt to re-enable.
    goto :EOF
)

:: 2. Set path to hex_converter.exe utility
set "CONV_TOOL=%SCRIPT_DIR%hex_converter.exe"

:: Set path to srec_cat.exe utility
set "SREC_TOOL=%SCRIPT_DIR%SREC\srec_cat.exe"

:: BOOT HEX is supplied manually in this outbin directory.
set "INPUT_BOOT=%SCRIPT_DIR%F28P55x_bootloader.hex"
set "INPUT_APP=%SCRIPT_DIR%..\Debug\F28P55x_APP.hex"

:: Get today's date formatted as MM_DD (e.g. 07_09)
set "DATE_VAL=%date:~5,2%_%date:~8,2%"

:: Set final output file paths
set "OUTPUT_BIN=%SCRIPT_DIR%DOhn_Boot_App_%DATE_VAL%.bin"
set "OUTPUT_HEX=%SCRIPT_DIR%DOhn_Boot_App_%DATE_VAL%.hex"

:: Set paths for temporary HEX files and binary chunks
set "BOOT_BYTE_HEX=%SCRIPT_DIR%Boot_byte.hex"
set "APP_BYTE_HEX=%SCRIPT_DIR%App_byte.hex"
set "BOOT_RAW_BIN=%SCRIPT_DIR%TempBoot.bin"
set "BOOT_LEN_BIN=%SCRIPT_DIR%TempBootLength.bin"
set "BOOT_CRC_BIN=%SCRIPT_DIR%TempBootCRC.bin"
set "BOOT_SLOT_BIN=%SCRIPT_DIR%Boot.bin"
set "APP_RAW_BIN=%SCRIPT_DIR%TempApp.bin"
set "APP_LEN_BIN=%SCRIPT_DIR%TempAppLength.bin"
set "APP_CRC_BIN=%SCRIPT_DIR%TempAppCRC.bin"
set "MERGED_TEMP_BIN=%SCRIPT_DIR%TempMerged.bin"
set "FINAL_BYTE_HEX=%SCRIPT_DIR%TempMergedByte.hex"
set "HEX_CONVERTER_SCRIPT=%SCRIPT_DIR%byte_hex_to_word_hex.ps1"

:: BOOT HEX must be copied into this directory before running this script.
if not exist "%INPUT_BOOT%" (
    echo [ERROR] BOOT HEX not found in outbin: %INPUT_BOOT%
    goto :EXIT_ERR
)

echo ====================================================
echo 1. Converting word-addressed HEX to byte-addressed HEX...
echo ====================================================
"%CONV_TOOL%" "%INPUT_BOOT%" "%BOOT_BYTE_HEX%"
if not exist "%BOOT_BYTE_HEX%" (
    echo [ERROR] Failed to convert Boot.hex to byte addressing!
    goto :EXIT_ERR
)
"%CONV_TOOL%" "%INPUT_APP%" "%APP_BYTE_HEX%"
if not exist "%APP_BYTE_HEX%" (
    echo [ERROR] Failed to convert APP.hex to byte addressing!
    goto :EXIT_ERR
)

echo ====================================================
echo 2. Extracting raw binaries...
echo ====================================================
:: Generate a variable-length Boot image. The short fill establishes offset 0
:: so the output size is the actual Boot image length rather than 32 KiB.
"%SREC_TOOL%" "%BOOT_BYTE_HEX%" -intel -fill 0xFF 0x100000 0x100050 -crop 0x100000 0x108000 -offset -0x100000 -byte-swap -o "%BOOT_RAW_BIN%" -binary
if not exist "%BOOT_RAW_BIN%" (
    echo [ERROR] Failed to generate TempBoot.bin!
    goto :EXIT_ERR
)

:: Convert APP HEX to TempApp.bin (from address 0x108000, fill header gap)
"%SREC_TOOL%" "%APP_BYTE_HEX%" -intel -fill 0xFF 0x108000 0x108050 -offset -0x108000 -byte-swap -o "%APP_RAW_BIN%" -binary
if not exist "%APP_RAW_BIN%" (
    echo [ERROR] Failed to generate TempApp.bin from hex!
    goto :EXIT_ERR
)

echo ====================================================
echo 2.5. Patching Boot length and appending Boot CRC32...
echo ====================================================
for %%A in ("%BOOT_RAW_BIN%") do set BootLength=%%~zA
set /a "BootCrcEnd = BootLength + 4"
set /a "BootLengthRemainder = BootLength %% 2"
echo BOOT length is: %BootLength% bytes

if not %BootLengthRemainder%==0 (
    echo [ERROR] Boot image length must be word aligned.
    goto :EXIT_ERR
)
if %BootCrcEnd% GTR 32768 (
    echo [ERROR] Boot image plus CRC exceeds the 32 KiB Boot slot.
    goto :EXIT_ERR
)

:: Flash_BootInfo is aligned to raw BIN offset 0x10. bootLength follows
:: codeType and keyword, so its 4-byte little-endian field is at offset 0x18.
"%SREC_TOOL%" "%BOOT_RAW_BIN%" -binary -crop 0 %BootLength% -exclude 0x18 0x1C -generate 0x18 0x1C -constant-l-e %BootLength% 4 -o "%BOOT_LEN_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to patch Boot length.
    goto :EXIT_ERR
)

:: Append the Boot CRC directly after the actual Boot image.
"%SREC_TOOL%" "%BOOT_LEN_BIN%" -binary -crop 0 %BootLength% -crc32-l-e %BootLength% -o "%BOOT_CRC_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to append Boot CRC.
    goto :EXIT_ERR
)

:: Preserve the fixed 32 KiB Boot slot and therefore the APP offset at 0x8000.
"%SREC_TOOL%" "%BOOT_CRC_BIN%" -binary -fill 0xFF %BootCrcEnd% 0x8000 -crop 0 0x8000 -o "%BOOT_SLOT_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to pad Boot image to 32 KiB.
    goto :EXIT_ERR
)

echo ====================================================
echo 3. Getting APP length and patching length placeholder...
echo ====================================================
:: APP metadata starts at APP BIN offset 0x10; the length field is at offset 0x18.
for %%A in ("%APP_RAW_BIN%") do set AppLength=%%~zA
echo APP length is: %AppLength% bytes (0x%AppLength%)

:: Patch 4-byte little-endian length value at metadata offset 0x18
"%SREC_TOOL%" "%APP_RAW_BIN%" -binary -exclude 0x18 0x1C -generate 0x18 0x1C -constant-l-e %AppLength% 4 -o "%APP_LEN_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to patch APP length.
    goto :EXIT_ERR
)

echo ====================================================
echo 4. Appending APP CRC32 (IEEE 802.3)...
echo ====================================================
:: Append 4-byte CRC32 at the end of APP (offset %AppLength%)
"%SREC_TOOL%" "%APP_LEN_BIN%" -binary -crop 0 %AppLength% -crc32-l-e %AppLength% -o "%APP_CRC_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to append APP CRC32.
    goto :EXIT_ERR
)

echo ====================================================
echo 5. Merging Bootloader and APP binary...
echo ====================================================
:: Merge the padded Boot slot (Boot + Boot CRC + FF) and APP (from 0x8000).
"%SREC_TOOL%" "%BOOT_SLOT_BIN%" -binary -crop 0x0000 0x8000 "%APP_CRC_BIN%" -binary -offset 0x8000 -o "%MERGED_TEMP_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to merge BOOT and APP binaries.
    goto :EXIT_ERR
)

echo ====================================================
echo 6. Appending total file CRC32 (IEEE 802.3)...
echo ====================================================
:: Total length to checksum: Boot (32768) + AppLength + APP CRC (4)
set /a "TotalLength = 32768 + %AppLength% + 4"
"%SREC_TOOL%" "%MERGED_TEMP_BIN%" -binary -crop 0 %TotalLength% -crc32-l-e %TotalLength% -o "%OUTPUT_BIN%" -binary
if errorlevel 1 (
    echo [ERROR] Failed to write merged BIN: %OUTPUT_BIN%
    goto :EXIT_ERR
)

echo ====================================================
echo 6.5. Generating merged HEX file (word-addressed)...
echo ====================================================
:: Final BIN -> C28x word-addressed HEX.
"%SREC_TOOL%" "%OUTPUT_BIN%" -binary -offset 0x100000 -byte-swap -o "%FINAL_BYTE_HEX%" -intel
if errorlevel 1 (
    echo [ERROR] Failed to generate byte-addressed HEX.
    goto :EXIT_ERR
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%HEX_CONVERTER_SCRIPT%" -InputFile "%FINAL_BYTE_HEX%" -Output "%OUTPUT_HEX%"
if errorlevel 1 (
    echo [ERROR] Failed to generate word-addressed HEX.
    goto :EXIT_ERR
)

goto :SUCCESS

:SUCCESS
echo ====================================================
echo [SUCCESS] Merged firmware generated at:
echo BIN: %OUTPUT_BIN%
echo HEX: %OUTPUT_HEX%
echo ====================================================

echo ====================================================
echo 7. Cleaning up temporary files...
echo ====================================================
:CLEANUP
if exist "%BOOT_BYTE_HEX%" del "%BOOT_BYTE_HEX%"
if exist "%APP_BYTE_HEX%" del "%APP_BYTE_HEX%"
if exist "%BOOT_RAW_BIN%" del "%BOOT_RAW_BIN%"
if exist "%BOOT_LEN_BIN%" del "%BOOT_LEN_BIN%"
if exist "%BOOT_CRC_BIN%" del "%BOOT_CRC_BIN%"
if exist "%BOOT_SLOT_BIN%" del "%BOOT_SLOT_BIN%"
if exist "%APP_RAW_BIN%" del "%APP_RAW_BIN%"
if exist "%APP_LEN_BIN%" del "%APP_LEN_BIN%"
if exist "%APP_CRC_BIN%" del "%APP_CRC_BIN%"
if exist "%MERGED_TEMP_BIN%" del "%MERGED_TEMP_BIN%"
if exist "%FINAL_BYTE_HEX%" del "%FINAL_BYTE_HEX%"
goto :EOF

:EXIT_ERR
echo [ERROR] Firmware merging aborted due to errors.
goto :CLEANUP
