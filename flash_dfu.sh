#!/bin/bash
# ============================================================
# STM32 DFU Flash Script
# Board: STM32F401xE (Device ID: 0x0433)
# ============================================================

# --- EDIT THIS to point to your .hex file ---
HEX_FILE="Design_Embedded_Subsystem.hex"

# --- CubeProgrammer CLI path ---
# Default Linux path:
PROGRAMMER="/opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI"
# Default Mac path (uncomment if on Mac):
# PROGRAMMER="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"

echo "============================================================"
echo " STM32 DFU Flasher"
echo " File: $HEX_FILE"
echo "============================================================"
echo ""

# Check file exists
if [ ! -f "$HEX_FILE" ]; then
    echo "ERROR: Could not find $HEX_FILE"
    echo "Make sure the .hex file is in the same folder as this script."
    exit 1
fi

echo "Make sure the board is in DFU mode (BOOT0 = HIGH, then reset)"
read -p "Press ENTER to continue..."

echo ""
echo "[1/2] Erasing flash..."
"$PROGRAMMER" -c port=USB1 -e all
if [ $? -ne 0 ]; then
    echo "ERROR: Erase failed. Is the board in DFU mode?"
    exit 1
fi

echo ""
echo "[2/2] Flashing $HEX_FILE..."
"$PROGRAMMER" -c port=USB1 -w "$HEX_FILE" -v -rst
if [ $? -ne 0 ]; then
    echo "ERROR: Flash failed."
    exit 1
fi

echo ""
echo "============================================================"
echo " SUCCESS! Board has been flashed and reset."
echo " Pull BOOT0 LOW before the next reset to run your app!"
echo "============================================================"
