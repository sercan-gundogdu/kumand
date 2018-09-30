REM Batch calling assembler
:start
avrasm2.exe -fI kumand.asm -e kumand.efile -m kumand.mfile -l kumand.lfile
avrdude -c usbtiny -p m328p -U flash:w:kumand.hex -U eeprom:w:kumand.efile
PAUSE
GOTO start