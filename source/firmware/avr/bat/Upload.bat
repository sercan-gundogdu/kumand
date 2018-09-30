REM Batch calling assembler
avrdude -c usbtiny -p m328p -U flash:w:kumand.hex -U eeprom:w:kumand.efile
PAUSE