REM Batch calling assembler
:cmp
avrasm2.exe -fI kumand.asm -e kumand.efile -m kumand.mfile -l kumand.lfile
PAUSE
GOTO cmp