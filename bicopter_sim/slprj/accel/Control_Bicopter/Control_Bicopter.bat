@echo off
set "VSCMD_START_DIR=%CD%"
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\VCVARSALL.BAT " amd64

cd .
nmake -f Control_Bicopter.mk 
@if errorlevel 1 goto error_exit
exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1
