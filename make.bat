@echo off
set wd="/mnt/c/Users/ahigg/OneDrive/Documents/STM/HexLed"

if "%~1"=="" goto blank

wsl --cd %wd% make %1
goto done

:blank
wsl --cd %wd% "make"
goto done

:done