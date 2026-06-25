@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cl /utf-8 /EHsc main.cpp Changingtek_p_rtu_Servo.cpp /Fe:example_servo.exe
if %errorlevel% neq 0 exit /b %errorlevel%
del *.obj
example_servo.exe
