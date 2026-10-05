@REM Run one stationary front-CSI yellow-divider calibration capture.
@ECHO OFF
setlocal enabledelayedexpansion

if exist ".qcar_credentials.bat" call ".qcar_credentials.bat"
if not defined QCAR_PASSWORD (
    echo Missing .qcar_credentials.bat or QCAR_PASSWORD environment variable.
    exit /b 1
)

set "configFile=config.txt"
for /f "usebackq tokens=1,* delims==" %%A in (!configFile!) do (
    set "key=%%A"
    set "value=%%B"
    set "!key!=!value!"
)

python python/capture_lane_calibration.py --qcar_ip 192.168.0.151 --remote_path "%REMOTE_PATH%" %*
exit /b %ERRORLEVEL%
