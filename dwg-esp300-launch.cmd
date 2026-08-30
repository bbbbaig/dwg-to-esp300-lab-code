@echo off
netstat -ano | findstr :8768 >nul
if errorlevel 1 (
    start "DWG to ESP300 Server" /min cmd /c ""C:\Users\bjh14\AppData\Local\Programs\Python\Python312\python.exe" "%~dp0visualizer_server.py" --host 127.0.0.1 --port 8768"
    timeout /t 1 /nobreak >nul
)
start "" http://127.0.0.1:8768/
