@echo off
REM Prefer the local Windows AVL binary when present.
setlocal
set "SCRIPT_DIR=%~dp0"
if exist "%SCRIPT_DIR%bin\avl.exe" (
  "%SCRIPT_DIR%bin\avl.exe" %*
  exit /b %ERRORLEVEL%
)
echo AVL not found at "%SCRIPT_DIR%bin\avl.exe" 1>&2
echo Download https://web.mit.edu/drela/Public/web/avl/avl352.exe into tools\bin\avl.exe 1>&2
exit /b 1
