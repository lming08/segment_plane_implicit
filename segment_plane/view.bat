::for %%i in (*.pcd) do  echo %%i

SETLOCAL ENABLEDELAYEDEXPANSION
for /f %%i in ('dir *.pcd /b') do (
  set name=!name! %%i 
)
::ENDLOCAL

@echo !name!

::开始显示pcd文件
pcd_viewer_release !name!