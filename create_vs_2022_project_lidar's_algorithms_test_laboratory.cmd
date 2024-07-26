@chcp 65001
@if not exist "c:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" goto missingVisualStudio2022 
@if not exist "%QTDIR%\bin\qmake.exe" goto missingQmake 
call "c:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
@ %QTDIR%\bin\qmake.exe -tp vc "lidar's_algorithms_test_laboratory.pro" -o "lidar's_algorithms_test_laboratory.vcxproj" -Wall -r

@pause
goto :exit

:missingVisualStudio2022
@echo Microsoft Visual Studio 2022 not detected
@pause
@exit
goto :eof

:missingQmake
@echo Qt qmake not detected
@pause
@exit
goto :eof

:exit

endlocal & exit /b %rc%