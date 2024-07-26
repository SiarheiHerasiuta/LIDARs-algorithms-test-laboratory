@chcp 65001
@if not exist "c:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" goto missingVisualStudio2022 
call "c:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
@@msbuild lidar's_algorithms_test_laboratory.vcxproj /p:configuration=release
@pause
goto :exit

:missingVisualStudio2022
@echo Microsoft Visual Studio 2022 not detected
@pause
@exit
goto :eof

:exit

  endlocal & exit /b %rc%