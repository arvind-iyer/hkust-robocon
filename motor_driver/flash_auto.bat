@echo off
@echo ================================================
@echo ^|^|         Shortcut of Hex Flashing           ^|^|
@echo ^|^|        By William LEE (04/01/2015)         ^|^|
@echo ================================================
@echo.
@echo Checking *.hex file...
@echo.
@echo Checking available COM port
@echo.
@echo off
@set /a num=-1
:CHECK
@set /a num=%num%+1
@set port=COM%num%
MODE %port% | find "RTS" > nul
@if %errorlevel% EQU 0 (
	@goto FOUND
) else if %num% NEQ 100 (
	@goto CHECK
) else (
	@echo NO available COM port, please check the following: 
	@echo - You have pluged in UART to USB port.
	@echo - Your FT232 driver is well installed.
	@echo - You allowed auto mount to guest OS if you are using virual machine.
	@goto END
)

:FOUND
if [%1] == [] (
	@echo ERROR: Missing argument 1 (the *.hex file location^). In Keil, use "./output/@H.hex".
) else if [%2] == [] (
	@echo Missing argument 2 (baudrate^). Please input the argument (e.g., 115200^).
)  else (
	@echo Checking the Flash Loader executable...
	@echo.
	@if exist "C:\Program Files\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" (
		@echo Start flashing with Flash loader 2.7...
		@echo.
		start "" /b /wait "C:\Program Files\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" -c --pn %num% --br %2 --to 1000 -i  STM32F1_Med-density_128K -e --all -p --dwp --drp -d --a 8000000 --fn %1
		@exit
	) else if exist "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" (
		@echo Start flashing with Flash loader 2.7...
		@echo.
		start "" /b /wait "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demo\STMFlashLoader.exe" -c --pn %num% --br %2 --to 1000 -i  STM32F1_Med-density_128K -e --all -p --dwp --drp -d --a 8000000 --fn %1
		@exit
	) else if exist "C:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" (
		@echo Start flashing with Flash loader 2.1...
		@echo.
		start "" /b /wait "C:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" -c --pn %num% --br %2 --to 1000 -i STM32_High-density_512K -e --all -p --dwp --drp -d --a 8000000 --fn %1
		@exit
	) else if exist "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" (
		@echo Start flashing with Flash loader 2.1...
		@echo.
		start "" /b /wait "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" -c --pn %num% --br %2 --to 1000 -i STM32_High-density_512K -e --all -p --dwp --drp -d --a 8000000 --fn %1
		@exit
	
	) else (
		@echo ERROR: The Flash Loader executable does not exist. Please check the followings:
		@echo - You have already installed either Flash Loader Demonstrator 2.7.0 or 2.1.0
		@echo - You have installed Flash Loader Demonstrator under any one of the following paths:
		@echo - version 2.7:
		@echo     - C^:\Program Files\STMicroelectronics\Software\Flash Loader Demo\
		@echo     - C^:\Program Files (x86^)\STMicroelectronics\Software\Flash Loader Demo\
		@echo - version 2.1:
		@echo     - C^:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\
		@echo     - C^:\Program Files (x86^)\STMicroelectronics\Software\Flash Loader Demonstrator\
		@echo - There exists an executable file called "STMFlashLoader.exe" under the path
	)
)
:END
PAUSE