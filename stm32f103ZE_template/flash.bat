@echo off
@echo ================================================
@echo ^|^|         Shortcut of Hex Flashing           ^|^|
@echo ^|^|        By Kenneth Au (28/12/2014)          ^|^|
@echo ================================================
@echo.
@echo Checking *.hex file...
@echo.

@if not exist "output/*.hex" (
	@echo ERROR: The *.hex file does not exist. Please check the followings:
	@echo - The project has been compiled successfully
	@echo - In Keil, under the 'Options for Targets...' ^=^> 'Output', 'Create HEX File' has been checked
) else if [%1] == [] (
	@echo ERROR: Missing argument 1 (the *.hex file location^). In Keil, use "./output/@H.hex".
) else if [%2] == [] (
	@echo Missing argument 2 (COM Port number^). Please input the argument (e.g., COM30^). 
) else if [%3] == [] (
	@echo Missing argument 3 (baudrate^). Please input the argument (e.g., 115200^).
)  else (
	@echo Checking the Flash Loader executable...
	@echo.
	@if exist "C:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" (
		@echo Start flashing...
		@echo.
		start "" /b /wait "C:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" -c --pn %2 --br %3 --to 1000 -i STM32_High-density_512K -e --all -p --dwp --drp -d --a 8000000 --fn %1 --v
		@exit
	) else if exist "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" (
		@echo Start flashing...
		@echo.
		start "" /b /wait "C:\Program Files (x86)\STMicroelectronics\Software\Flash Loader Demonstrator\STMFlashLoader.exe" -c --pn %2 --br %3 --to 1000 -i STM32_High-density_512K -e --all -p --dwp --drp -d --a 8000000 --fn %1 --v
		@exit
	) else (
		@echo ERROR: The Flash Loader executable does not exist. Please check the followings:
		@echo - You have already installed Flash Loader Demonstrator 2.1.0
		@echo - You have installed Flash Loader Demonstrator under one of the following paths:
		@echo     - C^:\Program Files\STMicroelectronics\Software\Flash Loader Demonstrator\
		@echo     - C^:\Program Files (x86^)\STMicroelectronics\Software\Flash Loader Demonstrator\
		@echo - There exists an executable file called "STMFlashLoader.exe" under the path
	)
)

timeout /t 10