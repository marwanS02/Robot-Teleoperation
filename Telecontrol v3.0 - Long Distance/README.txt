receiver.py: 
-python file that receives the camera data in real time from robot workstation
-Either through tailscale network: join via the link https://login.tailscale.com/uinv/iWv8SRywbn11SdbqfRKe811 (you need to be approved by admin Huimin for e.g.)
-Either through local network (use static ip address 192.168.4.x）
-Run sender.py from robot workstation first via command "python3 sender.py"
-Run receiver.py by installing  dependencies from requirements.txt (python 3.12.x) into a new venv
-To create new env “python -m venv venv“
-To activate the env ".\venv\Scripts\Activate.ps1"
-To install requirements "pip install -r requirements.txt"
-Use command "python receiver.py"
---------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------
main.cpp:
-Script to communicate with robot via UDP as well as with the exosuit, EMG predictor and magnet tracker scripts and plots relevant  data
-To use it first time you need to install Visual studio 2022 community from https://aka.ms/vs/17/release/vs_Community.exe
-In Visual Studio Installer, check:

✅ Desktop development with C++

	Then, on the right-hand side (Installation details), make sure these are selected (some auto-select, verify anyway):

		✅ MSVC v143 – VS 2022 C++ x64/x86 build tools

		✅ Windows 10 SDK or Windows 11 SDK

		✅ C++ CMake tools for Windows

		✅ C++ core features
- Install cmake from https://cmake.org/download/  specifically: Windows x64 Installer and add to PATH
- Write the command in terminal: cmake -S . -B build -G "Visual Studio 17 2022" -A x64 （only first time running ad setting up the project)
- To build the c++ app you put in terminal: cmake --build build --config Release (run every time you change the code)
- To run the app you put in terminal: build\Release\udp_tx_rx_sines_imgui.exe
- You can record usinng record button on the GUI which  will save a csv file with all the data with the chosen name
- To stop the app running in the background use in terminal:  Get-Process udp_tx_rx_sines_imgui -ErrorAction SilentlyContinue | Stop-Process -Force
