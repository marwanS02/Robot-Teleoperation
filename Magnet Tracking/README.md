mag_field_tracker_v6.py:
-Reads data from the mag sensor matrix and infers the position and orientation of the magnet
-Opens a GUI to show tracking of magnet and sends the data via UDP to the main.cpp script in the project "Telecontrol v3.0 - Long Distance"
-Make sure all 3 teensies are flashed each with firmware in Mag3D_plateX, Mag3D_plateY, Mag3D_plateZ and connected via USB to the laptop
-To create new env “python -m venv venv“
-To activate the env ".\venv\Scripts\Activate.ps1"
-To install requirements "pip install -r requirements.txt"
-python 3.12.x is the one tested
-When the script starts usig "python mag_field_tracker_v6.py” it will calibrate so make sure there is no metal or magnet in the workspace when you start it
-To enable or disable GUI ENABLE_VISUALIZATION = True or False