Please note that you should have libfranka installed from https://github.com/frankaemika/libfranka
The tested version is 0.13.3.

Make sure it is installed in this folder "Robot Station" and you should see a folder called frank_ws which includes another folder called libfranka after installation.

This repo contains the script Telecontrol.cpp which you should move to "Robot Station/franka_ws/libfranka/examples". Then go to CMakeLists.txt in the same folder and add the name Telecontrol inside "set(EXAMPLES)". 

After that you can build by going to franka_ws/libfranka/build/examples in the terminal and then writing "make".

You can run it by writing "./Telecontrol 172.16.0.2" in terminal in the same location you built it, where 172.16.0.2 is an example of the robot ip address.
----------------------------------------------------------------------------------
sender.py:
-Sends webcam data to the user workstation
-Make sure it is a logitec 1080 p camera (only one tested)
-If using tailscale make sure you are connected to the internet and in the tailscale network of the user workstation. Do not forget to put WINDOWS_IP = user tailscale workstation ip
-If using local network tp link exo check user workstation static ip and put WINDOWS_IP = user workstation ip
- Run python3 sender.py
------------------------------------------------------------------------------------
Telecontrol.cpp:
-Controls the robot in 7 DOF by using the data sent from user workstation as well as reads the the data from the robot and sends it back to user workstation
-Make sure to change the ip address of the user workstation based on if you're using tailscale vpn or local wifi. (Just as described for camera communication) 
-It is located in /home/aries/marwan/franka_ws/libfranka/examples
-To rebuild it after changes in terminal put the command "cd ~/marwan/franka_ws/libfranka/build/examples" to go to the build folder
-Write "make"
-After that to run the app write in the command "./Telecontrol 172.16.0.2"
-To create a new cpp file create it (e.g. new_file.cpp) in /home/aries/marwan/franka_ws/libfranka/examples
 Then go to CMakeLists.txt in the same folder and add the name inside "set(EXAMPLES)"
 After that you can build it using "make" as previously explained
-Make sure robot is in execution mode, unlocked, and FCI is activated
