__Only Work on Windows 64-bit__

1. install https://sourceforge.net/projects/vjoystick/

2. install assetto corsa @ steam https://store.steampowered.com/app/244210/Assetto_Corsa/

3. copy /importtest into $(SteamInstallDir)/steamapps/common/assettocorsa/apps/python/

4. run assetto corsa

5. run ac_server() on MATLAB

6. set controller on Assetto Corsa and Allow Python App (aclab)
- AxisX : steering
- AxisY : gas
- AxisZ : brake

7. start run on Assetto Corsa

8. ac_server.getMap() : convert AI path into MATLAB, ac_server.ai

9. ac_server.poll() : get drive data into MATLAB, ac_server.data

10. ac_server.push(steering, gas, brake) : push control into Assetto Corsa