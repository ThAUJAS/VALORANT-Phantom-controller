# Valorant phantom controller
## Project description
 This project's purpose is to create a valorant phantom controller. There are the code used to get the values of the button sent by an Arduino Nano by serial communication and the orientation (roll/pitch/yaw) of the phantom is detected with an IMU (MPU560).
Then, a simulated Xbox controller is created by Python and the software REWASD is used to transform the controller input back into Keyboard and mouse input. The simpler would have been to directly map the orientation to mouse movements but I can't simulate directely mouse and keyboard, because Vanguard prevents it (the anti-cheat software). Furthermore, the REWASD software isn't free, although it's olny 6$ and there is a 14 days free trial.

# All you need
Make sure to install Arduino and Python3. (but I guess that if you're ready to do this project, you already know what you are doing)
- For Arduino, just transfer the code into the board. The wiring is shown in the picture bellow:
- For the simulated controller can check the [VgamePad page](https://pypi.org/project/vgamepad/), but basically:
    - first install the library by opening a command prompt and copy:
        ```
        pip3 install vgamepad
        ```
    - Then install the latest release of the driver [VigemBus](https://github.com/ViGEm/ViGEmBus/releases) (**ViGEmBusSetup_x64.msi**)
- Install REWASD and buy the license if required and import my configuration (but you can also create your own)
- Finammy, launch the python code and make sure that a Xbox controller appears in REWASD, you can then choose the configuration. Apply and you are ready to play.

The configuration, the button mapping and the wiring are obviously up to you, if you want to change them, you can, very easily.

Enjoy!!!


