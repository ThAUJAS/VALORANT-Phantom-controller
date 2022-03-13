# Valorant phantom controller
## Project description
 This project's purpose is to create a Valorant phantom controller. There is the code used to get the values of the button sent by an Arduino Nano by serial communication and the orientation (roll/pitch/yaw) of the phantom is detected with an IMU (MPU560).
Then, a simulated Xbox controller is created by Python and the software REWASD is used to transform the controller input back into Keyboard and mouse input. The simpler would have been to directly map the orientation to mouse movements but I can't simulate directly mouse and keyboard, because Vanguard prevents it (the anti-cheat software). Furthermore, the REWASD software isn't free, although it's only 6$ and there is a 14-day free trial.

There are in total 14 buttons, an IMU, a potentiometer, and an RGB LED (optional)wired to the Arduino nano. This is pretty much like a controller, each element has a purpose:
- 4 for moving 
- 1 to shoot
- 1 to reload
- 1 to change from aiming mode to walking mode (will go into detail about that)
- 3 custom buttons (currently mapped to: Aiming, use/walk, and jump)
- 1 button coupled with 1 potentiometer to choose the abilities (4 thresholds of pot values for each ability), a LED light-up to indicated the current chosen abilities.
- The IMU detects the YAW for the left/right movements and the PITCH for up/down. Also, the ROLL is used to change weapons.

Now, the REWASD software only allows a "joystick" type of mapping for the mouse, which means when you move the joystick, the mouse will move to a certain speed (depending on the amplitude of the movement and the sensitivity of the mouse). This results in a very bad aiming experience, but it is very useful for moving. On that note, I implemented a mouse position control depending on the orientation, which is way more intuitive but it limits your movements to the screen size. Therefore, I decided to use a combination of both behaviors to be able to have a good aim and be able to move freely. The button I mentioned "to change from aiming mode to walking mode" is the button used to switch between the two behaviors and allow full control over your movements.

## What you need
### Software-wise
Make sure to install Arduino and Python3. (but I guess that if you're ready to do this project, you already know what you are doing)
- For Arduino, just transfer the code into the board. 
- For the simulated controller can check the [VgamePad page](https://pypi.org/project/vgamepad/), but basically:
    - first, install the library by opening a command prompt and copying:
        ```
        pip3 install vgamepad
        ```
    - Then install the latest release of the driver [VigemBus](https://github.com/ViGEm/ViGEmBus/releases) (**ViGEmBusSetup_x64.msi**)
- Install REWASD and buy the license if required and import my configuration (but you can also create your own)
- Finally, launch the python code (**point to your screen and put your phantom straight for calibration and wait 2 seconds**) and make sure that an Xbox controller appears in REWASD, you can then choose the configuration. Apply and you are ready to play.

The configuration, the button mapping, and the wiring are obviously up to you, if you want to change them, you can, very easily.

To stop the code you just close the Python code, if you're using Visual Studio Code or somthing similar, you will need to close the terminal (I didn't found a way to stop the thread...).
 ### Hardware-wise
- The wiring is shown in the picture below:
  You might need to solder, a lot actually... but these are just buttons, easy peasy.
- Print all the parts in the folder STL, some parts can be glued together such as the silencer, the buttons holders, or the stock.
