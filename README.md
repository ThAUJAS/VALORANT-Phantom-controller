# valorant_irl
 Creating a valorant phantom controller. These are the code used to get the values of the button sent by the arduino by serial comm.
The orientation of the phantom is detected with an aruco marker.
Then, a fake controller is simulated with Python and REWASD is used to transform the controller input back into Keyboard and mouse input. (and no, I can't simulate directely mouse and keyboard, Vanguard prevents it).
