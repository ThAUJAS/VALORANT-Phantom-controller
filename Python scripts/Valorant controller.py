from dis import dis
import numpy as np
import cv2
import cv2.aruco as aruco
import time, math, sys
from threading import Thread
import vgamepad as vg
import serial

#create the virtual gamepad
gamepad = vg.VX360Gamepad()

arduinomsg = [0]*14
arduinosave = [0]*14
arucomsg = [0]*3
angle = [0]*3
yaw_camera = 0
displacement = False

def direction(val,oldval):
  if val >= 13:
    return int(5000)
  elif val <= -13:
    return int(-5000)
  elif val>oldval:
    return int(abs(val-oldval)*20000/13)
  elif val<oldval:
    return int(-abs(oldval-val)*20000/13)
  else:
    return 0

def expFilter(new, old):
    filtered = [0]*3
    alpha = 0.2
    for i in range(3):
        filtered[i]=round(new[i] * alpha + old[i] * (1-alpha), 2)
    return filtered

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
 
    return np.array([x, y, z])

def arduinoMove():
  global arduinomsg, displacement
  data = ['']*2
  arduinomsg = [0]*14
  arduinosave = [0]*14
  #state the port and baudrate of the arudino
  arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

  buttonList = [
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT, # left / q
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT, # right / d
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN, # backward / s
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP, # forward / z
    vg.XUSB_BUTTON.XUSB_GAMEPAD_START, # reload / T
    vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER, # shoot / left click
    vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER, # aim / right click
    vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK, # walk / Lctrl
    vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB, #  jump / space 
    vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB, # use / E
    vg.XUSB_BUTTON.XUSB_GAMEPAD_A, # skill 1 / R
    vg.XUSB_BUTTON.XUSB_GAMEPAD_B, # skill 2 / A
    vg.XUSB_BUTTON.XUSB_GAMEPAD_Y, # skill 3 / F
    vg.XUSB_BUTTON.XUSB_GAMEPAD_X] # ult / X

  while True:
    data = arduino.readline()
    try:                                                                                             
      dataDecoded = data.decode('utf-8').split()
      for i in range(14):
        arduinomsg[i] = int(dataDecoded[i])
        #print(arduinomsg)
        if (arduinomsg[i] == 0 and arduinosave[i] != 0):  
          gamepad.press_button(button = buttonList[i])                          
        if (arduinomsg[i] == 1 and arduinosave[i] != 1):
          gamepad.release_button(button = buttonList[i]) 
        gamepad.update()
        arduinosave = arduinomsg.copy()
        if(arduinomsg[7]==0):
          displacement = True
        if(arduinomsg[7]==1):
          displacement = False
    except:
      pass

def arucoRead():
  global arucomsg, yaw_camera,arduinomsg, displacement
  filteredData = [0.]*3
  #--- Define Tag
  id_to_find  = 5
  marker_size  = 5 #- [cm]

  #--- Get the camera calibration path
  calib_path  = "C:/Projets/valorant_irl/valorant_irl_code/Python scripts"
  camera_matrix   = np.loadtxt(calib_path+'/cameraMatrix.txt', delimiter=',')
  camera_distortion   = np.loadtxt(calib_path+'/cameraDistortion.txt', delimiter=',')

  #--- 180 deg rotation matrix around the x axis
  R_flip  = np.zeros((3,3), dtype=np.float32)
  R_flip[0,0] = 1.0
  R_flip[1,1] =-1.0
  R_flip[2,2] =-1.0
  save = [0]*3
  
  #--- Define the aruco dictionary
  aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  while True:
      ret, frame = cap.read()
      gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
      corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, camera_matrix, camera_distortion)
      
      if ids is not None and ids[0] == id_to_find:
          ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
          rvec, arucomsg = ret[0][0,0,:], ret[1][0,0,:]
          R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
          R_tc    = R_ct.T
          #-- Draw the detected marker and put a reference frame over it
          aruco.drawDetectedMarkers(frame, corners)
          aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, arucomsg, 10)
          r, p, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc) 
          #filteredData = expFilter(tvec,filteredData)
          gamepad.left_joystick(x_value = 0, y_value = int(math.degrees(yaw_camera)*32768/180))
          if displacement:
            gamepad.right_joystick(x_value = int(-arucomsg[0]*32768/60), y_value = int(-arucomsg[1]*32768/60))
          else:
            gamepad.right_joystick(x_value = direction(-arucomsg[0],-save[0]), y_value = direction(-arucomsg[1],-save[1]))
          time.sleep(0.05)
          print(displacement)
          save = arucomsg.copy()
          gamepad.update()
      else:
        gamepad.right_joystick(x_value = 0, y_value = 0)
        gamepad.update()

      key = cv2.waitKey(1) & 0xFF

      if key == ord('q'):
          cap.release()
          cv2.destroyAllWindows()
          break
                                                                                                     
class thread_with_trace(Thread):

  def __init__(self, *args, **keywords):
    Thread.__init__(self, *args, **keywords)
    self.killed = False
  
  def start(self):
    self.__run_backup = self.run
    self.run = self.__run      
    Thread.start(self)
  
  def __run(self):
    sys.settrace(self.globaltrace)
    self.__run_backup()
    self.run = self.__run_backup
  
  def globaltrace(self, frame, event, arg):
    if event == 'call':
      return self.localtrace
    else:
      return None
  
  def localtrace(self, frame, event, arg):
    if self.killed:
      if event == 'line':
        raise SystemExit()
    return self.localtrace
  
  def kill(self):
    self.killed = True

if __name__ == "__main__":
    thread = thread_with_trace(target = arucoRead) # "       " read the values, process it and send it with Socket
    thread.start()  
    thread1 = thread_with_trace(target = arduinoMove) # "       " read the values, process it and send it with Socket
    thread1.start()                                                                                 
