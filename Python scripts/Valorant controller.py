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

def maxi(val):
  if abs(val)<2:
    return 0
  if abs(val)>16:
    return -np.sign(val)*20000
  else:
    return int(-np.sign(val)*math.sqrt(abs(val))*32768/4)

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
  global arduinomsg
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
    vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK, # walk / Lctrl
    vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER, # aim / right click
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
    except:
      pass

def arucoRead():
  global arucomsg,angle
  filteredData = [0.]*3
  count = 0
  #--- Define Tag
  id_to_find  = 5
  marker_size  = 5 #- [cm]

  #--- Get the camera calibration path
  calib_path  = "C:/Projets/valorant_irl"
  camera_matrix   = np.loadtxt(calib_path+'/cameraMatrix.txt', delimiter=',')
  camera_distortion   = np.loadtxt(calib_path+'/cameraDistortion.txt', delimiter=',')

  #--- 180 deg rotation matrix around the x axis
  R_flip  = np.zeros((3,3), dtype=np.float32)
  R_flip[0,0] = 1.0
  R_flip[1,1] =-1.0
  R_flip[2,2] =-1.0

  #--- Define the aruco dictionary
  aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  while True:
      count = count +1
      ret, frame = cap.read()
      gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
      corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, camera_matrix, camera_distortion)
      
      if ids is not None and ids[0] == id_to_find:
          ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
          rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
          R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
          R_tc    = R_ct.T
          #-- Draw the detected marker and put a reference frame over it
          aruco.drawDetectedMarkers(frame, corners)
          aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
          roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc) 
          angle = [math.degrees(yaw_camera), math.degrees(roll_camera),math.degrees(pitch_camera)]
          pos_camera = -R_tc*np.matrix(tvec).T   
          filteredData = expFilter(tvec,filteredData)
          arucomsg = filteredData
          #.append(ardata)
          count = 0
          
      elif (count>10):
          arucomsg = [0.,0.,0.]

      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
          cap.release()
          cv2.destroyAllWindows()
          break

def joystickMove():
  global arduinomsg,arucomsg,angle
  while True:
    gamepad.left_joystick(x_value = 0, y_value = int(angle[0]*32768/180))
    gamepad.right_joystick(x_value = int(-arucomsg[0]*32768/15), y_value = int(-arucomsg[1]*32768/15))
    print(arucomsg,arduinomsg)
    gamepad.update()
                                                                                                        
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
    thread1 = thread_with_trace(target = joystickMove) # virtual controller
    thread1.start()         
    thread = thread_with_trace(target = arduinoMove) # "       " read the values, process it and send it with Socket
    thread.start()                                                                                 
