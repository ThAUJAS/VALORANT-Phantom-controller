import numpy as np
import cv2
import cv2.aruco as aruco
import time, math, sys
from threading import Thread
import vgamepad as vg
import serial


#create the virtual gamepad
gamepad = vg.VX360Gamepad()

arduinomsg = [0]*2
arucomsg = [0]*3

def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

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
  data = ['']*2
  arduinomsg = [0]*2
  arduinosave = [0]*2
  #state the port and baudrate of the arudino
  arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

  buttonList = [vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_GUIDE,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
    vg.XUSB_BUTTON.XUSB_GAMEPAD_Y]

  while True:
    data = arduino.readline()   
    try:                                                                                             
      dataDecoded = data.decode('utf-8').split()
      for i in range(2):
        arduinomsg[i] = int(dataDecoded[i])
        if (arduinomsg[i] == 0 and arduinosave[i] != 0):  
          gamepad.press_button(button = buttonList[i])                          
        if (arduinomsg[i] == 1 and arduinosave[i] != 1):
          gamepad.release_button(button = buttonList[i]) 
        gamepad.update()
        arduinosave = arduinomsg.copy()
    except:
      pass
    
    # release buttons and things
    """"gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.right_trigger_float(value_float=0.0)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.left_trigger_float(value_float=0.5)
    gamepad.right_trigger_float(value_float=1)
    gamepad.left_joystick_float(x_value_float=0, y_value_float=0.2)"""

def arucoRead():
  global arucomsg
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
          filteredData = expFilter(angle,filteredData)
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
  global arucomsg
  while True:
    gamepad.right_joystick(x_value = int(-arucomsg[2]*32768/90), y_value = int(arucomsg[1]*32768/90))
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
