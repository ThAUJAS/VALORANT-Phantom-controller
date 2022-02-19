from dis import dis
import numpy as np
import time, math, sys
from threading import Thread
import vgamepad as vg
import serial

#create the virtual gamepad
gamepad = vg.VX360Gamepad()

data = ['']*2
arduinomsg = [0]*17
arduinosave = [0]*17
#state the port and baudrate of the arudino
try:
  arduino = serial.Serial(port='COM4', baudrate=38400, timeout=.1)   
except:
  print("Error connection")
  sys.exit()
displacement = False

def direction(val,oldval):
  if val>oldval:
    return int(abs(val-oldval)*32768/1500)
  elif val<oldval:
    return int(-abs(oldval-val)*32768/1500)
  else:
    return 0

def arduinoMove():
  global arduinomsg
  data = ['']*2
  while True:
    data = arduino.readline()
    try:                                                                                             
      dataDecoded = data.decode('utf-8').split()
      for i in range(17): 
        arduinomsg[i] = int(dataDecoded[i])
    except:
      pass
    
def joystickmove():
  global arduinomsg
  arduinosave = [0]*17
  xOffset = 0
  yOffset = 0
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
    print(arduinomsg)
    for i in range(14):
      if (arduinomsg[i] == 0 and arduinosave[i] != 0):  
        gamepad.press_button(button = buttonList[i])                          
      if (arduinomsg[i] == 1 and arduinosave[i] != 1):
        gamepad.release_button(button = buttonList[i]) 
    if arduinomsg[7]==1:
      gamepad.right_joystick(x_value = direction(-arduinomsg[14],-arduinosave[14]), y_value = direction(arduinomsg[15],arduinosave[15]))
    else:
      gamepad.right_joystick(x_value = int(-arduinomsg[14]*32768/9000), y_value = int(arduinomsg[15]*32768/9000))
    #gamepad.left_joystick(x_value = 0, y_value = int(math.degrees(arduinomsg[16])*32768/18000))
    arduinosave = arduinomsg.copy()
    time.sleep(1/30)
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
    thread1 = thread_with_trace(target = arduinoMove) # "       " read the values, process it and send it with Socket
    thread1.start()    
    thread = thread_with_trace(target = joystickmove) # "       " read the values, process it and send it with Socket
    thread.start()                                                                               
