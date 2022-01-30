from threading import Thread
import socket
import sys
import time
import vgamepad as vg

localIP     = "192.168.43.78"
localPort   = 20001
bufferSize  = 1024
message = ""

gamepad = vg.VX360Gamepad()
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

UDPServerSocket.bind((localIP, localPort))

def readval():
    global message
    print("UDP server up and listening")
    while(True):
      bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
      message = bytesAddressPair[0].decode("utf-8")

def move():
  global message
  val = [0]*3
  while(True):
    data = message.split(",")
    if(message == ''):
      data = ['0']*3
    for i in range(len(data)):
      val[i] = float(data[i])
    # press buttons and things
    """gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.left_trigger_float(value_float=0.5)
    gamepad.right_trigger_float(value_float=0.5)
    gamepad.left_joystick_float(x_value_float=0, y_value_float=0.2)"""
    gamepad.right_joystick_float(x_value_float= float(val[2]/25), y_value_float= float(val[1]/25))

    gamepad.update()

    time.sleep(0.1)

    # release buttons and things
    """gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.right_trigger_float(value_float=0.0)"""
    #gamepad.right_joystick_float(x_value_float=0.0, y_value_float=0.0)

    # reset gamepad to default state
    gamepad.reset()

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
    thread = thread_with_trace(target = readval) # "       " read the values, process it and send it with Socket
    thread.start()
    thread1 = thread_with_trace(target = move) # virtual controller
    thread1.start()                                                                                                         