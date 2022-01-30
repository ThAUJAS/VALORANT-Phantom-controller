import vgamepad as vg
import time
import serial

gamepad = vg.VX360Gamepad()
data = 0
save = 1

xright = 0.
yright = 0.
xleft = 0.
yleft = 0.

while True:
    yleft = yleft + 1
    if (yleft % 2) == 0:  
        gamepad.left_joystick_float(x_value_float=0, y_value_float=1.0)
        gamepad.update()
        time.sleep(0.5)
    else:
        gamepad.left_joystick_float(x_value_float=0, y_value_float=-1.0)
        gamepad.update()
        time.sleep(0.5)  

    # press buttons and things
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.left_trigger_float(value_float=0.5)
    gamepad.right_trigger_float(value_float=0.5)
    gamepad.left_joystick_float(x_value_float=0, y_value_float=0.2)
    gamepad.right_joystick_float(x_value_float=-1.0, y_value_float=1.0)

    gamepad.update()

    time.sleep(1.0)

    # release buttons and things
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.right_trigger_float(value_float=0.0)
    gamepad.right_joystick_float(x_value_float=0.0, y_value_float=0.0)

    gamepad.update()

    time.sleep(1.0)

    # reset gamepad to default state
    gamepad.reset()

    gamepad.update()

    time.sleep(1.0)                