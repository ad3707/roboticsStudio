import serial
import lewansoul_lx16a
from lewansoul_lx16a_controller import ServoController
#from lx16a import *
import time
from math import sin, cos

def main():
    #LX16A.initialize('/dev/cu.usbserial-14340')
    SERIAL_PORT = '/dev/cu.usbserial-1420'

    controller = lewansoul_lx16a.ServoController(
        serial.Serial(SERIAL_PORT, 115200, timeout=1),
    )

    try:
        servo1 = controller.servo(1) #top right
        servo2 = controller.servo(2) #bottom left
        servo3 = controller.servo(3)
        servo4 = controller.servo(4)


    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        exit()



    t = 0
    while True:
    # Oscillate servos out of phase
    # Parametric locomotion method
        servo1.move(sin(t) * 120 + 180)
        # servo2.move(cos(t) * 120 + 180)
        # servo3.move(cos(t) * 120 - 180)
        # servo4.move(cos(t) * 120 + 180)
        # time.sleep(0.05)
        # t += 0.05

#def walk():
    #Use keyrframing
    #Physically move the robot to walk along the desired path
    #Use LX16A.getPhysicalPos() which returns the current physical position of the servo1 to record the motor positions along the way
    #Use those points to make the robot walk slowly
    #optimize it using scipy.optimize.curve to fit a function to the data

    # LX16A.initialize('/dev/cu.usbserial-1420')
    # servo1 = LX16A(1)
    # servo2 = LX16A(2)
    # servo3 = LX16A(3)
    # servo4 = LX16A(4)
    #
    # servo1_pos = servo1.getPhysicalPos()
    # # servo1_pos = LX16A.getPhysicalPos(servo1)
    # # servo2_pos = LX16A.getPhysicalPos(servo2)
    # # servo3_pos = LX16A.getPhysicalPos(servo3)
    # # servo4_pos = LX16A.getPhysicalPos(servo4)
    #
    # print("servo1 current pos = ", servo1_pos)
    # print("servo2 current pos = ", servo2_pos)
    # print("servo3 current pos = ", servo3_pos)
    # print("servo4 current pos = ", servo4_pos)

def getPositions():
    #SERIAL_PORT = '/dev/cu.usbserial-1420'
    s = serial.Serial('/dev/cu.usbserial-1420', 115200, timeout=1)
    c = ServoController(s, timeout=5)


    servo1_id = 1
    # servo2_id = 2
    # servo3_id = 3
    # servo4_id = 4

    # servo1_pos =  c.get_positions([servo1_id])
    # servo2_pos =  c.get_positions([servo2_id])
    # servo3_pos =  c.get_positions([servo3_id])
    # servo4_pos =  c.get_positions([servo4_id])

    print(c.get_positions([servo1_id]))


def homing_routine():
    SERIAL_PORT = '/dev/cu.usbserial-14340'

    #set initial motor positions
    initial_servo1_pos = 0
    initial_servo2_pos = 90
    initial_servo3_pos = 90
    initial_servo4_pos = 0

    controller = lewansoul_lx16a.ServoController(
        serial.Serial(SERIAL_PORT, 115200, timeout=1),
    )

    try:
        servo1 = controller.servo(1)
        servo2 = controller.servo(2)
        servo3 = controller.servo(3)
        servo4 = controller.servo(4)

    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        exit()

    while LX16A.getPhysicalPos(servo1) != initial_servo1_pos:
        servo1.move(initial_servo1_pos)
    while LX16A.getPhysicalPos(servo2) != initial_servo2_pos:
        servo2.move(initial_servo2_pos)
    while LX16A.getPhysicalPos(servo3) != initial_servo3_pos:
        servo3.move(initial_servo3_pos)
    while LX16A.getPhysicalPos(servo4) != initial_servo4_pos:
        servo4.move(initial_servo4_pos)


def boot_test():

    SERIAL_PORT = '/dev/cu.usbserial-14340'

    controller = lewansoul_lx16a.ServoController(
        serial.Serial(SERIAL_PORT, 115200, timeout=1),
    )

    try:
        servo1 = controller.servo(1)
        servo2 = controller.servo(2)
        servo3 = controller.servo(3)
        servo4 = controller.servo(4)

    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        exit()

    #Checks voltage
    voltage_response = self._query(servo1, SERVO_VIN_LIMIT_READ, timeout=timeout)
    voltage_response2 = self._query(servo2, SERVO_VIN_LIMIT_READ, timeout=timeout)
    voltage_response3 = self._query(servo3, SERVO_VIN_LIMIT_READ, timeout=timeout)
    voltage_response4 = self._query(servo4, SERVO_VIN_LIMIT_READ, timeout=timeout)

    #turns on LED 3 times
    for i in range(1,3):
        self._command(servo1, SERVO_LED_CTRL_WRITE, 0)
        self._command(servo2, SERVO_LED_CTRL_WRITE, 0)
        self._command(servo3, SERVO_LED_CTRL_WRITE, 0)
        self._command(servo4, SERVO_LED_CTRL_WRITE, 0)




if __name__ == "__main__":
    main()
    #getPositions()
