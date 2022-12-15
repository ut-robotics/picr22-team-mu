import serial
import serial.tools.list_ports
import struct
import numpy as np

class Robot:
    def __init__(self):
        super().__init__()
        self.speed = 1
        self.motors = [
            {"r":38, "R":98, "angle": 1/6*np.pi},
            {"r":38, "R":98, "angle": 11/6*np.pi},
            {"r":22, "R":98, "angle": np.pi    }
        ]

        port = [i[0] for i in serial.tools.list_ports.comports()][0]
        self.ser = serial.Serial(port)

    def setSpeed(self, speed):
        self.speed = speed


    def stop(self):
        self.move(0, 0, 0)

    # self.speed - from 0 to 1, float, out of maximum speed
    # 32767 on max valid
    # -32768 on min valid

    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    def set_speed(self, speed1 : int, speed2 : int, speed3 : int, throwerSpeed=0, disableFailsafe=0, delimiter=0xAAAA) -> None:
        print(speed1, speed2, speed3)
        self.ser.write(struct.pack("<hhhHBH", speed1, speed2, speed3, throwerSpeed, disableFailsafe, delimiter))
        # self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))

    # radiaanides!!
    def move_omni(self, direction: float, velocity: float, turning: float) -> None:      
        speed = []        
        for i in range(len(self.motors)):
            speed += [int(-np.sin(self.motors[i]["angle"] - direction)*(velocity/self.motors[i]["r"])+(turning*self.motors[i]["R"]/self.motors[i]["r"]))]
        self.set_speed(*speed)

    
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    # motor range - 48-2047
    def move(self, speed3 : int, speed1 : int, speed2 : int, thrower_speed=0, disable_failsafe=0, delimiter=0xAAAA) -> None:
        print(speed1, speed2, speed3, thrower_speed)
        self.ser.write(struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, disable_failsafe, delimiter))
        # self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))


if __name__ == "__main__":
    import time
    r = Robot()
    start_time = time.time()
    while time.time() < start_time + 2:
        r.move_omni(np.pi/4, 1000, -5)
