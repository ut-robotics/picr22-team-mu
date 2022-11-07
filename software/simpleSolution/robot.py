import serial
import struct

class Robot:
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial('/dev/ttyACM0')
        self.speed = 5 / 32767


    def setSpeed(self, speed):
        self.speed = speed


    def stop(self):
        self.move(0, 0, 0)

    # self.speed - from 0 to 1, float, out of maximum speed
    # 32767 on max valid
    # -32768 on min valid
    def forward(self):
        speed = int(self.speed * 32767)
        self.move(0, speed, -speed)


    def backward(self):
        speed = int(self.speed * 32767)
        self.move(0, -speed, speed)


    # portion - turning / driving straight: 1 - spin // 0 - straight
    def orbit(self, portion, forward=True, left=True):
        if left:
            speed1 = portion
            if forward:
                speed2 = 1
                speed3 = 2 * portion - 1
            else:
                speed2 = 2 * portion - 1
                speed3 = 1
        else:
            speed1 = -portion
            if forward:
                speed2 = 1 - 2 * portion
                speed3 = -1
            else:
                speed2 = -1
                speed3 = 1 - 2 * portion
        
        speed1 = int(speed1 * 32767 * self.speed)
        speed2 = int(speed2 * 32767 * self.speed)
        speed3 = int(speed3 * 32767 * self.speed)
        self.move(speed1, speed2, speed3)

    
    def spinLeft(self):
        speed = int(self.speed * 32767)
        self.move(speed, speed, speed)


    def spinRight(self):
        speed = int(self.speed * 32767)
        self.move(-speed, -speed, -speed)

    
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    def move(self, speed1 : int, speed2 : int, speed3 : int, throwerSpeed=0, disableFailsafe=0, delimiter=0xAAAA) -> None:
        print(speed1, speed2, speed3)
        self.ser.write(struct.pack("<hhhHBH", speed1, speed2, speed3, throwerSpeed, disableFailsafe, delimiter))
        # self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))


if __name__ == "__main__":
    import time
    r = Robot()
    # r.orbit(1, 20 / 32767, forward=False, left=False)
    startTime = time.time()
    while time.time() - startTime < 2:
        r.move(-15, -15, -15) # liigub paremale