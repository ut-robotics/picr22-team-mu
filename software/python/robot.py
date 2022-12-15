import serial
import serial.tools.list_ports
import struct

class Robot:
    def __init__(self):
        super().__init__()

        port = [i[0] for i in serial.tools.list_ports.comports()][0]
        self.ser = serial.Serial(port)
        self.speed = 5 / 32767


    def set_speed(self, speed):
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

    
    def spin_left(self):
        speed = int(self.speed * 32767)
        self.move(speed, speed, speed)


    def spin_right(self):
        speed = int(self.speed * 32767)
        self.move(-speed, -speed, -speed)

    
    # speed1 - tagumine
    # speed2 - parem
    # speed3 - vasak
    # motor range - 48-2047
    def move(self, speed1 : int, speed2 : int, speed3 : int, thrower_speed=0, disable_failsafe=0, delimiter=0xAAAA) -> None:
        print(speed1, speed2, speed3, thrower_speed)
        self.ser.write(struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, disable_failsafe, delimiter))
        # self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))


if __name__ == "__main__":
    import time
    r = Robot()
    start_time = time.time()
    while True:
        #r.orbit(0.5, forward=False, left=False)
        i = int(input(">"))
        #r.move(10, 10, 10) # liigub paremale
        r.move(i, i, i, 0, disable_failsafe=1)
