from robotInterface import RobotInterface
import serial

class Robot(RobotInterface):
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial('/dev/ttyACM0')


    def forward(self):
        pass


    def backward(self):
        pass


    def orbit(self):
        pass

    
    def spinLeft(self):
        pass


    def spinRight(self):
        pass

    
    def left(self):
        pass


    def right(self):
        pass


    def move(self):
        self.ser.write(bytes(bytearray.fromhex('100010001000000000AAAA')))

if __name__ == "__main__":
    r = Robot()
    r.move()