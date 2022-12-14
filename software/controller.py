import basketBallDriver
import socket
import json
from robot import Robot
from enum import Enum
import cv2


def fakeGenerator(basket):  # Temporary solver for testing
    while True:
        print("Program", basket)
        yield


def getGenerator(robot, basket):
    return basketBallDriver.main(True, robot, basket)
    # return fakeGenerator(basket)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.01)
        s.bind(("127.0.0.1", 6969))
        s.listen(1)

        robot = Robot()

        while True:
            conn, addr = None, None
            while True:
                try:
                    conn, addr = s.accept()
                    conn.settimeout(0.01)
                except socket.timeout:
                    print("waiting")
                    continue
                break
            print("Got connection")
            generator = None
            getFrames = False
            while True:
                try:
                    data = conn.recv(1024)
                    text = data.decode().rstrip()
                    conn.send(b"ack")
                    getFrames = False
                    cv2.destroyAllWindows()
                    try:
                        obj = json.loads(text)
                        if 'mode' in obj and 'speeds' in obj:
                            if obj['mode'] == 'manual':
                                robot.move(obj['speeds'][0], obj['speeds'][1], obj['speeds'][2], obj['speeds'][3])
                            elif obj['mode'] == 'auto' and 'basket' in obj:
                                generator = getGenerator(robot, obj['basket'])
                                next(generator)
                                getFrames = True
                    except Exception as e:
                        print(e)
                except socket.timeout:
                    if getFrames:
                        next(generator)
                except Exception as e:
                    print(e)
                    conn.close()
                    break


if __name__ == "__main__":
    main()