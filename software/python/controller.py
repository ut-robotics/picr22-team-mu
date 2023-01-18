import basket_ball_driver
import basket_enum
import constants
import image_detection
import img_processor
import robot
import state

import socket
import json
from enum import Enum
import cv2
import importlib


def fake_generator(basket):  # Temporary solver for testing
    while True:
        print("Program", basket)
        yield


def get_generator(r, basket):
    # importlib.reload(basket_ball_driver)
    # importlib.reload(basket_enum)
    # importlib.reload(constants)
    # importlib.reload(image_detection)
    # importlib.reload(img_processor)
    # importlib.reload(robot)
    # importlib.reload(state)
    import basket_ball_driver
    import basket_enum
    import constants
    import image_detection
    import img_processor
    import robot
    import state

    if (basket == "blue"):
        return basket_ball_driver.main(True, r, basket_enum.Basket.BLUE)
    else:
        return basket_ball_driver.main(True, r, basket_enum.Basket.MAGENTA)
    # return fake_generator(basket)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.01)
        s.bind(("127.0.0.1", 6969))
        s.listen(1)

        r = robot.Robot()

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
            get_frames = False
            while True:
                try:
                    data = conn.recv(1024)
                    text = data.decode().rstrip()
                    conn.send(b"ack")
                    get_frames = False
                    cv2.destroyAllWindows()
                    try:
                        obj = json.loads(text)
                        if 'mode' in obj and 'speeds' in obj:
                            if obj['mode'] == 'manual':
                                r.move(obj['speeds'][0], obj['speeds'][1], obj['speeds'][2], obj['speeds'][3])
                            elif obj['mode'] == 'auto' and 'basket' in obj:
                                generator = get_generator(r, obj['basket'])
                                next(generator)
                                get_frames = True
                    except Exception as e:
                        print(e)
                except socket.timeout:
                    if get_frames:
                        next(generator)
                except Exception as e:
                    print(e)
                    conn.close()
                    break


if __name__ == "__main__":
    main()