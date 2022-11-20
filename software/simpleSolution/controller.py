import basketBallDriver
import socket


def main():
    # basketBallDriver.main(True)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.05)
        s.bind(("127.0.0.1", 6969))
        s.listen(1)
        while True:
            conn, addr = None, None
            while True:
                try:
                    conn, addr = s.accept()
                except socket.timeout:
                    print("waiting")
                    continue
                break
            print("Got connection")
            generator = basketBallDriver.main(True)
            mode = "REMOTE CONTROL"
            while True:
                try:
                    data = conn.recv(1024)
                    mode = "REMOTE CONTROL"
                    text = data.decode().rstrip()
                    conn.send(b"ack")
                    print(text)
                except socket.timeout:
                    if mode != "REMOTE CONTROL":
                        next(generator)
                except:
                    conn.close()
                    break


if __name__ == "__main__":
    main()