#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, socket
import numpy as np

class SocketServer(object):
    def __init__(self, host="127.0.0.1", port=20001):
        self.host = host
        self.port = port
        self.socket_server = socket.socket()
        self.socket_server.bind((host, port))
        self.socket_server.setblocking(True)
        self.socket_server.listen(1)
        print("server listen on {}:{}".format(self.host, self.port))

    def recv_loop(self):
        while True:
            print("server waiting connection")
            self.conn, self.address = self.socket_server.accept()
            print("accept connection from:{}".format(self.address))
            while True:
                # receive data stream. it won't accept data packet greater than 1 bytes
                # data = self.conn.recv(1).decode()
                msg = self.conn.recv(1)
                if not msg:
                    print("recv error, server quit")
                    break
                data = np.int8(msg[0])
                print("recv msg from client: {}, data: {}".format(msg, data))

                # send data back to the client
                # self.conn.send(data.encode())
            # close the connection
            self.conn.close()
            print("close connection with:{}".format(self.address))
            time.sleep(1)

    def start_recv_thread(self):
        self.th = threading.Thread(target=self.recv_loop, args=())
        # self.th.setDaemon(True)
        self.th.start()
        print("recv thread start")


class SocketClient(object):
    def __init__(self, host="127.0.0.1", port=20001):
        self.host = host
        self.port = port
        self.socket_client = socket.socket()
        self.socket_client.connect((host, port))  # connect to the server
        print("client connect to {}:{}".format(self.host, self.port))

    def send_byte(self, byte_data=-1):
        msg = bytearray([byte_data])
        print("send msg: {}".format(msg))
        self.socket_client.send(msg)
        return True


def run_test1():
    s_server = SocketServer(host="127.0.0.1", port=20001)
    s_server.start_recv_thread()

    s_client = SocketClient(host="127.0.0.1", port=20001)
    s_client.send_byte(byte_data=np.uint8(-1))
    time.sleep(1)
    s_client.send_byte(byte_data=np.uint8(10))
    time.sleep(1)
    s_client.send_byte(byte_data=np.uint8(11))
    time.sleep(1)

def run_test2():
    for idx in range(100):
        s_client = SocketClient(host="127.0.0.1", port=20001)
        s_client.send_byte(byte_data=np.uint8(-2))
        time.sleep(1)
        s_client.send_byte(byte_data=np.uint8(20))
        time.sleep(1)
        s_client.send_byte(byte_data=np.uint8(21))
        time.sleep(1)

if __name__ == "__main__":
    print("********** running test1 ************")
    run_test1()
    time.sleep(2)
    print("********** running test2 ************")
    run_test2()