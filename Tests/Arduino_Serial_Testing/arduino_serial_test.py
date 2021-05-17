# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
import serial
import numpy as np
from time import sleep
import struct
# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
class Arduino():
    def __init__(self, port):
        self.board = serial.Serial(port, baudrate=9600, timeout=5)
        sleep(1)
    
    def __write(self, message):
        self.board.write(message.encode('ascii'))
    
    def __read(self):
        self.line = self.board.readline().decode('ascii').strip()
        
    def __write_byte(self, float_data):
        #self.board.write(b"<" + struct.pack("f", float_data) + b">")
        self.board.write(struct.pack("f", float_data))
        
    def __read_byte(self):
        float_data = self.board.read(4) # Refer IEEE 754
        self.float_return = struct.unpack('f', float_data)[0]
        
    def query(self, message):
        self.__write(message)
        self.__read()
        return(self.line)
        
    def query_byte(self, message):
        self.__write_byte(message)
        self.__read_byte()
        return(self.float_return)

    def disconnect(self):
        self.board.close()
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def LED_test():
    uno = Arduino('COM3')

    for _ in range(10):
        print(uno.query('1'))
        sleep(0.5)
        print(uno.query('0'))
        sleep(0.5)

    uno.disconnect()


def serial_read_write():
    uno = Arduino('COM3')
    i = 0

    time_values = np.linspace(0, 2*np.pi, 10)
    send_values = np.sin(time_values)  # np.zeros(10)

    for val in send_values:
        print(i)
        print("Sent: ", val)
        received = uno.query_byte(val)
        print("Received: ", received)
        if abs(val - received) < 0.0001:
            print("Correct\n")
        else:
            print("Incorrect\n")
        i += 1
    uno.disconnect()
# --------------------------------------------------------------------------- #


#LED_test()
serial_read_write()
