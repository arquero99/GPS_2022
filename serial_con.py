# Serial4.py

import io
import os

import pynmea2
import serial


ser = serial.Serial('COM5', 4800, timeout=0.5)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
clear = lambda: os.system('cls')

while 1:
    try:
        line = sio.readline()
        msg = pynmea2.parse(line)
        #print(msg.timestamp)
        #if(msg.sentence_type=='talker'):
            #print(repr(msg))

        print(repr(msg))
    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue