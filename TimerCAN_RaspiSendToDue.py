#!/usr/bin/python3
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import os
import can
from multiprocessing import Process, Queue

count = 0
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
time.sleep(0.1)
bus = can.interface.Bus(channel='can0', bustype='socketcan_native')

triggerInputPin = 4
plus = 0

def can_rx_task():
        while True:
                message = bus.recv()
                q.put(message)            # Put message into queue

q = Queue()
t = Process(target = can_rx_task)    # Start receive thread
t .deamon = True
t.start()

class CANErrorMessage:
        def __init__(self):
                self.arbitration_id = 0xFFFF
                self.dlc = 8
                self.data = [0xFF]*8
                self.extended_id = False
                self.timestamp = float(0)

def getCANMessage(ID):
        while q.empty() != True:    # Check if there is a message in queue
                message = q.get()
                if message.arbitration_id == ID:
                        return message
        return CANErrorMessage()

def printCANMessage(message):
        c = '{0:f} {1:x} {2:x} '.format(message.timestamp, message.arbitration_id, message.dlc)
        s=''
        for i in range(message.dlc ):
                s +=  '{0:x} '.format(message.data[i])
        print(plus, ' {}'.format(c+s))

def callbackFromDue10ms(triggerInputPin):
        global plus

#        message = getCANMessage(0x407)
#        printCANMessage(message) #Id = ffff is no found Id error message

        msg = can.Message(arbitration_id=0x7de,data=[0x00,0x01,0x02, 0x03, 0x04, 0x05,0x06, count & 0xff],extended_id=False)
        bus.send(msg)

        plus += 1
        #print(plus)

GPIO.setmode(GPIO.BCM)
GPIO.setup(triggerInputPin, GPIO.IN)
GPIO.add_event_detect(triggerInputPin, GPIO.BOTH)
GPIO.add_event_callback(triggerInputPin, callbackFromDue10ms)

try:
    plus = 0
    while True:
        time.sleep(10)

finally:
    GPIO.cleanup()
    os.system("sudo /sbin/ip link set can0 down")
