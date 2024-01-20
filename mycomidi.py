#!/usr/bin/python
# -*- coding:utf-8 -*-

#import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import ADS1256
import RPi.GPIO as GPIO
#import serial
import time
import numpy as np
import collections
import rtmidi

trigger_down = -1
trigger_up = 1
buffer = 100 #Samples
spike = 0

midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()

d0 = collections.deque(maxlen=buffer)
d1 = collections.deque(maxlen=buffer)
d2 = collections.deque(maxlen=buffer)
d3 = collections.deque(maxlen=buffer)
d4 = collections.deque(maxlen=buffer)
d5 = collections.deque(maxlen=buffer)
d6 = collections.deque(maxlen=buffer)
d7 = collections.deque(maxlen=buffer)

MQTT_HOST = 'localhost'
#MQTT_TOPICNote = 'mycomidi/notes/note'
MQTT_TOPIC0 = 'mycomidi/ADC_values/0/spike'
MQTT_TOPIC1 = 'mycomidi/ADC_values/1/spike'
MQTT_TOPIC2 = 'mycomidi/ADC_values/2/spike'
MQTT_TOPIC3 = 'mycomidi/ADC_values/3/spike'
MQTT_TOPIC4 = 'mycomidi/ADC_values/4/spike'
MQTT_TOPIC5 = 'mycomidi/ADC_values/5/spike'
MQTT_TOPIC6 = 'mycomidi/ADC_values/6/spike'
MQTT_TOPIC7 = 'mycomidi/ADC_values/7/spike'

#ser = serial.Serial('/dev/ttyAMA0', baudrate=(31250*25))

if available_ports:
    midiout.open_port(0)
else:
    midiout.open_virtual_port("MycoMIDI")

def moving_average(a, n=buffer):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    avg_out = (sum(ret[n - 1:] / n)) / len(a)
    return avg_out * n
def note_on(moving_average, ADC_val, channel, min, max):
    if (ADC_val > moving_average):
        spike = (ADC_val-moving_average)
    if (ADC_val < moving_average):
        spike = (moving_average-ADC_val)
    note = int(np.interp(spike, [0,2.5],[min,max])) #1.5v seems about average for weaker spikes
    velocity = int(np.interp(spike, [0,2.5],[50,127]))
    msg_note_on  = bytearray([(9 << 4) | channel, note, velocity])
    #publish.single(MQTT_TOPICNote, payload=str(note), hostname=MQTT_HOST)
    return note, spike, msg_note_on
def note_off(note, channel):
    msg_note_off = bytearray([(8 << 4) | channel, note, 50])
    return msg_note_off

def loop_fn(channel, ADC_voltage, buffer, trigger_up, trigger_down, notemin=10, notemax=100):
        if (ADC_voltage - moving_average(buffer)) >= trigger_up:
                note, spike, msg_note_on = note_on(moving_average(buffer), ADC_voltage, channel, notemin, notemax)
                #ser.write(msg_note_on)
                midiout.send_message(msg_note_on)
                time.sleep((spike/30))
                msg_note_off = note_off(note, channel)
                #ser.write(msg_note_off)
                midiout.send_message(msg_note_off)
        if (ADC_voltage - moving_average(buffer)) <= trigger_down:
                note, spike, msg_note_on = note_on(moving_average(buffer), ADC_voltage, channel, notemin, notemax)
                #ser.write(msg_note_on)
                midiout.send_message(msg_note_on)
                time.sleep((spike/30))
                msg_note_off = note_off(note, channel)
                #ser.write(msg_note_off)
                midiout.send_message(msg_note_off)

ADC = ADS1256.ADS1256()
ADC.ADS1256_init()
note = 0

while(True):
    ADC_Value = ADC.ADS1256_GetAll()
    ADC_Value0_num = (ADC_Value[0]*5.0/0x7fffff)
    ADC_Value1_num = (ADC_Value[1]*5.0/0x7fffff)
    ADC_Value2_num = (ADC_Value[2]*5.0/0x7fffff)
    ADC_Value3_num = (ADC_Value[3]*5.0/0x7fffff)
    ADC_Value4_num = (ADC_Value[4]*5.0/0x7fffff)
    ADC_Value5_num = (ADC_Value[5]*5.0/0x7fffff)
    ADC_Value6_num = (ADC_Value[6]*5.0/0x7fffff)
    ADC_Value7_num = (ADC_Value[7]*5.0/0x7fffff)
    d0.append(ADC_Value0_num)
    d1.append(ADC_Value1_num)
    d2.append(ADC_Value2_num)
    d3.append(ADC_Value3_num)
    d4.append(ADC_Value4_num)
    d5.append(ADC_Value5_num)
    d6.append(ADC_Value6_num)
    d7.append(ADC_Value7_num)
    messages = [
    {'topic': MQTT_TOPIC0, 'payload': str((ADC_Value[0]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC1, 'payload': str((ADC_Value[1]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC2, 'payload': str((ADC_Value[2]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC3, 'payload': str((ADC_Value[3]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC4, 'payload': str((ADC_Value[4]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC5, 'payload': str((ADC_Value[5]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC6, 'payload': str((ADC_Value[6]*5.0/0x7fffff))},
    {'topic': MQTT_TOPIC7, 'payload': str((ADC_Value[7]*5.0/0x7fffff))}
    ]
    publish.multiple(messages, hostname=MQTT_HOST)
    if (len(d0) >= buffer): #Wait for buffer and moving average
        channel = 0
        loop_fn(channel, ADC_Value0_num, d0, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d1) >= buffer): #Wait for buffer and moving average
        channel = 1
        loop_fn(channel, ADC_Value1_num, d1, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d2) >= buffer): #Wait for buffer and moving average
        channel = 2
        loop_fn(channel, ADC_Value2_num, d2, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d3) >= buffer): #Wait for buffer and moving average
        channel = 3
        loop_fn(channel, ADC_Value3_num, d3, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d4) >= buffer): #Wait for buffer and moving average
        channel = 4
        loop_fn(channel, ADC_Value4_num, d4, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d5) >= buffer): #Wait for buffer and moving average
        channel = 5
        loop_fn(channel, ADC_Value5_num, d5, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d6) >= buffer): #Wait for buffer and moving average
        channel = 6
        loop_fn(channel, ADC_Value6_num, d6, trigger_up, trigger_down, notemin=10, notemax=100)
    if (len(d7) >= buffer): #Wait for buffer and moving average
        channel = 7
        loop_fn(channel, ADC_Value7_num, d7, trigger_up, trigger_down, notemin=10, notemax=100)
