from re import S
import serial
import struct

FlightData_struct = struct.Struct('< I f f f f f f f f f f f f')
STRUCT_SIZE = 52
MAGIC = 0xBEEFF00D


#print(ser.name)
ser = serial.Serial('COM5', 115200)

while False:
    if ser.read() == b'\r':
        if ser.read() == b'\xf0':
            ser.read(2)
            unpacked = ser.read(STRUCT_SIZE)
            packet = FlightData_struct.unpack(unpacked)
            print(packet[0], packet[1], packet[2], packet[3], packet[4],  packet[5], packet[6], packet[7], packet[8], packet[9], packet[10], packet[11])
    else:
        print("Something went wrong")
        print(ser.read())

while True:
    unpacked = ser.read(STRUCT_SIZE)
    packet = FlightData_struct.unpack(unpacked)
    print(hex(int(packet[0])))
    print(packet)
