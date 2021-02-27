#! /usr/bin/env python

from time import sleep
import random
from opcua import Server

#Setup server
server=Server()
server.set_endpoint("opc.tcp://127.0.0.1:12345")
server.register_namespace("AGV")

#get objects refference
objects=server.get_objects_node()

tempsens=objects.add_object('ns=2;s="TS1"', "Temperature Sensor 1")

tempsens.add_variable('ns=2;s="TS1_VendorName"', "TS1 Vendor Name", "Sensor King")
tempsens.add_variable('ns=2;s="TS1_SerialNumber"', "Serial Number", 1234)
temp=tempsens.add_variable('ns=2;s="TS1_Temperature"', "Temperature", 20)
tempsens.add_variable('ns=2;s="TS1_array"', "myarrayvar", [6.7, 7.9])


bulb=objects.add_object('ns=2;s="Bulb"', "Light Bulb")
print bulb

state=bulb.add_variable('ns=2;s="Bulb_State"', "State of Ligth Bulb", False)
state.set_writable()

temperature =20.00

try:
    print("Starting Server")
    server.start()
    print("Server Online")
    while True:
        temperature = random.uniform(-1,1)
        temp.set_value(temperature)
        print ("New Temperature: " + str(temp.get_value()))
        print ("State of light bulb " + str(state.get_value()))
        sleep(2)
finally:
    server.stop()
    print ("Server offline")


