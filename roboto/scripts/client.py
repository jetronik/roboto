#! /usr/bin/env python

from opcua import Client


client=Client("opc.tcp://127.0.0.1:12345")
client.connect()

#client.get_namespace_array()
objects=client.get_objects_node()
#print objects
bulb=objects.get_children()[2]
tempsens=objects.get_children()[1]


state = bulb.get_children()[0]

try:
    #print state
    #print state.get_value()
    
    #print get_attribute(client, "TS1_Temperature")
    state.set_value(True)

    

    for i in objects.get_children():
        node=i.get_children()[0]
        print node
        #print node.get_child(["0:TS1 Vendor Name"])

    #print tempsens.get_attribute("TS1_Temperature").get_value()
    #get root object
    #root = client.get_root_node()
    #print root
    
    # gettting our namespace idx
    uri = "AGV"
    idx = client.get_namespace_index(uri)
    print idx

    # Now getting a variable node using its browse path
    #myvar = objects.get_child([ "{}:MyObject".format(2), "{}:MyVariable".format(1)])
    #obj = root.get_child(["0:Objects", "{}:MyObject".format(idx)])
    #print("myvar is: ", myvar)
        

finally:

    client.disconnect()




