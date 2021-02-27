#! /usr/bin/env python

import sys
sys.path.insert(0, "..")
import time
import logging
import os
from opcua import Client
#from opcua import ua



if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)
    IP = os.environ['ROS_IP']
    client = Client("opc.tcp://{}:12345".format(IP))
    server_up=False
    #print("Trying to connect to server opc.tcp://{}:12345".format(IP))
    try:
        client.connect()
        server_up=True
        root = client.get_root_node()
        status=client.get_node('ns=2;s="status"')
        
        #objects = client.get_objects_node()
        if (len(sys.argv)>1):    
            if sys.argv[1]=="save":
                save = client.get_node('ns=2;s="save"')
                save.set_value(True)
            elif sys.argv[1]=="goto":
                if (len(sys.argv)<=2):print("needs a name for a goal")
                else: 
                    navgoal=client.get_node('ns=2;s="navgoal"')
                    navgoal.set_value(int(sys.argv[2]))
                    execute= client.get_node('ns=2;s="start"')
                    execute.set_value(True)
            elif sys.argv[1]=="status":
                status=client.get_node('ns=2;s="status"')
                print(status.get_value())
            
            elif sys.argv[1]=="location":
                location=client.get_node('ns=2;s="location"')
                print(str(location.get_value()))
            
            elif sys.argv[1]=="goals":
                goals=client.get_node('ns=2;s="goals"')
                print(len(goals.get_value())-1)
            
            elif sys.argv[1]=="reset":
                reset=client.get_node('ns=2;s="reset"')
                reset.set_value(True)
            
            elif sys.argv[1]=="cancel":
                cancel=client.get_node('ns=2;s="cancel"')
                cancel.set_value(True)

            elif sys.argv[1]=="exclude":
                if (len(sys.argv)<=2):print("needs a name for a goal")
                else: 
                    ex_goal=client.get_node('ns=2;s="exclude"')
                    ex_goal.set_value(int(sys.argv[2]))
            elif sys.argv[1]=="offset":
                if len(sys.argv)>2:
                    x=client.get_node('ns=2;s="x"')
                    x.set_value(float(sys.argv[2]))
                if len(sys.argv)>3:
                    y=client.get_node('ns=2;s="y"')
                    y.set_value(float(sys.argv[3]))
                if len(sys.argv)>4:
                    yaw=client.get_node('ns=2;s="yaw"')
                    yaw.set_value(float(sys.argv[4])/57.29)
                offset=client.get_node('ns=2;s="offset"')
                offset.set_value(True)

            
            elif sys.argv[1]=="program":
                goals=client.get_node('ns=2;s="goals"')
                navgoal=client.get_node('ns=2;s="navgoal"')
                execute= client.get_node('ns=2;s="start"')
                i=1
                print(status.get_value())
                while True:
                    
                    navgoal.set_value(i)
                    execute.set_value(True)
                    i+=1
                    time.sleep(5)


                    while status.get_value()!="Ready":
                        time.sleep(5)

                    
                    if i==len(goals.get_value()): 
                        print("Last goal")
                        i=1

        
        else:
			
			print("None")
            
        #struct_array = client.get_node("ns=2;i=10323")
        #before = struct.get_value()
        #before_array = struct_array.get_value()
        client.load_type_definitions()  # scan server for custom structures and import them
        #after = save.set_value(True)
        #after= start.set_value(True)
        #struct.set_value(not after)
        #after_array = struct_array.get_value()
        #print after
    except:
        print("Server is offline")
    
    finally:
        if server_up: client.disconnect()
        else: "Client fail to connect"
            

       