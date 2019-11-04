#!/usr/bin/env python3
import rtde_io

import time
import rospy

#from std_msgs.msg import String
from our_ur.msg import StateMsg

stateName = None

def setLight(io, green, yellow, red):
    io.setStandardDigitalOut(1, green) # GREEN
    io.setStandardDigitalOut(2, yellow) # YELLOW
    io.setStandardDigitalOut(3, red)  # RED
def callback(data):
    global stateName 
    stateName = data.name
    #print("State name from light control: ", data.name)

if __name__ == '__main__':
    rospy.init_node('light_node', anonymous=True)

    rospy.Subscriber('packml_state_info', StateMsg, callback)
    rtde_io = rtde_io.RTDEIOInterface("192.168.100.1") # 192.168.100.1

    now = int(time.time()*1000)

    while not rospy.is_shutdown():
        ## SOLID RED
        if(stateName == "STOPPED" or stateName == "STOPPING"):
            setLight(rtde_io, False, False, True)
        ## FLASHING GREEN
        elif(stateName == "IDLE"):
            if(int(time.time()*1000) - now < 500): 
                setLight(rtde_io, True, False, False)
            else:
                setLight(rtde_io, False, False, False)
            
            if(int(time.time()*1000) - now > 1000): 
                now = int(time.time()*1000)
        ## SOLID GREEN
        elif(stateName == "EXECUTE" or stateName == "STARTING" or stateName == "UNHOLDING" or stateName == "UNSUSPENDING"):
            setLight(rtde_io, True, False, False)
        ## FLASHING RED
        elif(stateName == "ABORTING" or stateName == "ABORTED" or stateName == "CLEARING"):
            if(int(time.time()*1000) - now < 500): 
                setLight(rtde_io, True, False, False)
            else:
                setLight(rtde_io, False, False, False)
            
            if(int(time.time()*1000) - now > 1000): 
                now = int(time.time()*1000)
        ## SOLID YELLOW
        elif(stateName == "SUSPENDING" or stateName == "SUSPENDED"):
            setLight(rtde_io, False, True, False)
        ## FLASHING YELLOW AND GREEN
        elif(stateName == "HELD"):    
            if(int(time.time()*1000) - now < 500): 
                setLight(rtde_io, True, True, False)    
            else:
                setLight(rtde_io, False, False, False)
            
            if(int(time.time()*1000) - now > 1000): 
                now = int(time.time()*1000)            
        else: 
            setLight(rtde_io, False, False, False)
            print("LIGHT STATE ERROR - MISSING STATE: ", stateName)

        