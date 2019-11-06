#!/usr/bin/env python3

import rospy
import time
import requests


import actionlib

#from our_ur.msg import RobotAction, RobotResult, RobotFeedback
import our_ur.msg

from mes_client import MesClient 
from pymodbus_client import ModbusClient

#------------------------------------------------------------------#
# STATES

class State(object):
    def __init__(self, FSM):
        self.FSM = FSM
        self.timer = 0
        self.startTime = 0
        self.totalLocalTime = 0
        self.totalTime = 0

    def Enter(self):
        self.startTime = int(time.time()*1000)

    def Exceute(self):
        pass

    def Exit(self):
        self.totalTime += self.totalLocalTime


# --------- STOPPED -------- #
class Stopped(State):
    def __init__(self, FSM):
        super(Stopped, self).__init__(FSM)

    def Enter(self):
        super(Stopped, self).Enter()

    def Execute(self):

        ## WAIT FOR OBERATOR TO RESET THE SYSTEM
        ## MAYBE WAIT FOR THE SYSTEM TO BE COMPLETLY STOPPED
        if(self.FSM.GetCmd() == 'reset'):
            self.FSM.ToTransition("toResetting")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Stopped, self).Exit()


# --------- IDLE -------- #
class Idle(State):
    def __init__(self, FSM):
        super(Idle, self).__init__(FSM)

    def Enter(self):
        super(Idle, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopping")
        if(self.FSM.GetCmd() == 'start'):
            self.FSM.ToTransition("toStarting")
        self.totalLocalTime = int(time.time()*1000) - self.startTime
    def Exit(self):
        super(Idle, self).Exit()


# --------- Execute -------- #
class Execute(State):
    def __init__(self, FSM):
        super(Execute, self).__init__(FSM)
        self.exeState = "getOrderFromServer"
        
        self.action_list = None
        self.action_index = 0

        self.startTimer1 = 0

        # MES CLIENT
        self.MES_URL = 'http://localhost:5000/orders'
        self.mes_client = MesClient(self.MES_URL)

        # MODBUS CLIENT
        self.MODBUS_IP = "10.42.0.142"
        self.MODBUS_PORT = "5020"
        self.modbus_client = ModbusClient(self.MODBUS_IP, self.MODBUS_PORT)

        # ROBOT ACTION CLIENT
        self.action_name = 'robot'
        self.action_client = actionlib.SimpleActionClient(self.action_name, our_ur.msg.RobotAction)
        
    

    def Enter(self):
        super(Execute, self).Enter()

    def GetExecuteState(self):
        return self.exeState

    def RobotAction(self, action_id):
        self.action_client.send_goal(our_ur.msg.RobotGoal(action_id))

    def Execute(self):

        ## ------------- TRYING TO GET AN ORDER AND TICKET ID ----------- ##

        if(self.exeState == "getOrderFromServer"):
            if(self.mes_client.get_order()): 
                self.action_list = self.mes_client.actionList 
                self.exeState = "pickUpBrick"
            else: 
                pass

        ## ------------- WHEN A ORDER IS READY WE NEED TO WAIT FOR MIR ROBOT TO ARRIVE ----------- ##
        elif(self.exeState == "waitForMirRobot"):

            ## CALL MIR ROBOT WITH SOME COMMAND
            ## IF MIRROBOT = IN POISITION THEN NEXT STATE
            self.RobotAction(action = 1)                                 ## ACTION = 1 - INDICATE PICKUP BOXES FROM MIR ROBOT
            self.exeState = "pickUpBrick"

        ## ------------- WHEN MIR ROBOT IS IN POSITION WE MUST GET THE BOXES FROM THE ROBOT ----------- ##
        elif(self.exeState == "waitPickupBox"):
            ## Wait for the result to be different from None
            if(self.client.get_result() != None):
                if(self.client_result() == True):
                    self.exeState = "pickUpBrick"
                else:
                    ## Maybe we should stop the system if the action failed
                    print("error")

        ## ------------- WHEN BOXES ARE IN PLACE PICK UP BRICK ----------- ##
        elif(self.exeState == "pickUpBrick"):
            if( self.action_index < len(self.action_list)):                       ## Keep track of the brick index is less than the size of the brick_lsit
                self.RobotAction(self.action_list[self.action_index])             ## Send action to robot
                self.exeState = "waitPickUpBrickDone"
            else:
                ## done itteration through bricks
                self.exeState = "complete"


        ## ------------- WHEN BOXES ARE IN PLACE PICK UP BRICK ----------- ##
        elif(self.exeState == "waitPickUpBrickDone"):
            #if(self.action_client.get_result() != None):
            #    if(self.action_client.get_result() == True):
            #        print ("Done picking up brick!")
            self.exeState = "checkBrick"

        ## ------------- WRITES A COMMAND TO MAKE THE PI CHECK THE CAMERA----------- ##
        elif(self.exeState == "checkBrick"):
            self.modbus_client.writeRegister(0x00, 0)
            self.startTimer1 = int(time.time())
            self.exeState = "checkBrickReponse"
            rospy.loginfo("WAITING FOR BRICK %s",self.action_list[self.action_index])
        
        
        ## ------------- CHECKS WHICH RESPOND WE GET FROM THE PI----------- ##
        elif(self.exeState == "checkBrickReponse"):
            if(self.modbus_client.readRegister(0x00) != 0): 
                #rospy.loginfo("Brick ID needed is [%f] and brick found is [%f]", self.action_list[self.action_index], self.modbus_client.readRegister(0x00))
                if(self.modbus_client.readRegister(0x00) == self.action_list[self.action_index]):
                    rospy.loginfo("BRICK IS CORRECT")
                    self.exeState = "putBrickInBox"

                elif(self.modbus_client.readRegister(0x00) == 4):
                    rospy.logerr("ERROR - NO BRICK FOUND!")
                    self.exeState = "discardBrick"
                else:
                    rospy.logerr("ERROR - WRONG COLOR")
                    self.exeState = "discardBrick"
                    pass
            
            if(int(time.time()) - self.startTimer1 > 10):
                rospy.logerr("TIMEOUT - NO RESPONSE FROM RASPBERRY PI [10 SEC]")
                self.exeState = "checkBrick"

        ## ------------- ----------- ##
        elif(self.exeState == "discardBrick"):
            rospy.loginfo("DISCARDING BRICK!")
            self.exeState = "pickUpBrick"


        ## ------------- WHEN ALL THE BRICKS HAVE BEEN PICKED AND PLACED ----------- ##
        elif(self.exeState == "putBrickInBox"):
            rospy.loginfo("PUTTING BRICK IN BOX")
            self.action_index+=1
            self.exeState = "pickUpBrick"

        ## ------------- COMPLETE ----------- ##
        elif(self.exeState == "complete"):
            rospy.loginfo("DONE!")

        ## ------------- WHEN THERE IS AN ERROR IN THE STATE NAME ----------- ##
        else:
            print ("STATE NAMING ERROR!")



        if(self.FSM.GetCmd() == 'hold'):
            self.FSM.ToTransition("toHolding")
        if(self.FSM.GetCmd() == 'suspend'):
            self.FSM.ToTransition("toSuspending")
        if(self.FSM.GetCmd() == 'sc'):
            self.FSM.ToTransition("toCompleting")
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopping")
        self.totalLocalTime = int(time.time()*1000)  - self.startTime

    def Exit(self):
        super(Execute, self).Exit()



# --------- Complete -------- #
class Complete(State):
    def __init__(self, FSM):
        super(Complete, self).__init__(FSM)

    def Enter(self):
        super(Complete, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'reset'):
            self.FSM.ToTransition("toResetting")
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopping")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Complete, self).Exit()


# --------- Aborted -------- #
class Aborted(State):
    def __init__(self, FSM):
        super(Aborted, self).__init__(FSM)

    def Enter(self):
        super(Aborted, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'clear'):
            self.FSM.ToTransition("toClearing")
        self.totalLocalTime = int(time.time()*1000) - self.startTime
    def Exit(self):
        super(Aborted, self).Exit()


# --------- HELD -------- #
class Held(State):
    def __init__(self, FSM):
        super(Held, self).__init__(FSM)

    def Enter(self):
        super(Held, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'unhold'):
            self.FSM.ToTransition("toUnholding")
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopping")
        self.totalLocalTime = int(time.time()*1000) - self.startTime
    def Exit(self):
        super(Held, self).Exit()


# --------- Suspended -------- #
class Suspended(State):
    def __init__(self, FSM):
        super(Suspended, self).__init__(FSM)

    def Enter(self):
        super(Suspended, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'unsuspended'):
            self.FSM.ToTransition("toUnsuspending")
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopping")
        self.totalLocalTime = int(time.time()*1000) - self.startTime
    def Exit(self):
        super(Suspended, self).Exit()


#------------------------------------------------------------------#
# TRANSITION STATES

# --------- STOPPING -------- #
class Stopping(State):
    def __init__(self, FSM):
        super(Stopping, self).__init__(FSM)

    def Enter(self):
        super(Stopping, self).Enter()

    def Execute(self):

        ## STOP THE ROBOT (PAUSED MODE)
        ## SET THE LIGHT TO INDICATE STOPPING/STOPPED
        ## MACHINE IS POWERED AND STATIONARY - CONTROLLED STOP
        self.FSM.ToTransition("toStopped")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Stopping, self).Exit()

# --------- RESETTING -------- #
class Resetting(State):
    def __init__(self, FSM):
        super(Resetting, self).__init__(FSM)

    def Enter(self):
        super(Resetting, self).Enter()

    def Execute(self):

        ## RESET STUFF NEEDED BEFORE A COMPLETE/STOPPED SYSTEM
        ## SET LIGHT TO INDICATE RESETTING
        ## MAYBE HOME THE ROBOT - SHOULD WE HOME THE ROBOT AFTER EACH 'SOFT-STOP'?
        ## IF SO, WAIT FOR THE ROBOT TO BE IN ITS HOME POSITION
        ## WE SHOULDN'T GIVE UP AN ORDER WHEN EVER WE HAVE A STOP OR MAYBE WE SHOULD
        self.FSM.ToTransition("toIdle")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Resetting, self).Exit()

# --------- STARTING -------- #
class Starting(State):
    def __init__(self, FSM):
        super(Starting, self).__init__(FSM)

    def Enter(self):
        super(Starting, self).Enter()

    def Execute(self):

        ## SET LIGHT TO INDICATE STARTING
        ## SET STUFF THAT NEEDS TO BE READY BEFORE EXECUTE (RAMP UP SPEED, OTHER MECHANICAL PARTS)
        ## MAYBE TEST SERVER CONECTION TO PLC, RASPBERRY OR MES
        ## IF SOMETHING FAIL (MAYBE USE TIMER VALUE) - GO TO STOPPING STATE
        self.FSM.ToTransition("toExecute")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Starting, self).Exit()

# --------- COMPLETING -------- #
class Completing(State):
    def __init__(self, FSM):
        super(Completing, self).__init__(FSM)

    def Enter(self):
        super(Completing, self).Enter()

    def Execute(self):

        ## SET LIGHT TO INDICATE COMPLETING/COMPLETED
        ## THIS STEP CAN BE ACTIVATED WHEN A CERTAIN UNIT COUNTER HAVE BEEN REACHED
        ##
        self.FSM.ToTransition("toComplete")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Completing, self).Exit()


# --------- ABORTING -------- #
class Aborting(State):
    def __init__(self, FSM):
        super(Aborting, self).__init__(FSM)

    def Enter(self):
        super(Aborting, self).Enter()

    def Execute(self):

        ## SET LIGHT TO INDICATE ABORTING/ABORTED
        ## BRING MACHINE TO RAPID SAFESTOP
        ## CAN CONTINUE AFTER ABORT CONDITION HAVE BEEN CORRECTED
        self.FSM.ToTransition("toAborted")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Aborting, self).Exit()

# --------- CLEARING -------- #
class Clearing(State):
    def __init__(self, FSM):
        super(Clearing, self).__init__(FSM)

    def Enter(self):
        super(Clearing, self).Enter()

    def Execute(self):
        self.FSM.ToTransition("toStopped")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Clearing, self).Exit()

# --------- HOLDING -------- #
class Holding(State):
    def __init__(self, FSM):
        super(Holding, self).__init__(FSM)

    def Enter(self):
        super(Holding, self).Enter()

    def Execute(self):

        ## SET LIGHT TO INDICATE HELD
        ## WHEN SOME INTERNAL MACHINE CONDITIONS DO NOT ALLOW THE MACHINE TO CONTINUE
        ## EXAMPLE - SOMETHING WITHIN THE MACHINE NEEDS TO BE REFILLED WHICH IS NOT POSSIBLE WHEN THE SYSTEM IS RUNNING
        ## STOP THE ROBOT
        self.FSM.ToTransition("toHeld")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Holding, self).Exit()

# --------- UNHOLDING -------- #
class Unholding(State):
    def __init__(self, FSM):
        super(Unholding, self).__init__(FSM)

    def Enter(self):
        super(Unholding, self).Enter()

    def Execute(self):

        ## SET LIGHT TO INDICATE UNHOLDING/EXECUTING
        ## SOME OF THE SAME FUNCTIONS AS STARTING - CHECK IF EVERYTHING IS READY FOR PRODUCTION
        self.FSM.ToTransition("toExecute")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Unholding, self).Exit()

# --------- SUSPENDING -------- #
class Suspending(State):
    def __init__(self, FSM):
        super(Suspending, self).__init__(FSM)

    def Enter(self):
        super(Suspending, self).Enter()

    def Execute(self):

        ## SET THE LIGHT TO INDICATE SUSPENDING/SUSPENDED
        ## IF EXTERIOR SYSTEMS ARE NOT FUNCTIONALL. THIS COULD BE INFEED PROBLEMS OR OUTFEED BLOCKAGE.
        ## STOP THE ROBOT OR MAYBE STOP THE ROBOT FROM MOVING INTO REFILL ZONE (ONLY NESSASARY WITH FEEDER MEASUREMENT SYSTEM)
        self.FSM.ToTransition("toSuspended")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Suspending, self).Exit()

# --------- UNSUSPENDING -------- #
class Unsuspending(State):
    def __init__(self, FSM):
        super(Unsuspending, self).__init__(FSM)

    def Enter(self):
        super(Unsuspending, self).Enter()

    def Execute(self):

        ## SET THE LIGHT TO INDICATE UNSUSPENDIG/EXECUTE
        ## SOME OF THE SAME FUNCTIONS AS STARTING - COULD CHECK THAT THE SYSTEM IS FILLED UP
        self.FSM.ToTransition("toExecute")
        self.totalLocalTime = int(time.time()*1000) - self.startTime

    def Exit(self):
        super(Unsuspending, self).Exit()
