#!/usr/bin/env python3
import rospy
import time
import threading
#from std_msgs.msg import String
from our_ur.msg import StateMsg

from packml_states import Stopped, Stopping, Starting, Execute, Complete, Completing, Holding, Held, Unholding, Suspending, Suspended, Unsuspending, Idle, Aborting, Aborted, Clearing, Resetting


#------------------------------------------------------------------#
# TRANSITION

class Transition(object):
    def __init__(self, toState):
        self.toState = toState

    def Execute(self):
        pass
        #print "Transitioning..."


#------------------------------------------------------------------#
# FINITE STATE MACHINE - FSM

class FSM(object):
    def __init__(self, character):
        self.char = character
        self.states = {}
        self.transitions = {}
        self.curState = None
        self.prevState = None
        self.trans = None
        self.cmd = None

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def SetState(self, stateName):
        self.prevState = self.curState
        self.curState = self.states[stateName]

    def GetCmd(self):
        return self.cmd

    def ToTransition(self, toTrans):
        self.trans = self.transitions[toTrans]

    def Execute(self, cmd):
        self.cmd = cmd
        
        if(self.trans):                             # If some other transaction have been set (Add self.curState.Exit() in begin if some exit function is needed)
            self.curState.Exit()                    # Saving timer value to class 
            self.SetState(self.trans.toState)       # Set the new current state 
            self.curState.Enter()                   # Enter the current state (start the time in that state)
            self.trans = None                       # Set transaction back to none

        self.curState.Execute()                     # Finally execute the current state 

#------------------------------------------------------------------#
# PACKML STATE MACHINE


class PACKML(object):
    def __init__(self):
        self.FSM = FSM(self)

        #STATES
        self.FSM.AddState("STOPPED", Stopped(self.FSM))
        self.FSM.AddState("IDLE", Idle(self.FSM))
        self.FSM.AddState("COMPLETE", Complete(self.FSM))
        self.FSM.AddState("EXECUTE", Execute(self.FSM))
        self.FSM.AddState("ABORTED", Aborted(self.FSM))
        self.FSM.AddState("HELD", Held(self.FSM))
        self.FSM.AddState("SUSPENDED", Suspended(self.FSM))

        #TRANSISTION STATES 
        self.FSM.AddState("STOPPING", Stopping(self.FSM))
        self.FSM.AddState("RESETTING", Resetting(self.FSM))
        self.FSM.AddState("STARTING", Starting(self.FSM))
        self.FSM.AddState("COMPLETING", Completing(self.FSM))
        self.FSM.AddState("ABORTING", Aborting(self.FSM))
        self.FSM.AddState("CLEARING", Clearing(self.FSM))
        self.FSM.AddState("SUSPENDING", Suspending(self.FSM))
        self.FSM.AddState("UNSUSPENDING", Unsuspending(self.FSM))
        self.FSM.AddState("HOLDING", Holding(self.FSM))
        self.FSM.AddState("UNHOLDING", Unholding(self.FSM))

        #TRANSITIONS
        self.FSM.AddTransition("toStopped", Transition("STOPPED"))
        self.FSM.AddTransition("toIdle", Transition("IDLE"))
        self.FSM.AddTransition("toComplete", Transition("COMPLETE"))
        self.FSM.AddTransition("toExecute", Transition("EXECUTE"))
        self.FSM.AddTransition("toAborted", Transition("ABORTED"))
        self.FSM.AddTransition("toHeld", Transition("HELD"))
        self.FSM.AddTransition("toSuspended", Transition("SUSPENDED"))


        self.FSM.AddTransition("toStopping", Transition("STOPPING"))
        self.FSM.AddTransition("toResetting", Transition("RESETTING"))
        self.FSM.AddTransition("toStarting", Transition("STARTING"))
        self.FSM.AddTransition("toCompleting", Transition("COMPLETING"))
        self.FSM.AddTransition("toAborting", Transition("ABORTING"))
        self.FSM.AddTransition("toClearing", Transition("CLEARING"))
        self.FSM.AddTransition("toSuspending", Transition("SUSPENDING"))
        self.FSM.AddTransition("toUnsuspending", Transition("UNSUSPENDING"))
        self.FSM.AddTransition("toHolding", Transition("HOLDING"))
        self.FSM.AddTransition("toUnholding", Transition("UNHOLDING"))


        self.FSM.SetState("STOPPED")
        self.FSM.curState.Enter()

    def Execute(self, cmd):
        if(cmd == 'abort'): self.FSM.ToTransition("toAborting")
        self.FSM.Execute(cmd)

    def GetStateTime(self):
        return (self.FSM.curState.totalTime + self.FSM.curState.totalLocalTime)

    def GetStateName(self):
        states = ["STOPPED", "IDLE", "COMPLETE", "EXECUTE", "ABORTED", "HELD", "SUSPENDED", "STOPPING", "RESETTING", "STARTING", "COMPLETING", "ABORTING", "CLEARING", "SUSPENDING", "UNSUSPENDING", "HOLDING", "UNHOLDING"]
        for state in states:
            if(self.FSM.curState == self.FSM.states[state]):
                return state
        return None

#------------------------------------------------------------------#
# LISENTER CALLBACK FUNCTION
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


#------------------------------------------------------------------#
# GETCH FOR INPUTS
import sys, tty, termios
from getch import getch

cmd = ''
ready = False

def getch_func():
    global cmd, ready
    ready = False
    cmd = input("> ")
    print('You pressed:', cmd)
    ready = True


if __name__ == '__main__':

    rospy.init_node('packml', anonymous=True)
    pub_state = rospy.Publisher('packml_state_info', StateMsg, queue_size = 1)

    r = rospy.Rate(30) ## 10 HERTZ


    pml = PACKML()

    startT = int(time.time())
    inputThread = threading.Thread()


    while not rospy.is_shutdown():
        # saving command if it is ready
        if(ready):
            command = cmd
            ready = False
        else:
            command = ''
        # Starting input thread if not alive
        if(not inputThread.isAlive() and not ready):
            inputThread = threading.Thread(target=getch_func, args=())
            inputThread.daemon = True
            inputThread.start()

        # Exit program it q is pressed
        if(command == 'quit'): break

        # The main state machine is executed.
        pml.Execute(command)

        # Publish the current state every 1 secound
        if((pml.GetStateName() != None )):
            msg = StateMsg()
            msg.name = pml.GetStateName()
            msg.time = pml.GetStateTime()/1000
            pub_state.publish(msg)
            startT =  int(time.time())
        
        r.sleep()
