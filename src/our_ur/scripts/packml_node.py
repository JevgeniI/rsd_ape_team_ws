#!/usr/bin/env python
import rospy
import time
import threading
from std_msgs.msg import String

#------------------------------------------------------------------#
# STATES

class State(object):
    def __init__(self, FSM):
        self.FSM = FSM
        self.timer = 0
        self.startTime = 0

    def Enter(self):
        self.timer = 5
        self.startTime = int(time.time())

    def Exceute(self):
        pass

    def Exit(self):
        pass

# --------- STOPPED -------- #
class Stopped(State):
    def __init__(self, FSM):
        super(Stopped, self).__init__(FSM)

    def Enter(self):
        if(self.FSM.prevState == self.FSM.states["ABORTED"]):
            print "TRANSITION - CLEARING"
        else:
            print "TRANSITION - STOPPING"
        super(Stopped, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'reset'):
            self.FSM.ToTransition("toIdle")

        if(self.startTime + 2 <=  int(time.time())):
            #print ("STATE = STOPPED")
            self.startTime = int(time.time())

# --------- IDLE -------- #
class Idle(State):
    def __init__(self, FSM):
        super(Idle, self).__init__(FSM)

    def Enter(self):
        print "TRANSITION - RESETTING"
        super(Idle, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopped")
        if(self.FSM.GetCmd() == 'start'):
            self.FSM.ToTransition("toExecute")

        if(self.startTime + 2 <=  int(time.time())):
            #print ("STATE = IDLE")
            self.startTime = int(time.time())


# --------- Execute -------- #
class Execute(State):
    def __init__(self, FSM):
        super(Execute, self).__init__(FSM)

    def Enter(self):
        print "TRANSITION - STARTING"
        super(Execute, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopped")
        if(self.FSM.GetCmd() == 'sc'):
            self.FSM.ToTransition("toComplete")

        if(self.startTime + 2 <=  int(time.time())):
            #print ("STATE = EXECUTE")
            self.startTime = int(time.time())



# --------- Complete -------- #
class Complete(State):
    def __init__(self, FSM):
        super(Complete, self).__init__(FSM)

    def Enter(self):
        print "TRANSITION - COMPLETING"
        super(Complete, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'stop'):
            self.FSM.ToTransition("toStopped")
        if(self.FSM.GetCmd() == 'reset'):
            self.FSM.ToTransition("toIdle")

        if(self.startTime + 2 <=  int(time.time())):
            #print ("STATE = COMPLETE")
            self.startTime = int(time.time())

# --------- Aborted -------- #
class Aborted(State):
    def __init__(self, FSM):
        super(Aborted, self).__init__(FSM)

    def Enter(self):
        print "TRANSITION - ABORTING"
        super(Aborted, self).Enter()

    def Execute(self):
        if(self.FSM.GetCmd() == 'clear'):
            self.FSM.ToTransition("toStopped")
        if(self.startTime + 2 <=  int(time.time())):
            #print ("STATE = ABORTED")
            self.startTime = int(time.time())


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
        if(self.cmd == "abort"):
            self.curState.Exit()
            self.ToTransition("toAborted")
            self.trans.Execute();
            self.SetState(self.trans.toState)
            self.curState.Enter()
            self.trans = None
        if(self.trans):
            self.curState.Exit()
            self.trans.Execute()
            self.SetState(self.trans.toState)
            self.curState.Enter()
            self.trans = None
        self.curState.Execute()

#------------------------------------------------------------------#
# PACKML

#Char = type("Char", (object), {})

class PACKML(object):
    def __init__(self):
        self.FSM = FSM(self)

        #STATES
        self.FSM.AddState("STOPPED", Stopped(self.FSM))
        self.FSM.AddState("IDLE", Idle(self.FSM))
        self.FSM.AddState("COMPLETE", Complete(self.FSM))
        self.FSM.AddState("EXECUTE", Execute(self.FSM))
        self.FSM.AddState("ABORTED", Aborted(self.FSM))

        #TRANSITIONS
        self.FSM.AddTransition("toStopped", Transition("STOPPED"))
        self.FSM.AddTransition("toIdle", Transition("IDLE"))
        self.FSM.AddTransition("toComplete", Transition("COMPLETE"))
        self.FSM.AddTransition("toExecute", Transition("EXECUTE"))
        self.FSM.AddTransition ("toAborted", Transition("ABORTED"))

        self.FSM.SetState("IDLE")

    def Execute(self, cmd):
        self.FSM.Execute(cmd)

    def GetState(self):
        states = ["STOPPED", "IDLE", "COMPLETE", "EXECUTE", "ABORTED"]
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
from getch import getch, pause

cmd = ''

def getch_func():
    global cmd
    cmd = raw_input("> ")#getch()
    print('You pressed:', cmd)


if __name__ == '__main__':

    rospy.init_node('packml', anonymous=True)
    pub_state = rospy.Publisher('packml_state', String, queue_size=1)


    pml = PACKML()

    startT = int(time.time())
    inputThread = threading.Thread()


    while not rospy.is_shutdown():
        # Starting server if not alive
        if(not inputThread.isAlive()):
            inputThread = threading.Thread(target=getch_func, args=())
            inputThread.daemon = True
            inputThread.start()
        # Exit program it q is pressed
        if(cmd == 'quit'): break;

        currentCmd = cmd
        pml.Execute(cmd)

        if((startT + 1 <=  int(time.time())) and (pml.GetState() != None )):
            pub_state.publish(pml.GetState())
            startT =  int(time.time())

        if(currentCmd == cmd):
            cmd = ''
