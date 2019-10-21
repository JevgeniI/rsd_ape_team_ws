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
        #print "Stopping"
        super(Stopped, self).Enter()

    def Execute(self):
        if(self.startTime + self.timer <= time.time()):
            print "Stopped - Time elapsed: ", (int(time.time()) - self.startTime)
            self.FSM.ToTransition("toIdle")

# --------- IDLE -------- #
class Idle(State):
    def __init__(self, FSM):
        super(Idle, self).__init__(FSM)

    def Enter(self):
        #print "Entering Idle state"
        super(Idle, self).Enter()

    def Execute(self):
        if(self.startTime == 0):
            self.Enter()

        if(self.startTime + self.timer <= time.time()):
            self.FSM.ToTransition("toExecute")
            print "Idle - Time elapsed: ", (int(time.time()) - self.startTime)



# --------- Execute -------- #
class Execute(State):
    def __init__(self, FSM):
        super(Execute, self).__init__(FSM)

    def Enter(self):
        #print "Starting ..."
        super(Execute, self).Enter()

    def Execute(self):
        if(self.startTime + self.timer <= time.time()):
            self.FSM.ToTransition("toComplete")
            print "Execute - Time elapsed: ", (int(time.time()) - self.startTime)




# --------- Complete -------- #
class Complete(State):
    def __init__(self, FSM):
        super(Complete, self).__init__(FSM)

    def Enter(self):
        #print "Entering Complete state"
        super(Complete, self).Enter()

    def Execute(self):
        if(self.startTime + self.timer <= time.time()):
            self.FSM.ToTransition("toIdle")
            print "Complete - Time elapsed: ", (int(time.time()) - self.startTime)

# --------- Aborted -------- #
class Aborted(State):
    def __init__(self, FSM):
        super(Aborted, self).__init__(FSM)

    def Enter(self):
        print "Aborting!"
        super(Aborted, self).Enter()

    def Execute(self):
        if(self.startTime + self.timer <= time.time()):
            self.FSM.ToTransition("toStopped")
            print "Aborted - Time elapsed: ", (int(time.time()) - self.startTime)


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

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def SetState(self, stateName):
        self.prevState = self.curState
        self.curState = self.states[stateName]

    def ToTransition(self, toTrans):
        self.trans = self.transitions[toTrans]

    def Execute(self, cmd):
        if(cmd == "a"):
            self.ToTransition("toAborted")
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

#------------------------------------------------------------------#
# LISENTER CALLBACK FUNCTION
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':

    rospy.init_node('packml', anonymous=True)

    #rospy.Subscriber('name', String, callback)
    #rospy.Publisher('name', String, queue_size=1)
    #rospy.publish("hej")

    pml = PACKML()

    startT = int(time.time())
    cmd = ''

    while not rospy.is_shutdown():

        if(startT + 7 < time.time()):
            cmd = "a"
            startT = time.time()


        pml.Execute(cmd)
        cmd = ''
