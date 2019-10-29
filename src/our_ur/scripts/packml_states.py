import time

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
        print ("TRANSITION - RESETTING")
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
        self.exeState = "getOrder"
        self.orderActive = false 

    def Enter(self):
        super(Execute, self).Enter()

    def Execute(self):

        if(self.exeState == "getOrder"): 
            ## request a new order

        elif(self.exeState == "")
        

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
        print("STATE: STOPPING")
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
        print("STATE: RESETTING")
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
        print("STATE: STARTING")
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
        print("STATE: COMPLETING")
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
        print("STATE: ABORTING")
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
        print("STATE: CLEARING")
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
        print("STATE: HOLDING")
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
        print("STATE: UNHOLDING")
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
        print("STATE: SUSPENDING")
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
        print("STATE: UNSUSPENDING")
        self.FSM.ToTransition("toExecute")
        self.totalLocalTime = int(time.time()*1000) - self.startTime 

    def Exit(self):
        super(Unsuspending, self).Exit()