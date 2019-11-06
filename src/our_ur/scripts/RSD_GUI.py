import sys
import time
import threading
import os

from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import *
from PySide2.QtUiTools import QUiLoader

IndexToFrame = {
    1: "frameAborted",
    2: "frameAborting",
    3: "frameClearing",
    4: "frameComplete",
    5: "frameCompleting",
    6: "frameExecuting",
    7: "frameHeld",
    8: "frameHolding",
    9: "frameIdle",
    10: "frameReseting",
    11: "frameStarting",
    12: "frameStopped",
    13: "frameStopping",
    14: "frameSuspended",
    15: "frameSuspending",
    16: "frameUnholding",
    17: "frameUnsuspending"
}

class Packml_GUI(QThread):
    StopButtonPressed = Signal() 
    StartButtonPressed = Signal() 
    PauseButtonPressed = Signal() 
    ResetButtonPressed = Signal() 

    @Slot()
    def StopButtonPushed(self):
        #print ( "StopButtonPressed")
        self.StopButtonPressed.emit()
    
    @Slot()
    def StartButtonPushed(self):
        #print ( "StartButtonPressed")
        self.StartButtonPressed.emit()
    
    @Slot()
    def PauseButtonPushed(self):
        #print ( "PauseButtonPressed")
        self.PauseButtonPressed.emit()

    @Slot()
    def ResetButtonPushed(self):
        #print ( "ResetButtonPressed")
        self.ResetButtonPressed.emit()

    @Slot(int)
    def OEE_value(self, value):
        self.main_window.labelOEEValue.setText(str(value))

    @Slot(int)
    def Uptime_value(self, value):
        self.main_window.labelUptimeValue.setText(str(value))

    @Slot(str)
    def Error_message(self, message):
        if message != "No errors":
            self.BoolErrorMessage = True
        else:
            self.BoolErrorMessage = False
        self.main_window.labelErrormessage.setText(message)


    @Slot(str)
    def CurrentState(self, state):
        if state == "ABORTED":
            self.CurrentStateIndex = 1
        elif state == "ABORTING":
            self.CurrentStateIndex = 2
        elif state == "CLEARING":
            self.CurrentStateIndex = 3
        elif state == "COMPLETE":
            self.CurrentStateIndex = 4
        elif state == "COMPLETING":
            self.CurrentStateIndex = 5
        elif state == "EXECUTE":
            self.CurrentStateIndex = 6
        elif state == "HELD":
            self.CurrentStateIndex = 7
        elif state == "HOLDING":
            self.CurrentStateIndex = 8
        elif state == "IDLE":
            self.CurrentStateIndex = 9
        elif state == "RESETTING":
            self.CurrentStateIndex = 10
        elif state == "STARTING":
            self.CurrentStateIndex = 11
        elif state == "STOPPED":
            self.CurrentStateIndex = 12
        elif state == "STOPPING":
            self.CurrentStateIndex = 13
        elif state == "SUSPENDED":
            self.CurrentStateIndex = 14
        elif state == "SUSPENDING":
            self.CurrentStateIndex = 15
        elif state == "UNHOLDING":
            self.CurrentStateIndex = 16
        elif state == "UNSUSPENDING":
            self.CurrentStateIndex = 17
        else:
            print ("WRONG COMMAND SENT TO CURRENT STATE UPDATEs")
        self.disable_all_frames()
        # self.enable_frame()

    def __init__(self, UI_file =os.path.dirname(os.path.realpath(__file__)) + "/RSD_GUI.ui" , PackML_image_file= os.path.dirname(os.path.realpath(__file__)) + "/PackML.png"):
        QThread.__init__(self)

        ui_file = QFile(UI_file)
        ui_file.open(QFile.ReadOnly)

        loader = QUiLoader()
        self.main_window = loader.load(ui_file)
        # self.main_window.
        self.main_window.labelImage.setPixmap(PackML_image_file)
        self.disable_all_frames()
        self.CurrentStateIndex = 9
        self.BoolErrorMessage = False
        self.main_window.labelOEEValue.setText("0")
        self.main_window.labelUptimeValue.setText("0")
        self.main_window.labelErrormessage.setText("No errors")
        self.main_window.pushButtonStop.clicked.connect(self.StopButtonPushed)
        self.main_window.pushButtonReset.clicked.connect(self.ResetButtonPushed)
        self.main_window.pushButtonStart.clicked.connect(self.StartButtonPushed)
        self.main_window.pushButtonPause.clicked.connect(self.PauseButtonPushed)
        ui_file.close()
        self.main_window.showMaximized()
        

    def disable_all_buttons(self):
        self.main_window.pushButtonReset.setEnabled(False)
        self.main_window.pushButtonStart.setEnabled(False)
        self.main_window.pushButtonPause.setEnabled(False)
        self.main_window.pushButtonStop.setEnabled(False)

    def disable_all_frames(self):
         for key in IndexToFrame:
            tmp = "self.main_window." + IndexToFrame[key] + ".hide()"
            eval(tmp)
            time.sleep(0.0005)

    def enable_all_frames(self):
         for key in IndexToFrame:
            tmp = "self.main_window." + IndexToFrame[key] + ".show()"
            eval(tmp)
            time.sleep(0.0005)

    def disable_frame(self, index):
        tmp = "self.main_window." + IndexToFrame[index] + ".hide()"
        eval(tmp)
        time.sleep(0.0005)

    def enable_frame(self, index):
        tmp = "self.main_window." + IndexToFrame[index] + ".show()"
        eval(tmp)
        time.sleep(0.0005)

    def __del__(self):
        self.wait()
    
    def run(self):
        while True:
            
            self.msleep(500)
            self.enable_frame(self.CurrentStateIndex)
            if self.BoolErrorMessage == True:
                self.main_window.labelErrormessage.setStyleSheet("background-color: rgb(255, 0, 0);")
            self.msleep(500)
            self.disable_frame(self.CurrentStateIndex)
            if self.main_window.labelErrormessage.styleSheet() == "background-color: rgb(255, 0, 0);" or self.BoolErrorMessage == False:
                self.main_window.labelErrormessage.setStyleSheet("")

#THIS CLASS ONLY FOR TESTING
class Communicate(QObject):                                                 
 # create a new signal on the fly and name it 'speak'                       
 speak = Signal(str)    

def test_func():
    print ("Making connection")
    someone = Communicate()                                                     
    # connect signal and slot                                                   
    someone.speak.connect(GUI.CurrentState)
    someone.speak.connect(GUI.Error_message)
    while True:
        print ("Next state: ")
        cmd = input("> ")
        # print ("Transitioning to state: {}".format(cmd))
        someone.speak.emit(cmd)

if __name__ == "__main__":
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication(sys.argv)

    GUI = Packml_GUI()
    GUI.start()
    inputThread = threading.Thread(target=test_func, args=())
    inputThread.daemon = True
    inputThread.start()
    sys.exit(app.exec_())