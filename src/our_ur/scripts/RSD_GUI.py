import sys
import time

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
        print ( "StopButtonPressed")
        self.StopButtonPressed.emit()
    
    @Slot()
    def StartButtonPushed(self):
        print ( "StartButtonPressed")
        self.StartButtonPressed.emit()
    
    @Slot()
    def PauseButtonPushed(self):
        print ( "PauseButtonPressed")
        self.PauseButtonPressed.emit()

    @Slot()
    def ResetButtonPushed(self):
        print ( "ResetButtonPressed")
        self.ResetButtonPressed.emit()

    @Slot(int)
    def OEE_value(self, value):
        self.main_window.labelOEEValue.setText(str(value))

    @Slot(int)
    def Uptime_value(self, value):
        self.main_window.labelUptimeValue.setText(str(value))

    @Slot(str)
    def Error_message(self, message):
        self.main_window.labelErrormessage.setText(message)

    def __init__(self):
        QThread.__init__(self)
        ui_file = QFile("RSD_GUI.ui")
        ui_file.open(QFile.ReadOnly)

        loader = QUiLoader()
        self.main_window = loader.load(ui_file)
        # self.main_window.
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
            self.disable_all_frames()
            self.msleep(500)
            for key in IndexToFrame:
                self.enable_frame(key)
            self.msleep(500)
    


if __name__ == "__main__":
    app = QApplication(sys.argv)

    GUI = Packml_GUI()
    GUI.start()
    sys.exit(app.exec_())