#!/usr/bin/env python
"""
Pymodbus Server With Callbacks
--------------------------------------------------------------------------

This is an example of adding callbacks to a running modbus server
when a value is written to it. In order for this to work, it needs
a device-mapping file.
"""
# --------------------------------------------------------------------------- #
# import the modbus libraries we need
# --------------------------------------------------------------------------- #
<<<<<<< Updated upstream
#from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.server.sync import StartTcpServer

#from pymodbus.server.asynchronous import StartUdpServer
#from pymodbus.server.asynchronous import StartSerialServer

=======
from pymodbus.server.sync import StartTcpServer
>>>>>>> Stashed changes
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer


# --------------------------------------------------------------------------- #
# import the python libraries we need
# --------------------------------------------------------------------------- #
from multiprocessing import Queue, Process

import threading
import time

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# --------------------------------------------------------------------------- #
# create your custom data block with callbacks
# --------------------------------------------------------------------------- #

block = ModbusSequentialDataBlock(0, [0]*100)
store = ModbusSlaveContext(di=block,co=block,hr=block,ir=block)
# di --> Discrete Inputs initializer
# co --> Coils initializer
# hr --> Holding register initializer
# ir --> Input register initalizer
context = ModbusServerContext(slaves=store, single=True)


def run_sync_server():

class CallbackDataBlock(ModbusSparseDataBlock):
    """ A datablock that stores the new value in memory
    and passes the operation to a message queue for further
    processing.
    """

    def __init__(self, address):


    def setValues(self, address, value):
        """ Sets the requested values of the datastore

        :param address: The starting address
        :param values: The new values to be set
        """
        super(CallbackDataBlock, self).setValues(address, value)
        self.queue.put((self.devices.get(address, None), value))

    # TCP Server
    StartTcpServer(context, identity=identity, address=("localhost", 5020))#,custom_functions=[CustomModbusRequest])


def set_register_value(address, value):
    store.setValues(1, address, [value])

def get_register_value(address):
    return store.getValues(1, address, 1)

if __name__ == "__main__":

    server = threading.Thread(target=run_sync_server, args=())
    server.daemon = True
    server.start()

    try:
        while True:
            print " **** main running **** "

            # HOW TO READ REGISTER VALUES FROM A ADDRESS SERVERSIDE (0x00 - 0x09)
            print "{} {}".format("Register address 0x00 = ",get_register_value(0x00)[0])
            print "{} {}".format("Register address 0x01 = ",get_register_value(0x01)[0])
            print "{} {}".format("Register address 0x02 = ",get_register_value(0x02)[0])
            time.sleep(2)

            # HOW TO SET A REGISTER VALUE FROM A ADDRESS SERVERSIE (0x00 - 0x09)
            set_register_value(0x00, 8)
            time.sleep(2)

    except KeyboardInterrupt:
        print "Exiting program ..."
