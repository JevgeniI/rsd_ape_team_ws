#!/usr/bin/env python
"""
Pymodbus Asynchronous Server Example
--------------------------------------------------------------------------

The asynchronous server is a high performance implementation using the
twisted library as its backend.  This allows it to scale to many thousands
of nodes which can be helpful for testing monitoring software.
"""
# --------------------------------------------------------------------------- #
# import the various server implementations
# --------------------------------------------------------------------------- #
#from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.server.sync import StartTcpServer

#from pymodbus.server.asynchronous import StartUdpServer
#from pymodbus.server.asynchronous import StartSerialServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import (ModbusRtuFramer,
                                  ModbusAsciiFramer,
                                  ModbusBinaryFramer)
from custom_message import CustomModbusRequest

import threading
import time

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)


block = ModbusSequentialDataBlock(0, [0]*100)
store = ModbusSlaveContext(di=block,co=block,hr=block,ir=block)
# di --> Discrete Inputs initializer
# co --> Coils initializer
# hr --> Holding register initializer
# ir --> Input register initalizer
context = ModbusServerContext(slaves=store, single=True)


def run_sync_server():

    # ----------------------------------------------------------------------- #
    # initialize the server information
    # ----------------------------------------------------------------------- #
    # If you don't set this or any fields, they are defaulted to empty strings.
    # ----------------------------------------------------------------------- #
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
    identity.ProductName = 'Pymodbus Server'
    identity.ModelName = 'Pymodbus Server'
    identity.MajorMinorRevision = '2.2.0'

    # ----------------------------------------------------------------------- #
    # run the server you want
    # ----------------------------------------------------------------------- #

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
