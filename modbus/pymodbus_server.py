#!/usr/bin/env python3
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
from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
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
#log.setLevel(logging.DEBUG)

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

identity = ModbusDeviceIdentification()



def run_sync_server():
    # TCP Server
    StartTcpServer(context, identity=identity, address=("10.42.0.96", 5020))#,custom_functions=[CustomModbusRequest])


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
            print (" **** main running **** ")

            # HOW TO READ REGISTER VALUES FROM A ADDRESS SERVERSIDE (0x00 - 0x09)
            print ("Register address 0x00 = ", get_register_value(0x00)[0])
            print ("Register address 0x01 = ", get_register_value(0x01)[0])
            print ("Register address 0x02 = ", get_register_value(0x02)[0])
            time.sleep(2)

            # HOW TO SET A REGISTER VALUE FROM A ADDRESS SERVERSIE (0x00 - 0x09)
            set_register_value(0x00, 8)
            time.sleep(2)

    except KeyboardInterrupt:
        print ("Exiting program ...")
