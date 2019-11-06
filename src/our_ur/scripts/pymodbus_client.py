#!/usr/bin/env python3
from pymodbus.client.sync import ModbusTcpClient
import rospy


class ModbusClient: 
    def __init__(self, IP, port): 
        self.client =  ModbusTcpClient(host=IP, port=port) 
        if(not self.Connect()):
            rospy.logerr("Could not connect to the modbus server...")
        else: 
            rospy.loginfo("Connected to host[%s] via port[%s]", IP, port)


    def Connect(self):
        return self.client.connect()

    def Close(self):
        self.client.close()

    def readRegister(self, address): 
        response = self.client.read_holding_registers(address, 1, unit=1) 
        if(response.function_code == 3):
            return response.registers[0]
        else:
            return False 

    def writeRegister(self, address, data): 
        response = self.client.write_register(address, data)   ## maximum 16 bit
        print(response)
        #if(respone.function_code == 3): 
        #    return response.registers[]

if __name__ == "__main__":
    with ModbusTcpClient(host='localhost', port=5020) as client:

        # HOW TO WRITE
        client.write_register(0x00, 101) # Maximum 16-bit ( 0 to 65535 )
        client.write_register(0x01, 202) # Maximum 16-bit ( 0 to 65535 )
        client.write_register(0x02, 303) # Maximum 16-bit ( 0 to 65535 )

        # HOW TO READ REGISTER VALUES FROM A ADDRESS CLIENTSIDE (0x00 - 0x09)
        response = client.read_holding_registers(0x00, 1, unit=1) # Maximum 16-bit ( 0 to 65535 )

        if(response.function_code == 3):
            print ("Register address 0x00= ", response.registers[0])

        if(response.function_code == 131):
            print ("Illigal address!")
        client.close()
