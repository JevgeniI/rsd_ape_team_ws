#!/usr/bin/env python


The following is an example of how to use the synchronous modbus client
implementation from pymodbus.

if __name__ == "__main__":
    with ModbusTcpClient(host='localhost', port=5020) as client:

        # HOW TO WRITE
        client.write_register(0x00, 101) # Maximum 16-bit ( 0 to 65535 )
        client.write_register(0x01, 202) # Maximum 16-bit ( 0 to 65535 )
        client.write_register(0x02, 303) # Maximum 16-bit ( 0 to 65535 )

        # HOW TO READ REGISTER VALUES FROM A ADDRESS CLIENTSIDE (0x00 - 0x09)
        response = client.read_holding_registers(0x00, 1, unit=1) # Maximum 16-bit ( 0 to 65535 )
        print response.function_code
        if(response.function_code == 3):
            print "{} {}".format("Register address 0x00= ", response.registers[0])

        if(response.function_code == 131):
            print "Illigal address!"
        client.close()
