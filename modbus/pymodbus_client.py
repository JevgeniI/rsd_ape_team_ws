#from pymodbus.client.sync import ModbusTcpClient
from pymodbus.client.sync import ModbusTcpClient
from custom_message import CustomModbusRequest
from custom_message import CustomModbusResponse



#client = ModbusTcpClient('127.0.0.1')
#client.write_coil(1, True)
#result = client.read_coils(1,1)
#print(result.bits[0])
#client.close()


if __name__ == "__main__":
    with ModbusTcpClient(host='localhost', port=5020) as client:
        client.register(CustomModbusResponse)
        request = CustomModbusRequest(1, unit=1)
        result = client.execute(request)
        print(result.values)
