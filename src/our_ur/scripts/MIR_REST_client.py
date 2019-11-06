import requests
import time

url = 'http://mir.com/api/v2.0.0/'

Authorization = { 'Authorization' : "Basic UlNEMzpmMjJhOWM4YmIxYTBlMjBkYzM1N2QwMjlmZmY2MWU0NWEzZDQxYzhjMDkyMzI3ZWY1MGUxZGU2ZDFmYjlhYjc0" }
language = {'Accept-Language' : "en_US"}

class MIR_REST():
    def __init__(self):
        self.authorization ={ 'Authorization' : "Basic UlNEMzpmMjJhOWM4YmIxYTBlMjBkYzM1N2QwMjlmZmY2MWU0NWEzZDQxYzhjMDkyMzI3ZWY1MGUxZGU2ZDFmYjlhYjc0" }

    def status(self):
        resp = requests.get(url + 'status', headers=self.authorization)
        status_ok = False
        if resp.status_code != 200:
            print(resp.status_code)
            print(resp.text)
        else:
            status_ok = True
        return status_ok

    def get_mission(self, mission_name = "RSD_Group3"):
        resp = requests.get(url + 'missions', headers=self.authorization)
        mission = ""
        if resp.status_code != 200:
            print(resp.status_code)
        for position in resp.json():
            print(position)
            if (position['name'] == mission_name):
                mission = position
        return mission

    def add_mission_to_queue(self, mission):
        # header = {**Authorization, **language}
        # print (header)
        # exit()
        mission_message = { "mission_id": str(mission["guid"])
                            }
        # print (mission_message)
        # print (url + "mission_queue")
        # exit()
        resp = requests.post(url + "mission_queue", json=mission_message, headers=self.authorization)
        print (resp.text)
        if resp.status_code != 201:
            print("failed to add mission, error code: " +  str(resp.status_code) )
        if resp.status_code == 201:
            print ("mission added to mission queue")

    def read_register(self, register_id):
        resp = requests.get(url + 'registers/' + str(register_id), headers=self.authorization)
        register_value = 0
        if resp.status_code != 200:
            print(resp.status_code)
            print(resp.text)

        if (resp.json()['id'] == register_id):
            register_value = resp.json()['value']
        return register_value

    def write_register(self, register_id, value):
        message = { "value": value }
        resp = requests.put(url + 'registers/' + str(register_id), headers=self.authorization, json=message)
        
        if resp.status_code != 200:
            print(resp.status_code)
            print(resp.text)

        if (resp.json()['id'] == register_id):
            if value == resp.json()['value']:
                return 1
        return 0
    # return register_value

if __name__ == '__main__':
    MIR = MIR_REST()
    mission = MIR.get_mission()
    if mission:
        MIR.add_mission_to_queue(mission)
        mission_ended = 0
        # print (read_register(69))
        while mission_ended !=1:
            mission_ended = MIR.read_register(69)
            print ("waiting for robot to finish mission")
            time.sleep(1)
        if (MIR.write_register(69, 0)):
            print ("mission finished")
        else: 
            print ("can't change register value")
    else:
        print ("mission not found")
    
