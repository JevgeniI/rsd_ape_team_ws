#!/usr/bin/env python3

import requests
import time
from collections import defaultdict

'''
@app.route('/orders/<int:order_id>/<string:ticket>', methods=['DELETE'])
@app.route('/orders/<int:order_id>', methods=['PUT'])
@app.route('/orders/<int:order_id>', methods=['GET'])
@app.route('/orders', methods=['GET'])
@app.route('/event_types', methods=['GET'])
@app.route('/log', methods=['POST'])
@app.route('/')
'''


'''
COMMENTS:
    - In order to send a DELETE request, we need the order ticket
    and the only way i managed to get the order ticket was by sending
    a PUT request first, which returns a dictionary with the ticket key
    and its value. I am pretty sure that is not the correct way, so
    we need to figure that out

    - If you ever want to reset the database to default values, just run
    this command on the command line:
    sudo mysql -u root -p rsd2018 < ABSOLUTE_PATH_TO_DATABASE.sql

    - While loop logic needs to be changed to adapt with orders that
    are taken from the start.
'''

class MesClient: 
    def __init__(self, url): 
        self.URL = url 
        self.order_ticket_dict = defaultdict(None)
        self.resetOrder()

######################### Helper Functions ###############################

    def delete_order(self, id, order_ticket):
        try: 
            resp = requests.delete(self.URL + "/" + str(id)+ "/" + order_ticket)

            if resp.status_code != 204:
                # This means something went wrong.
                raise requests.api.ApiError('DELETE /tasks/ {}'.format(resp.status_code))
                print("ERROR - SOMETHING WENT WRONG")
            
            print("Order with id: {} is now deleted!".format(id))
        except: 
            print("ERROR - COULDN'T DELETE %s/%s/%s", self.URL, str(id), order_ticket)


    def put_order(self, id):
        try:
            resp = requests.put(self.URL + "/" +  str(id))

            if resp.status_code != 200:
                # This means something went wrong.
                raise requests.api.ApiError('PUT /tasks/ {}'.format(resp.status_code))
                print("ERROR - SOMETHING WENT WRONG")
                return None

            data = resp.json()
            print("resp with id {} is now taken!".format(id))
            order_ticket = data['ticket']

            return order_ticket 
        except:
            print("ERROR - COULDN'T PUT %s/%s", self.URL, str(id))
            return None 


    def update_order_dict(self, id, order_ticket):
        # Note: This is updating globally
        self.ticket_order_dict[str(id)] = order_ticket

    def get_order_ticket(self, id):
        return self.ticket_order_dict(str(self.orderÍD))

    def get_order(self):
        self.resetOrder()
        try:
            resp = requests.get(self.URL)
            data = resp.json()
            if (len(data['orders'])):
                for order in data['orders']:
                    self.orderID = order['id']
                    self.orderStatus = order['status']

                    if self.orderStatus == "taken":
                        #delete_order(self.order_id, order_ticket_dict[str(self.order_id)])
                        pass
                    elif self.orderStatus == "undefined":
                        # if order is undefined, send a DELETE request i guess
                        pass
                    elif self.orderStatus == "ready":
                        print("Order with id {} is ready! Will attempt to PUT!".format(self.orderID))
                        self.orderTicket = self.put_order(self.orderID)
                        print("Ticket", self.orderTicket)
                        if(self.orderTicket != None):
                            ## Saving the number of yellows
                            self.nYellow = order['yellow']
                            list_y = [2] * self.nYellow                ## yellow action = 2
                            self.nRed = order['red']
                            list_r = [1] * self.nRed                   ## red action = 3
                            self.nBlue = order['blue']
                            list_b = [3] * self.nBlue                ## blue action = 4
                            self.brickIndex = 0

                            self.actionList = list_y + list_r + list_b
                            print(self.actionList)
                            ## Updating dictonary and deleting the order for testing purpose
                            print("Order with id {} has been taken!".format(self.orderID))
                            #self.update_order_dict(self.orderID, self.orderTicket)
                            self.delete_order(self.orderID, self.orderTicket)
                            return True 
            else:
                print("Waiting for new orders!")
                return False 

        except:
            print("Could not access url: ", self.URL)
            return False 

    def resetOrder(self):
        self.currentOrder = None 
        self.orderID = None 
        self.orderTicket = None 
        self.orderStatus = None 

        self.nYellow = None 
        self.nRed = None 
        self.nBlue = None 
        self.brickIndex = None 
        self.actionList = None 
    #############################################################


