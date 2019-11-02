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

######################### Helper Functions ###############################

def delete_order(id, order_ticket):
    
    
    URL = "http://localhost:5000/orders/" + str(id)+ "/" + order_ticket
    print(URL)
    resp = requests.delete(url=URL)

    if resp.status_code != 204:
        # This means something went wrong.
        raise requests.api.ApiError('DELETE /tasks/ {}'.format(resp.status_code))
        print("Something went wrong\n")
    
    print("Order with id: {} is now deleted!".format(id))



def put_order(id):
    
    URL = "http://localhost:5000/orders/" + str(id)
    resp = requests.put(url=URL)

    if resp.status_code != 200:
        # This means something went wrong.
        raise requests.api.ApiError('PUT /tasks/ {}'.format(resp.status_code))
        print("Something went wrong\n")
        return None

    print("Order with id: {} is now taken!".format(id))


    data = resp.json()
    print("resp with id {} is now taken!".format(id))
    order_ticket = data['ticket']

    return order_ticket


def update_order_dict(ticket_order_dict, id, order_ticket):
    # Note: This is updating globally
    ticket_order_dict[str(id)] = order_ticket

#############################################################

order_ticket_dict = defaultdict(None)
