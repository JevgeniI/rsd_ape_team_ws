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

    print("Order with id: {} is now taken!".format(id))


    data = resp.json()
    print("resp with id {} is now taken!".format(id))
    order_ticket = data['ticket']

    return order_ticket


def update_order_dict(ticket_order_dict, id, order_ticket):
    # Note: This is updating globally
    ticket_order_dict[str(id)] = order_ticket

#############################################################


####################### Some print statements #####################

# params= {asdadad}
URL = 'http://localhost:5000/orders'

resp = requests.get(url=URL)


print("------------------")
print("Response object type is: \n{}".format(type(resp)))
print()
print("Response: \n{}".format(resp))
print()
print("Response status code is: \n{}".format(resp.status_code))
print()
print("Response headers['content_type']: {}".format(resp.headers['content-type']))
print()
print("Response url is: \n{}".format(resp.url))
print()
print("Response text is: \n{}".format(resp.text))
print()
print("------------------")

if resp.status_code != 200:
    # This means something went wrong.
    raise requests.api.ApiError('GET /tasks/ {}'.format(resp.status_code))
    print("Something went wrong\n")
    



data = resp.json()
print("This is of type: {}".format(type(data)))
print()
for key, value in data.items():
    print("The following key: '{}'\nhas the following value(s) of type {}:\n{}".format(key, type(value), value))
    print()
print()


for order in data['orders']:
    print(order)
print()
print("-----------------------------------")
print()

##################################################################


##################################### Main Code ##############################


order_ticket_dict = defaultdict(None)

while True:
    URL = 'http://localhost:5000/orders'
    resp = requests.get(url=URL)
    data = resp.json()
    # If there are no orders, wait a bit
    if not len(data['orders']):
        print("Waiting for orders...")
        time.sleep(5)
    else:
        for order in data['orders']:
            print(order)
            order_id = order['id']
            n_yellow = order['yellow']
            n_red = order['red']
            n_blue = order['blue']
            order_status = order['status']

            if order_status == "taken":
                # If taken, move on to the next available order
                # because order was taken by another group
                # If all are taken, check every 2 minutes for new orders
                print("Order with id {} is taken! Will attempt to DELETE".format(order_id))
                delete_order(order_id, order_ticket_dict[str(order_id)])
                time.sleep(3)
            elif order_status == "undefined":
                # if order is undefined, send a DELETE request i guess
                pass
            elif order_status == "ready":
                print("Order with id {} is ready! Will attempt to PUT!".format(order_id))
                # Send to the robot the commands
                # Perform quality check, if success, send a DELETE request
                order_ticket = put_order(order_id)
                update_order_dict(order_ticket_dict, order_id, order_ticket)
                time.sleep(3)


