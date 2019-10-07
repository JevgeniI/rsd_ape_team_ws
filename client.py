import requests

url = 'http://localhost:5000/'


def get_order():
    resp = requests.get(url + 'orders')
    if resp.status_code != 200:
        print(resp.status)
        return -1

    for order in resp.json()['orders']:
        if(order['status'] == 'ready'):
            return order
    return -1

def show_orders():
    resp = requests.get(url + 'orders')
    if resp.status_code != 200:
        print(resp.status)
    for order in resp.json()['orders']:
        print(order)

def get_order_by_id(ID):
    resp = requests.get(url + 'orders/' + str(ID))
    if resp.status_code != 200:
        print(resp.status)
        return -1

    print(resp.json()['order'])

def take_order(ID):
    resp = requests.put(url + 'orders/' + str(ID))
    if(resp.status_code != 200):
        print(resp.json())#['message'])
        return -1
    return resp.json()['ticket']

def delete_order(ID, ticket):
    resp = requests.delete(url + 'orders/' + str(ID) + '/' + str(ticket))
    if(resp.status_code != 204):
        print(resp.json())
        return False
    return True

if __name__ == '__main__':
    show_orders()

    order = get_order()
    order_id = order['id']
    print("Order found: ")
    print(order)
    #id_order = get_order_by_id(1);
    if(order != -1):
        ticket = take_order(order_id)
        print('Got the order ticket: ' + ticket)
        if(delete_order(order_id, ticket)):
            print('Order was deleted')
