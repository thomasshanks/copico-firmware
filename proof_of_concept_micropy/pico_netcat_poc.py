import gc
import machine
import network
import socket
import time


def load_ssid_and_key():
    with open('ssid') as ssid_fp:
        ssid = ssid_fp.read()
    with open('key') as key_fp:
        key = key_fp.read()
    return ssid, key

def connect(ssid, key):
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    for net in wlan.scan():
        print(net) # TODO: Pretty-print network list
    print(f'Will connect to SSID: {ssid}')
    wlan.connect(ssid, key)

    while wlan.isconnected() == False:
        print('Waiting for connection...')
        time.sleep(1)

    print('Connected')
    print(f'My IPv4 address: {wlan.ifconfig()[0]}')

def get_content():
    addr_info = socket.getaddrinfo('192.168.1.155', 777)
    print(f'Connecting to {addr_info[0][-1]}')
    addr = addr_info[0][-1] # Grab the IP address and port tuple from the first result
    del addr_info
    sock = socket.socket()
    sock.connect(addr)
    del addr
    data = ' '
    sock.send('\n')
    sock.settimeout(2)
    while data: # TODO: Term on conn drop
     try:
        data = sock.recv(128)
        print(str(data, 'utf8'), end='')
     except OSError:
        pass
    del data
    print('End of stream, closing socket')
    sock.close()
    del sock
    gc.collect() 

ssid, key = load_ssid_and_key()
connect(ssid, key)
while True:
    get_content()
