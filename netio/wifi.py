import socket
import time

import micropython
import network
import gc

# A special constant that is assumed to be True by 3rd party static type checkers. It is False at runtime.
TYPE_CHECKING = False

if TYPE_CHECKING:
    from typing import Any, Optional, Tuple

BUFSIZE = 1024

class WifiConnectionManager:
    def __init__(self, ssid: "Optional[str]" = None, key: "Optional[str]" = None):
        self.ssid = ssid
        if not self.ssid:
            with open('ssid') as ssid_fp:
                self.ssid = ssid_fp.read()
        self.key = key
        if not self.key:
            with open('key') as key_fp:
                self.key = key_fp.read()

        self.wlan = network.WLAN(network.STA_IF)

    def __enter__(self):
        self.scan()
        self.connect()
        return self

    def __exit__(self, *args):
        self.cleanup()

    def scan(self) -> None:
        self.wlan.active(True)
        for net in self.wlan.scan():
            print(net) # TODO: Pretty-print network list

    def connect(self) -> None:
        """Connect to WLAN."""
        self.wlan.active(True)
        print(f'Will connect to SSID: {self.ssid}')
        self.wlan.connect(self.ssid, self.key)

        while not self.wlan.isconnected():
            print('Waiting for connection...')
            time.sleep(1)

        print('Connected')
        print(f'My IPv4 address: {self.wlan.ifconfig()[0]}')

    def disconnect(self) -> None:
        if self.wlan.isconnected():
            self.wlan.disconnect()
        if self.wlan.active():
            self.wlan.active(False)

    def cleanup(self) -> None:
        self.disconnect()
        self.wlan.deinit()
        del self.wlan

# Derived from https://hg.python.org/cpython/file/e57c8a90b2df/Lib/socket.py
class SocketWithContextManager(socket.socket):
    """A subclass of socket.socket adding a contextmanager."""

    def __init__(self, family=socket.AF_INET, type=socket.SOCK_STREAM, proto=0, fileno=None):
        super().__init__(family, type, proto, fileno)
        self._closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        if not self._closed:
            self.close()
    
    def close(self):
        self._closed = True
        return super().close()

def load_host_and_port_from_file() -> "Tuple[str, int]":
    with open('server_ip') as host_fp:
        host_str = host_fp.read()
    with open('server_port') as port_fp:
        port_str = port_fp.read()
    return host_str, int(port_str)

def get_address_tuple(host: "Optional[str]" = None, port: "Optional[int]" = None) -> "Tuple[int, int, int, Any, Any]":
    addr_info = socket.getaddrinfo(host, port)
    # Grab the address tuple needed to call socket.connect(addr) from the first result
    addr = addr_info[0][-1]
    del addr_info
    return addr

def read_url(host: str, port: int) -> bytearray:
    addr_tuple = get_address_tuple(host, port)
    print(f'Connecting to {addr_tuple}')

    buf = bytearray(BUFSIZE)
    try:
        with SocketWithContextManager() as sock:
            sock.connect(addr_tuple)
            del addr_tuple
            sock.send(b'30                                                             ')
            sock.settimeout(2)
            bytes_rcvd = sock.readinto(buf)
            print(f'Recevied {bytes_rcvd} bytes; disconnecting')
    finally:
        gc.collect()
    return buf

if __name__ == '__main__':
    # allow interrupts to throw errors
    micropython.alloc_emergency_exception_buf(100)

    with WifiConnectionManager() as wifi_mgr:
        host, port = load_host_and_port_from_file()
        data_rcvd = read_url(host, port)
        del host; del port
        print(data_rcvd.decode('ascii'))
