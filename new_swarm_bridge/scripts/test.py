#!/usr/bin/env python

from Bridge import BridgeServer
import threading

if __name__ == "__main__":
    server = BridgeServer()
    server.init(8888)

