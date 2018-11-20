#!/usr/bin/env python

from simpleOSC import initOSCClient, initOSCServer, setOSCHandler, sendOSCMsg, closeOSC, \
     createOSCBundle, sendOSCBundle, startOSCServer
import time

    
def myTest():
    initOSCClient("192.168.0.2", 9000) # takes args : ip, port
    initOSCServer("192.168.0.7", 9001, 0) # takes args : ip, port, mode => 0 basic, 1 threading, 2 forking
    
    setOSCHandler('/test', test)
    startOSCServer()

    print 'ready to receive and send osc messages...'

    try:
        while 1:
            sendOSCMsg("/sup", [444, 4.4, 'yomama is a baby'])
            time.sleep(1)
			
    except KeyboardInterrupt:
       print "closing all OSC connections and exiting"
       closeOSC()
  

def test(addr, tags, data, source):
    print "addr : %s" % addr
    print "typetags :%s" % tags
    print "data : %s" % data


if __name__ == '__main__': myTest()














