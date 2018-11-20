"""
This program provides a basic OSC API for an ST Robotics ST12 five DOF robot.
It's meant to be run on a Raspberry Pi which is connected via serial/USB to
the ST12 and via OSC to the network.

Written by Danny Bazo in 2014
"""


""" dependencies """

from simpleOSC import initOSCClient, initOSCServer, setOSCHandler, sendOSCMsg, closeOSC, \
  createOSCBundle, sendOSCBundle, startOSCServer
import serial
import time
import math


""" global variables """

haveClient = False
port = 0
messageToRobot = 0
sentAddr = 0
sentData = 0
busy = False
inJointMode = False



""" OSC callbacks / robot commands """

def client(addr, tags, data, source):
  """ sets address of client computer controlling the robot """
  global haveClient
  print "changing client to " + source[0]
  initOSCClient(source[0], 9000)
  sendOSCMsg(addr, ["NEW CLIENT ADDRESS: ", source[0]])
  haveClient = True
  
def initialize(addr, tags, data, source):
  """ starts robot systems, must be called after setting client but before
  doing anything else """
  handleOSCCmd(addr, data)

def calibrate(addr, tags, data, source):
  """ initiates the robot's joint encoder calibration procedure. This should be
  called only after calling initialize, which de-energizes the robot joints to
  allow for manual positioning """
  handleOSCCmd(addr, data)

def toollength(addr, tags, data, source):
  """ sets the distance from the wrist/hand junction to the desired position from
  which to calculate /xyz and /where """
  handleOSCCmd(addr, data)

def speed(addr, tags, data, source):
  """ sets the maximum speed at which any joint will move. movements accelerate
  up to this speed, if attainable. """
  handleOSCCmd(addr, data)

def where(addr, tags, data, source):
  """ queries the robot for its position. depending on which mode (joint or
  cartesian) is currently active, the response varies accordingly """
  handleOSCCmd(addr, data)

def cartesian(addr, tags, data, source):
  """ puts robot into cartesian mode, expects xyz commands in this state """
  handleOSCCmd(addr, data)

def joint(addr, tags, data, source):
  """ puts robot into joint mode, expects individual commands to joints in this
  state """
  handleOSCCmd(addr, data)

def home(addr, tags, data, source):
  """ moves the robot to the home position """
  handleOSCCmd(addr, data)

def move(addr, tags, data, source):
  """ moves the robot. the first argument is the move type, from {waist, shoulder,
  elbow, wrist, hand, xyz}. all moves except xyz target a single joint and must
  have one argument after the move type which is a float in radians. xyz moves
  must have three arguments which are the desired cartesian coords. """
  handleOSCCmd(addr, data)

def off(addr, tags, data, source):
  """ de-energizes the robot in preparation for turning off """
  handleOSCCmd(addr, data)



""" helper functions """

def handleOSCCmd(addr, data):
  """ if command is well-formatted and robot is not busy, set the command
  to send to the robot """

  # check for bad command
  if not formatOK(addr, data):
    sendOSCMsg(addr, [data, "BAD_COMMAND"])
    return

  # check for busy robot
  global busy
  if busy:
    sendOSCMsg(addr, [data, "BUSY"])
    return

  # confirm receipt of command to client and prep command to send to robot
  setCommand(addr, data) # sets busy flag to True
  sendOSCMsg(addr, [data, "GOT"]);


def formatOK(cmd, val):
  if cmd == "/initialize":
    return True
  if cmd == "/calibrate":
    return True
  if cmd == "/toollength":
    return val[0] >= 0.0 and val[0] <= 1000.0
  if cmd == "/speed":
    return val[0] > 0.0 and val[0] <= 1.0
  if cmd == "/home":
    return True
  if cmd == "/where":
    return True
  if cmd == "/cartesian":
    return True
  if cmd == "/joint":
    return True
  if cmd == "/off":
    return True

  if cmd == "/move":
    if inJointMode:
      # check for correct number of arguments
      if len(val) != 2:
        return False
      # check for correct joint name
      if val[0] not in ['waist', 'shoulder', 'elbow', 'wrist', 'hand']:
        return False
      # check for proper range of arguments
      if val[1] < -1.57 or val[1] > 1.57:
        return False
      return True

    else: # in cartesian mode
      # check for correct number of arguments
      if len(val) != 4:
        return False
      # check for correct move type
      if val[0] != 'xyz':
        return False
      # z < 0 disallowed
      if val[3] < 0:
        return False
      # directly backwards disallowed
      if val[1] > -1000 and val[1] < 1000 and val[2] < 0:
        return False
      return True
  

def setCommand(cmd, val):
  global busy, messageToRobot, sentAddr, sentData, inJointMode
  busy = True
  sentAddr = cmd
  sentData = val
  
  if cmd == "/initialize":
    messageToRobot = "ROBOFORTH"
  elif cmd == "/calibrate":
    messageToRobot = "CALIBRATE"
  elif cmd == "/toollength":
    messageToRobot = "%3.1f TOOL-LENGTH !" % val[0]
  elif cmd == "/speed":
    speed = round(scale(val[0], 0.0, 1.0, 0.0, 5000.0))
    messageToRobot = "%d TRACKSPEED !" % speed 
  elif cmd == "/home":
    messageToRobot = "HOME"
  elif cmd == "/where":
    messageToRobot = "WHERE"
  elif cmd == "/cartesian":
    inJointMode = False
    messageToRobot = "CARTESIAN"
  elif cmd == "/joint":
    inJointMode = True
    messageToRobot = "JOINT"
  
  elif cmd == "/move":
    # handle cartesian move command
    if not inJointMode:
      messageToRobot = "%d %d %d MOVETO" % (val[1], val[2], val[3])
  
    # handle joint move command
    else:
      # constrain to between (-pi/2, pi/2)
      radianVal = min(val[1], 1.57)
      radianVal = max(-1.57, radianVal)
      
      # convert from radians to motor steps
      jointName = "WAIST"
      motorSteps = 0
      if val[0] == "waist":
        jointName = "WAIST"
        motorSteps = round(scale(radianVal, -1.57, 1.57, -3640, 3640))
      elif val[0] == "shoulder":
        jointName = "SHOULDER"
        motorSteps = round(scale(radianVal, -1.57, 1.57, -8400, 8400))     
      elif val[0] == "elbow":
        jointName = "ELBOW"
        motorSteps = round(scale(radianVal, -1.57, 1.57, -6000, 6000))     
      elif val[0] == "hand":
        jointName = "HAND"
        motorSteps = round(scale(radianVal, -1.57, 1.57, -4000, 4000))     
      elif val[0] == "wrist":
        jointName = "WRIST"
        motorSteps = round(scale(radianVal, -1.57, 1.57, -4500, 4500))
      messageToRobot = "TELL %s %d MOVETO" % (jointName, motorSteps)
  
  elif cmd == "/off":
    messageToRobot = "DE-ENERGIZE"


def initializeRobot():
  global port, sentAddr
  port.write("ROBOFORTH\r")
  if getReply() != True:
    sendOSCMsg(sentAddr, ["ROBOFORTH ERROR"])
  else:
    port.write("START\r")
    if getReply() != True:
      sendOSCMsg(sentAddr, ["START ERROR"])
    else:
      port.write("DE-ENERGIZE\r")
      if getReply() != True:
        sendOSCMsg(sentAddr, ["DE-ENERGIZE ERROR"])
      else:
        sendOSCMsg(sentAddr, ["INITIALIZE DONE, PUT ROBOT IN HOME POSITION AND CALIBRATE"])


def calibrateRobot():
  global port, sentAddr, inJointMode
  port.write("CALIBRATE\r")
  if getReply() != True:
    sendOSCMsg(sentAddr, ["CALIBRATE ERROR"])
  else:
    port.write("CARTESIAN\r")
    if getReply() != True:
      sendOSCMsg(sentAddr, ["CARTESIAN ERROR"])
    else:
      inJointMode = False
      port.write("2500 TRACKSPEED !\r")
      if getReply() != True:
        sendOSCMsg(sentAddr, ["SPEED ERROR"])
      else:
        sendOSCMsg(sentAddr, ["DONE"])


def sendCommand():
  """ (blocks!) sends pending command message to the robot and waits
  for the robot to finish before returning. busy flag must have been
  previously set in setCommand(). """
  global port, messageToRobot 
  print "sending: " + messageToRobot  
  port.write(str(messageToRobot) + "\r")


def getReply():
  global port
  rcv = ""
  wasCmdCompleted = False
  while True:
    if port.inWaiting() > 0:
      rcv = rcv + port.read()
    if rcv[-4:-2] == "OK":
      wasCmdCompleted = True
      break
    elif rcv[-9:-2] == "ABORTED":
      wasCmdCompleted = False
      break
  print "got: " + rcv

  # handle /where query
  if sentAddr == "/where":
    global inJointMode
    splitted = rcv.split("\r")
    posLine = splitted[2]
    posStrings = posLine.split()

    # handle joint mode response
    if inJointMode:
      print "posStrings: ", posStrings
      pos = map(lambda x: float(x), posStrings)
      print "pos: ", pos 
      # convert to radians and send joint positions via OSC
      pos[0] = scale(pos[0], -3640, 3640, -1.57, 1.57)
      pos[1] = scale(pos[1], -8400, 8400, -1.57, 1.57)
      pos[2] = scale(pos[2], -6000, 6000, -1.57, 1.57)
      pos[3] = scale(pos[3], -4000, 4000, -1.57, 1.57)
      pos[4] = scale(pos[4], -4500, 4500, -1.57, 1.57)
      sendOSCMsg(sentAddr, [pos[0], pos[1], pos[2], pos[3], pos[4]])      

    # handle cartesian mode response
    else:
      pos = map(lambda x: float(x), posStrings)
      print "pos: ", pos
      # send cartesian position via OSC
      sendOSCMsg(sentAddr, [pos[0], pos[1], pos[2]])

  return wasCmdCompleted


def scale(v, inLo, inHi, outLo, outHi):
  return (v - inLo) * (outHi - outLo) / (inHi - inLo) + outLo 

  

""" main function definitions """

def setup():
  """ initializes everything """

  # start serial comm
  global port
  port = serial.Serial("/dev/ttyUSB0", baudrate=19200, timeout=3.0)

  # start OSC comm and attach handlers
  global haveClient
  haveClient = False
  initOSCServer("192.168.0.160", 9001, 0) # last arg: 0 basic, 1 threading, 2 forking
  setOSCHandler("/client", client)
  setOSCHandler("/initialize", initialize)
  setOSCHandler("/calibrate", calibrate)
  setOSCHandler("/toollength", toollength)
  setOSCHandler("/speed", speed) 
  setOSCHandler("/home", home)
  setOSCHandler("/where", where)
  setOSCHandler("/cartesian", cartesian)
  setOSCHandler("/joint", joint)
  setOSCHandler("/move", move)
  setOSCHandler("/off", off)

  startOSCServer() 
  
  global busy
  busy = False

  print "roboforthOSC started..."


def loop():
  """ sends pending commands to robot, listens for reply, relays reply to OSC """
  global haveClient, messageToRobot, busy, sentAddr, sentData

  try:
    while True:
      if haveClient and messageToRobot != 0:
        if sentAddr == "/initialize":
          initializeRobot() # blocks
        elif sentAddr == "/calibrate":
          calibrateRobot() # blocks
        else:
          sendCommand()
          if getReply() == True: # blocks
            sendOSCMsg(sentAddr, [sentData, "DONE"])
          else:
            sendOSCMsg(sentAddr, [sentData, "ABORTED"])
        busy = False
        messageToRobot = 0
        sentAddr = 0
        sentData = 0

  except KeyboardInterrupt:
    print "closing all connections and exiting..."
    closeOSC()



""" main """

setup()
loop()
