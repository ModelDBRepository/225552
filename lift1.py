#!/usr/bin/env python
# dynamixel.py

# ---------------------------------------------
# FSA code copied


#-- This is the code to create and link an FSA.
#-- variable at the top.  
#-- The main at the bottom tests a three state FSA (0 and 1 ignite 2
#-- which turns them off.

simulator = "nest"

DELAY = 1.0

#headers
from nest import *
import nest.raster_plot
import numpy as np
import pylab


#FSA Parmaeters
CA_SIZE = 5  #This will (almost certainly) not work with a different sized CA.
if simulator == "nest":
    INPUT_WEIGHT = 36.0
    INTRA_CA_WEIGHT = 8.0
    HALF_ON_WEIGHT = 2.0 
    CA_STOPS_CA_WEIGHT = -20.0 
    CELL_TYPE = "iaf_cond_exp"
    CELL_PARAMS = {'V_reset': -70.0, 't_ref':2.0, 'tau_syn_ex':5.0,'g_L':40.0}


#Function  to isolate differences between different list synapse constructors
def nealProjection(preNeurons,postNeurons, connectorList,inhExc):
    if simulator == "nest":
        #print 'nest', connectorList
        for connection in connectorList:
            Connect(preNeurons[connection[0]],postNeurons[connection[1]],
                    {'weight':connection[2]})
    
    else: print "bad simulator"

#--------Finite State Automata Functions ------------
#-- Function to ignite a state from a spike source
#-- Uses INPUT_WEIGHT
def turnOnState(spikeSource, start, neurons):
    connector = []
    for toOffset in range (0,CA_SIZE):
        toNeuron = toOffset + (start*CA_SIZE)
        connector = connector + [(0,toNeuron,INPUT_WEIGHT,DELAY)]
    nealProjection(spikeSource, neurons, connector,'excitatory')

#---Create a CA that will persistently fire.
#-- Assumes neurons in the same population
#-- Uses INTRA_CA_WEIGHT
def makeCA(start, neurons):
    #print 'makeCA'
    connector = []
    for fromOffset in range (0,CA_SIZE):
        fromNeuron = fromOffset + start
        for toOffset in range (0,CA_SIZE):
            toNeuron = toOffset + start
            if (toNeuron != fromNeuron):
                connector = connector + [(fromNeuron,toNeuron,INTRA_CA_WEIGHT,DELAY)]

    nealProjection(neurons,neurons,connector,'excitatory')

#-- Two states are needed to turn on a third.  
#-- This connects one of the inputs to the the third.
#-- Uses HALF_ON_WEIGHT
def stateHalfTurnsOnState(start, finish, neurons):
    connector = []
    #uses HALF_ON_WEIGHT
    for fromOffset in range (0,CA_SIZE):
        fromNeuron = fromOffset + (start*CA_SIZE)
        for toOffset in range (0,CA_SIZE):
            toNeuron = toOffset + (finish*CA_SIZE)
            if (toNeuron != fromNeuron):
                connector=connector+[(fromNeuron,toNeuron,HALF_ON_WEIGHT,DELAY)]

    nealProjection(neurons,neurons,connector,'excitatory')

#-- One State or other CA turns off another
#-- Uses CA_STOPS_CA_WEIGHT
def stateTurnsOffState(start, finish, neurons):
    connector = []
    for fromOffset in range (0,CA_SIZE):
        fromNeuron = fromOffset + (start*CA_SIZE)
        for toOffset in range (0,CA_SIZE):
            toNeuron = toOffset + (finish*CA_SIZE)
            if (toNeuron != fromNeuron):
                connector = connector + [(fromNeuron,toNeuron,CA_STOPS_CA_WEIGHT,DELAY)]

    nealProjection(neurons,neurons,connector,'inhibitory')

#---Below here are functions to test the system---
RUN_DURATION = 200
#initialize the simulator.
def init():
    if simulator == "nest":
        ResetKernel()
        SetKernelStatus({'resolution':DELAY})

    
def createTwoInputs():
    inputSpikeTimes0 = [10.0]
    inputSpikeTimes1 = [50.0]
    if simulator == "nest":
        spikeGen0 = Create('spike_generator',
            #params = {'spike_times': np.array([10.0,15.0, 20.0,25.0,30.0])})
            params = {'spike_times': np.array(inputSpikeTimes0)})

        spikeGen1 = Create('spike_generator',
            params = {'spike_times': np.array(inputSpikeTimes1)})

        spikeDet = Create('spike_detector')

    
    else: print "bad simulator for spike generator"

    return [spikeGen0,spikeGen1]

def createSixInputs():
    inputSpikeTimes0 = [10.0]
    inputSpikeTimes1 = [50.0]

    if simulator == "nest":
        spikeGen0 = Create('spike_generator',
            #params = {'spike_times': np.array([10.0,15.0, 20.0,25.0,30.0])})
            params = {'spike_times': np.array(inputSpikeTimes0)})

        spikeGen1 = Create('spike_generator',
            params = {'spike_times': np.array(inputSpikeTimes1)})

        
        spikeDet = Create('spike_detector')

    
    else: print "bad simulator for spike generator"

    return [spikeGen0,spikeGen1,spikeGen2,spikeGen3,spikeGen4,spikeGen5]



def createNeurons():
    if simulator == "nest":
        #in python Models() to see all the models including these
        #print GetDefaults (neuronType)

        cells = Create(CELL_TYPE,n=15,params = CELL_PARAMS)

    elif simulator == 'spinnaker':
        cells=Population(100,CELL_TYPE,CELL_PARAMS)

    return cells

def createRecorder():
    spikeDet = 0
    if simulator == "nest":
        spikeDet = Create('spike_detector')

    #with spinnaker you can just set record on the cells
    return spikeDet

def setupRecording(cells, spikeDetector):
    if simulator == "nest":
        Connect(cells,spikeDetector)

    elif simulator == 'spinnaker':
        cells.record()

def runFSA():
    if simulator == "nest":
        Simulate(RUN_DURATION)

    elif simulator == 'spinnaker':
        run(RUN_DURATION)


def printResults(spinnCells,nestRecorder):
    #print
    if simulator == "nest":
        spikes = GetStatus(nestRecorder)
        test = spikes[0]['events']['times']
        print 'spikes', test
	return spikes
    




# ----------------------------------------------







import sys
import serial
import time

import Adafruit_BBIO.ADC as ADC

ADC.setup()
value = ADC.read_raw("P9_33")


# The types of packets.
PING       = [0x01]
READ_DATA  = [0x02]
WRITE_DATA = [0x03]
REG_WRITE  = [0x04]
ACTION     = [0x05]
RESET      = [0x06]
SYNC_WRITE = [0x83]

# The various errors that might take place.
ERRORS = {64 : "Instruction",
          32 : "Overload",
          16 : "Checksum",
           8 : "Range",
           4 : "Overheating",
           2 : "AngleLimit",
           1 : "InputVoltage"}

def _Checksum(s):
  """Calculate the Dynamixel checksum (~(ID + length + ...)) & 0xFF."""
  return (~sum(s)) & 0xFF

def _VerifyID(id):
  """ 
  Just make sure the id is valid.
  """
#  print "Motor ID is %d " % id #print the motor id connected
  if not (0 <= id <= 0xFD):
    raise ValueError, "ID %d isn't legal!" % id

def _EnWire(v):
  """
  Convert an int to the on-wire (little-endian) format. Actually returns the
  list [lsbyte, msbyte]. Of course, this is only useful for the 16-bit
  quantities we need to deal with.
  """
#  print "Its position is %d " % v #print the position
  if not 0 <= v <= 1023:
    raise ValueError, "EnWiring illegal value: %d" % v
  return [v & 255, v >> 8]

def _DeWire(v):
  """
  Invert EnWire. v should be the list [lsbyte, msbyte].
  """
class Response:
  """
  A response packet. Takes care of parsing the response, and figuring what (if
  any) errors have occurred. These will appear in the errors field, which is a
  list of strings, each of which is an element of ERRORS.values().
  """
  def __init__(self, data):
    """
    Data should be the result of a complete read from the serial port, as a
    list of ints. See ServoController.Interact().
    """
    if len(data) == 0 or data[0] != 0xFF or data[1] != 0xFF:
      raise ValueError, "Bad Header! ('%s')" % str(data)

    if _Checksum(data[2:-1]) != data[-1]:
      raise ValueError, "Checksum %s should be %s" % (_Checksum(data[2:-1]), data[-1])

    self.data = data
    self.id, self.length = data[2:4]
    self.errors = []

    for k in ERRORS.keys():
      if data[4] & k != 0:
        self.errors.append(ERRORS[k])
    # Lastly, the data we actually asked for, if any.
    self.parameters = self.data[5:-1]

  def __str__(self):
    return " ".join(map(hex, self.data))

  def Verify(self):
    """
    Ensure that nothing went wrong.
    """
    if len(self.errors) != 0:
       raise ValueError, "ERRORS: %s" % " ".join(self.errors)
    return self  # Syntactic sugar; lets us do return foo.Verify().

class ServoController:
  """
  Interface to a servo. Most of the real work happens in Interact(), which
  does a complete round of send-and-recv. The rest of the functions do what it
  sounds like they do. Note that this represents an entire _collection_ of
  servos, not just a single servo: therefore, each function takes a servo ID
  as its first argument, to specify the servo that should get the command.
  """
  def __init__(self, portstring="/dev/ttyUSB0"):
    """
    Provide the name of the serial port to which the servos are connected.
    """
    self.portstring = portstring
    self.port = serial.Serial(self.portstring, 1000000, timeout=5) # Picked from a hat.

  def Close(self):
    """Close the serial port."""
    self.port.close()

  def __del__(self):
    """
    All this needs to do is shut down, which you can also do by hand using Close().
    """
    self.Close()

  def Interact(self, id, packet):
    """
    Given an (assembled) payload, add the various extra bits, and transmit to
    servo at id. Returns the status packet as a Response. id must be in the
    range [0, 0xFD].

    Note that the payload should be a list of integers, suitable for passing
    to chr(). See the user manual, page 10, for what's going on here.
    This is the low-level communication function; you probably want one of the
    other functions that does specific things.
    """
    _VerifyID(id)
    P = [id, len(packet)+1] + packet
    self.port.write("".join(map(chr, [0xFF, 0xFF] + P + [_Checksum(P)])))
    self.port.flushOutput()
    time.sleep(0.05)

    # Handle the read.
    res = []
    while self.port.inWaiting() > 0:
      res.append(self.port.read())
    return Response(map(ord, res)).Verify()

  # From here on out, you're looking at functions that really do something to
  # the servo itself. You should look at the user manual for details on what
  # all of these mean, although most are self-explanatory.
  def Reset(self, id):
    """
    Perform a reset on the servo. Note that this will reset the ID to 1, which
    could be messy if you have many servos plugged in.
    """
    _VerifyID(id)
    self.Interact(id, RESET).Verify()

  def GetPosition(self, id):
    """
    Return the current position of the servo. See the user manual, page 16,
    for what the return value means.
    """
    _VerifyID(id)
    packet = READ_DATA + [0x24] + [2]
    res = self.Interact(id, packet).Verify()
    if len(res.parameters) != 2:
      raise ValueError, "GetPosition didn't get two parameters!"
    return _DeWire(res.parameters)

  def GetPositionDegrees(self, id):
    """
    If you'd rather work in degrees, use this one. Again, see the user manual,
    page 16, for details.
    """
    return self.GetPosition(id) * (300.0 / 1023.0)

  def SetPosition(self, id, position):

    """
    Set servo id to be at position position. See the user manual, page 16, for
    how this works. This just sends the set position packet; the servo won't
    necessarily go where you told it. You can use GetPosition to figure out
    where it actually went.
    """
    _VerifyID(id)
    if not (0 <= position <= 1023):
      raise ValueError, "Invalid position!"
    packet = WRITE_DATA + [0x1e] + _EnWire(position)
    self.Interact(id, packet).Verify()

  def SetPosition(self, id, position):
    """
    Set servo id to be at position position. See the user manual, page 16, for
    how this works. This just sends the set position packet; the servo won't
    necessarily go where you told it. You can use GetPosition to figure out
    where it actually went.
    """
    _VerifyID(id)
    if not (0 <= position <= 1023):
      raise ValueError, "Invalid position!"
    packet = WRITE_DATA + [0x1e] + _EnWire(position)
    self.Interact(id, packet).Verify()

  def SetPositionDegrees(self, id, deg):
    """
    Set the position in degrees, according to the diagram in the manual on
    page 16.
    """
    if not 0 <= deg <= 300:
      raise ValueError, "%d is not a valid angle!" % deg
    self.SetPosition(id, int(1023.0/300 * deg))

  def GetComplianceMargin(self, id):
    """
    Return the compliance margins as (CW, CCW).
    """

    _VerifyID(id)
    packetcw  = READ_DATA + [0x1a] + [1]
    packetccw = READ_DATA + [0x1b] + [1]
    Q = self.Interact(id, packetcw).Verify()
    if len(Q.parameters) != 1:
      raise ValueError, "CW Compliance Margin parameter count problem!"
    temp = Q.parameters[0]
    Q = self.Interact(id, packetccw).Verify()

    if len(Q.parameters) != 1:
      raise ValueError, "CCW Compliance Margin parameter count problem!"
    return (temp, Q.parameters[0])


  def SetCWAngleLimit(self, id, limit):
    """
    Set the clockwise (smaller) angle limit, in servo units.
    """
    _VerifyID(id)

    if not 0 <= limit <= 1023:
      raise ValueError, "%d is not a valid CW angle limit!" % limit
    packet = WRITE_DATA + [0x06] + _EnWire(limit)
    self.Interact(id, packet).Verify()

  def GetCWAngleLimit(self, id):
    _VerifyID(id)
    packet = READ_DATA + [0x06] + [2]
    Q = self.Interact(id, packet).Verify()

    if len(Q.parameters) != 2:
      raise ValueError, "GetCWAngleLimit has the wrong return shape!"
    return _DeWire(Q.parameters)

  def SetCCWAngleLimit(self, id, limit):
    """
    Set the counterclockwise (larger) angle limit, in servo units.
    """
    _VerifyID(id)
    if not 0 <= limit <= 1023:
      raise ValueError, "%d is not a valid CCW angle limit!" % limit
    packet = WRITE_DATA + [0x08] + _EnWire(limit)
    self.Interact(id, packet).Verify()

  def GetCCWAngleLimit(self, id):
    _VerifyID(id)
    packet = READ_DATA + [0x08] + [2]
    Q = self.Interact(id, packet).Verify()
    if len(Q.parameters) != 2:
      raise ValueError, "GetCCWAngleLimit has the wrong return shape!"
    return _DeWire(Q.parameters)

  def SetID(self, id, nid):
    """
    Change the ID of a servo. Note that this is persistent; you may also be
    interested in Reset().
    """
    _VerifyID(id)
    if not 0 <= nid <= 253:
      raise ValueError, "%id is not a valid servo ID!" % nid
    packet = WRITE_DATA + [0x03] + [nid]
    self.Interact(id, packet).Verify()

  def GetMovingSpeed(self, id):
    """
    Get the moving speed. 0 means "unlimited".
    """
    _VerifyID(id)
    packet = READ_DATA + [0x20] + [2]
    Q = self.Interact(id, packet).Verify()
    if len(Q.parameters) != 2:
      raise ValueError, "GetMovingSpeed has the wrong return shape!"
    return _DeWire(Q.parameters)


  def SetMovingSpeed(self, id, speed):
    """
    Set the moving speed. 0 means "unlimited", so the servo will move as fast
    as it can.
    """
    _VerifyID(id)
    if not 0 <= speed <= 1023:
      raise ValueError, "%d is not a valid moving speed!" % speed
    packet = WRITE_DATA + [0x20] + _EnWire(speed)
    self.Interact(id, packet).Verify()


  def Moving(self, id):
    """
    Return True if the servo is currently moving, False otherwise.
    """
    _VerifyID(id)
    packet = READ_DATA + [0x2e] + [1]
    Q = self.Interact(id, packet).Verify()
    return Q.parameters[0] == 1


  def WaitUntilStopped(self, id):
    """
    Spinlock until the servo has stopped moving.
    """
    while self.Moving(id):
      pass

# Handy for interactive testing.

if __name__ == "__main__":

  if len(sys.argv) != 1:  # Specifying a port for interactive use
    ps = sys.argv[1]
    

  else:
    ps = "/dev/ttyUSB0"
    
  X = ServoController(ps)
  X.SetPosition(15,470)   #initialisation of the motors
  X.SetPosition(9,544)
  X.SetPosition(11,555)
  X.SetPosition(1,512)

#FSA.
spikeGenerators = createSixInputs()
firstSpikeGenerator =  spikeGenerators[0]
secondSpikeGenerator =  spikeGenerators[1]

stateCells = createNeurons()

recorder = createRecorder()
setupRecording(stateCells,recorder)

#turn on first state
turnOnState(firstSpikeGenerator,0,stateCells)


m9=544
step = 1

while step == 1:
   X.SetPosition(9, m9)
   m9  -= 3
   value = ADC.read_raw("P9_33")
   values = (value/1000)
   print "value is ", values
   if m9<=400:
       m9=544

   if values>0.2:
       print "@@@@@@@@@@@@@@@@@@@''"
       step = 2
       turnOnState(secondSpikeGenerator,1,stateCells)

 
makeCA(0, stateCells)
makeCA(1*CA_SIZE, stateCells)
makeCA(2*CA_SIZE, stateCells)
stateHalfTurnsOnState(0,2,stateCells)
stateTurnsOffState(2,0,stateCells)
#comment below out to check state 0 alone does not turn on state 2
stateHalfTurnsOnState(1,2,stateCells)
stateTurnsOffState(2,1,stateCells)

runFSA()

printResults(stateCells,recorder)

a = printResults(stateCells,recorder)
b =  list(a[0]['events']['senders'])
c =  list(a[0]['events']['times'])
print "b-----------"
print b
print "c-----------"
print c

# nest.raster_plot._make_plot(c, c, b, b, hist=True, hist_binwidth=10.0, grayscale=False, title=None, xlabel=None)
# pylab.savefig('test.png')
# pylab.show()


   
m15=470
step = 2
while step == 2:
    X.SetPosition(15, m15)  
    m15  += 8
    if m15>=600:            
        step = 3
        



m1=512
step = 3
while step == 3:
    X.SetPosition(1, m1)  
    m1  += 8
    if m1>=680:            
       step = 4
      

m15=600
step = 4
while step == 4:
    X.SetPosition(15, m15)  
    m15  -= 8
    if m15<=470:            
        step = 5
        

m9=400
step = 5
while step == 5:
    X.SetPosition(9, m9)
    m9  += 8
    if m9>=544:
        step = 6
        





