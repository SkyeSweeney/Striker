#!/usr/bin/python 
#-------------------------------------------------------------------------------
# 
#     Striker Monitor (Linux side)
#
#
#
#-------------------------------------------------------------------------------
import serial
import time
import argparse
import os
import sys


#-----------------------------------------------------------------------
# Main
#-----------------------------------------------------------------------
def main():
    furnace = Furnace()
    furnace.loop()


#-----------------------------------------------------------------------
# Class FURNACE
#-----------------------------------------------------------------------
class Furnace:


  #-----------------------------------------------------------------------
  # Constructor
  #-----------------------------------------------------------------------
  def __init__(self):

    self.ser = 0

    # Extract the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--log'   , default='striker.log')
    parser.add_argument('--port'  , default="COM5")
    args = parser.parse_args()

    # Save parameters
    self.log             = args.log.rstrip(os.sep)
    self.port            = args.port
    
    self.lfn = open(self.log, "a+")

  # End __init__



  #-----------------------------------------------------------------------
  # Loop
  #-----------------------------------------------------------------------
  def loop(self):

    now = time.time()

    # Open serial port to detector
    self.openSerialPort()

    # Seed the initial time
    then = now

    try:

      # Do for ever
      while 1:

        # Get the next line waiting for up to n seconds
        s = self.getLineFromPort(60*10)

        # Get the current time
        now = time.time()

        self.lfn.write(str(now) + ", " + s + "\n")
        self.lfn.flush()
        os.fsync(self.lfn)
        print str(now) + ", " + s 
      
      # End loop

    except KeyboardInterrupt:
      print "Control-C received!"

    # End try

  # End loop




  #----------------------------------------------------------------
  # Get a full line of text from serial port
  # Lines are terminated with a <LF>.
  #----------------------------------------------------------------
  def getLineFromPort(self, timeout=80):
    """Get a full line of text from 'fm' with a timeout.
       Lines must be terminated with a <LF>.
    """

    i = 0
    j = 0
    s = ""
    cnt = 0

    # Determine when time expires
    then = time.time() + timeout

    # Do till timeout or we find termination character
    while 1:

      # Break out of loop if time has expired
      if (time.time() > then):
        print("Timed out")
        break

      # If we have characters waiting for us
      try:
        n = self.ser.inWaiting()
      except:
        return("")
      # end try

      if (n > 0):

        # Get the next byte
        b = self.ser.read(1)

        # Ignore CR
        if (b == '\r'):
          continue
        # Endif

        # Break if the end of a line
        if (b == '\n'):
          break
        # Endif

        # Add it to string s
        s = s[:i] + b + s[i+1:]
        i = i + 1

      # If no byte waiting for us
      else:
        time.sleep(0.5)
      # Endif

    # Return string without CRLF
    return s


  #-----------------------------------------------------------------------
  # Open the serial port
  # This is overly complicated due to EMI bug. Should just open a 
  # specified file, not try to open from a list
  #-----------------------------------------------------------------------
  def openSerialPort(self):

    # Open serial port to detector
    try:
      self.ser = serial.Serial(self.port)
      self.ser.baudrate=115200
      self.ser.bytesize=8
      self.ser.parity='N'
      self.ser.stopbits=1
      self.ser.timeout=1
      self.ser.xonxoff=0
      self.ser.rtscts=0
      print "Opening Striker monitor on port: ", self.port
    except:
      print "Unable to open serial port", self.port
      sys.exit(1)
    # end try

    # Flush serial port
    self.ser.flushInput()
    self.ser.flushOutput()


  #-----------------------------------------------------------------------
  # Close the serial port
  #-----------------------------------------------------------------------
  def closeSerialPort(self):
    try:
      self.ser.close();
      print "Closed serial port ", self.ser.portstr
    except:
      print "Failed to close port ", self.ser.portstr
    # end try




#-----------------------------------------------------------------------
# Run the program
#-----------------------------------------------------------------------
if __name__ == "__main__":
  main()
