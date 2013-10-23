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

if (os.name == 'nt'):
  windows = 1
else:
  windows = 0


if (windows):
  import msvcrt 
else:
  import linuxcrt


#-----------------------------------------------------------------------
# Main
#-----------------------------------------------------------------------
def main():
    if (windows):
      pass
    else:
      linuxcrt.set_curses_term()

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


  def logText(self, txt):
    self.lfn.write(txt)
    self.lfn.write("\n")
    self.lfn.flush()
    os.fsync(self.lfn)
    print txt 

  #-----------------------------------------------------------------------
  # Loop
  #-----------------------------------------------------------------------
  def loop(self):

    i = 0
    s = ""
    j = 0
    t = ""

    # Open serial port to detector
    self.openSerialPort()

    # Do for ever
    while 1:

      # Assume we will not see anything
      zip = True

      try:

        # If we have characters waiting for us
        n = self.ser.inWaiting()
        if (n > 0):

          zip = False

          # Get the next byte
          b = self.ser.read(1)

          # Ignore CR
          if (b == '\r'): 
            continue

          # Break if the end of a line
          elif (b == '\n'):

            # Get the current time
            now = time.time()

            txt = ("%.3f, %s, %s")%(now, time.asctime(), s)
	    self.logText(txt)

            i = 0
            s = ""

          # If just a normal character
          else:

            # Add it to string s
            s = s[:i] + b + s[i+1:]
            i = i + 1

          # Endif

        # Endif

        # If we have keyboard bytes available
        if (windows):
          x = msvcrt.kbhit()
        else:
          x = linuxcrt.kbhit()

        if (x>0): 

          zip = False

          # Get the next byte
          if (windows):
            b = msvcrt.getch() 
          else:
            b = linuxcrt.getch() 

          if (windows):
            ignore = '\n'
            terminator = '\r'
          else:
            ignore = '\r'
            terminator = '\n'


          # Ignore CR
          if (b == ignore): 
            continue

          # Break if the end of a line
          elif (b == terminator):

            # Get the current time
            now = time.time()

            # If this is a command for the Arduino
            if (len(t) > 0) and (t[0] != '!'):
	      self.logText("Cmd: "+t)
              self.ser.write(t+"\r\n")

            # If this is an event log
            elif (len(t)>0) and (t[0] == '!'):
              txt = ("%.3f, %s")%(now, t)
	      self.logText(txt)

            j = 0
            t = ""

          # If just a normal character
          else:

            # Add it to string s
            t = t[:j] + b + t[j+1:]
            j = j + 1

          # Endif

        # If keyboard bytes  

        # If no activity, wait a moment
        if (zip):
          time.sleep(0.2)

      except KeyboardInterrupt:
        print "Control-C received!"
        break

      # End try

    # End loop

  # End definition  


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
