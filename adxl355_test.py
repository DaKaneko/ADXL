import RPi.GPIO as GPIO
import smbus
import time
import numpy as np
import matplotlib.pyplot as plt

ADDR = 0x53 #asel=1 high speed mode
#addr = 0x1D #asel=0 low speed mode   
BUS  = 1

class ADXL355():
   
   def __init__(self):
      self.bus = smbus.SMBus(BUS)
      self.checkID()
      self.initParam()
      self.range=2.0
      #mode = self.bus.read_byte_data(ADDR, 0x2C) & 0b11
      #   if mode==1


   def checkID(self):
      print(self.bus.read_byte_data(ADDR, 0x00)) #analog devices ID 0xAD
      print(self.bus.read_byte_data(ADDR, 0x02)) #device part ID 0xED

   def initParam(self):
      self.bus.write_byte_data(ADDR, 0x2C, 0x81) #highspeed, active low, 2g range
      self.bus.write_byte_data(ADDR, 0x2D, 0x00) #set active
      #print(self.bus.write_byte_data(ADDR, 0x28, 0x0A)) #LPF 1Hz #default 0x00:4kHz ODR 1kHz LPF
      print(self.bus.write_byte_data(ADDR, 0x28, 0x00)) 

   def getValue(self, axis): #X=0, Y=1, Z=2
      b1,b2,b3 = self.bus.read_i2c_block_data(ADDR, 0x08+axis*3, 3)
      print(bin(b1), bin(b2), bin(b3))
      tmp=(b1<<16) + (b2<<8) + b3
      print(bin(1<<23))
      print(bin(tmp))
      sign = (tmp & 1<<23)>>23
      tmp = tmp >> 4
      print(sign, bin(tmp))
      if sign == 1:
         tmp = tmp - (1<<20) 
      print(sign, bin(tmp))
      return(tmp)

   def setOffset(self, axis, val):
      pass

# main
nMes=100
trange=100000

a = ADXL355()
a.initParam()

tval = np.full(nMes, np.nan, dtype=float)
aval = np.full((3,nMes), np.nan, dtype=float)

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111)

mt0 = time.time() * 1000

i=0
# main loop
while(i<nMes):
   tval[i] = time.time() * 1000 - mt0
   aval[0][i] = a.range * a.getValue(0) / 2**19
   aval[1][i] = a.range * a.getValue(1) / 2**19
   aval[2][i] = a.range * a.getValue(2) / 2**19
   #time.sleep(0.0001)
   #ax.plot(tval,aval[2], color="blue")
   #plt.xlim([tval[i]-trange, tval[i]])
   #plt.pause(0.0001)
   i=i+1

ax.plot(tval,aval[2], color="blue")
#plt.xlim([tval[i]-trange, tval[i]])
plt.show()
