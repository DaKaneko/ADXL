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
      mode = self.bus.read_byte_data(ADDR, 0x2C) & 0b11
      if mode==1:
         self.range=2.0
      elif mode==2:
         self.range=4.0
      elif mode==3:
         self.range=8.0

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
      #print(bin(b1), bin(b2), bin(b3))
      tmp=(b1<<12) + (b2<<4) + (b3>>4)
      #print(bin(1<<23))
      #print(bin(tmp))
      return(tmp)

   def setOffset(self, axis, val):
      pass

# main
nMes=2048
trange=100000

a = ADXL355()
#a.initParam()

tval = np.full(nMes, np.nan, dtype=float)
mval = np.full(nMes, np.nan, dtype=float)
bval = np.full((3,nMes), np.nan, dtype=int)
xval = np.full(nMes, np.nan, dtype=float)
yval = np.full((3,nMes), np.nan, dtype=float)

mt0 = time.time()
deltat = 0.01
i=0
# main loop
while(i<nMes):
   tval[i] = time.time()# * 1000 - mt0
   if tval[i] > i*deltat + mt0:
      #aval[0][i] = 1.
      bval[0][i] = a.getValue(0)
      bval[1][i] = a.getValue(1)
      bval[2][i] = a.getValue(2)
   #aval[2][i] = a.range * a.getValue(2) / 2**19
   #time.sleep(0.0001)
   #ax.plot(tval,aval[2], color="blue")
   #plt.xlim([tval[i]-trange, tval[i]])
   #plt.pause(0.0001)
      i=i+1


Dataname="/home/pi/ADXL355/txyz_2048pt_10ms_1.txt"
file0 = open(Dataname,"a")
xval[0]=0
mval[0]=(tval[0]-mt0)* 1000
for j in range(1, nMes):
   mval[j] = (tval[j]- mt0)*1000
   xval[j] = mval[j] - mval[j-1]
   for x in [0, 1, 2]:
      tmp = bval[x][j]
      sign = tmp>>19
      tmp = tmp - (sign<<20)   
      yval[x][j] = tmp * a.range / 2**19
   print("%d, %.3f, %.7f, %.7f, %.7f" %(j,mval[j],yval[0][j],yval[1][j],yval[2][j]), file=file0)

fig1 = plt.figure(figsize=(8, 6))
ax1 = fig1.add_subplot(111)
ax1.scatter(mval,xval, color="black")
fig2 = plt.figure(figsize=(8, 6))
ax2 = fig2.add_subplot(111)
ax2.scatter(mval,yval[0], color="red")
ax2.scatter(mval,yval[1], color="green")
#ax2.scatter(mval,yval[2], color="blue")

#plt.xlim([tval[i]-trange, tval[i]])
plt.show()
