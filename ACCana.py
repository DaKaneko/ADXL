import math
import cmath
import numpy as np
import matplotlib.pyplot as plt
from scipy import fftpack

bins = 2048
dt = 0.002
tlast = dt*bins
"""
tlast = 5.
dt = tlast/bins
freq0 = 3.2
ampl0 = 20.
freq1 = 0.2
ampl1 = 0.
ampln = 100
"""

y = np.zeros(bins)

with open("txyz_2048pt_2ms_1.txt") as file:
   for l in file.readlines():
      w = l.split(",")
      #print(w)
      i = int(w[0])
      #print(i,float(w[2]),i*20.*2.0)
      y[i] = float(w[2])
      # + 10. * np.sin(i*0.3)
      
#print(y)

#time, unit[s]
t = np.linspace(0.,tlast,bins)
#data, unit[A]or[ADC count]
#y = np.sin(t*freq0*np.pi*2.0) * ampl0 #+ np.sin(t*freq1*np.pi*2.0) * ampl1 + np.random.randn(bins) * ampln
#y = np.sign(np.sin(t*freq0*np.pi*2.0)) * ampl0 + np.sin(t*freq1*np.pi*2.0) * ampl1 + np.random.randn(bins) * ampln

#spectrum (discrete value)
f = fftpack.fft(y)

#frequency unit[Hz]
k = fftpack.fftfreq(bins, dt)
dk = 1. / bins / dt
#k = np.linspace(0,1.0/tlast/2,bins)

psd = abs(f) / np.sqrt(dk) / bins * np.sqrt(2)
psd[0]=0. # remove DC

#just sum square of f
#s2 = np.sqrt(sum(p[:bins//2-1]**2))
#integrate power spectrum considering unit of x axis
integ1 = np.sqrt(sum(abs(f[:bins//2-1]/bins*np.sqrt(2))**2)*2)
integ2 = np.sqrt(sum(psd[:bins//2-1]**2 * dk)) * np.sqrt(2)

print(integ1, integ2)

p = abs(f)
q = np.angle(f)

fig = plt.figure(figsize=(9,6))
#original data
ax1 = plt.subplot(211)
ax1.scatter(t, y)
#FFT amplitude
ax2 = plt.subplot(212)
ax2.scatter(k, psd)
plt.xlim(0.0, max(k))
plt.yscale("log")
plt.ylim(1e-7, 1e-2)
plt.show()

