# This program is a small project form an engineering student that took interst in topics in dynamics and controll and
# Wants to give them a particular application test
# Baltazar Villasol


# DRAFT
# Define constants(m,v0,cd,mu,g,dt,vt, thita, Kp, Ki, Kd)
# Find en with eq en=vt-vn
# Define arrays(e,v,sf)

# Start loop
# Take last value in v and set vn to it
# Find MOTn with equation MotN=(mu * m*g-Cd*(vt)^2)+Kp en+ Ki sum(en) * delT +Kd(en-en-1)/delT
# Find vn with the equation Vn=Vn-1+delT(motn-mu*m*g*cos(thita)-Cd*(vn-1)^2-sin(thita)mg)/m
# Find en with eq en=vt-vn
# Append to respective arrays
# End loop
# Plot


#Imports

import math
import numpy as np
import matplotlib.pyplot as plt


#Constants/Variavles
m=50000   #Mass kg
v0=0    #Initial Velocity m/s
cd=0.5  #Drag Coefficient
mu=0.1 #Friction Coefficient
g=9.8   #Acceleration due to gravity
dt=0.1 #Time step
tt=800  #Total Time
theta=0 #Inclination angle
vt=50   #Target Velocity
kp=35    #Proportional Controller
kd=0    #Derivative Controller
ki=0.01 #Integral Controller
k=10
en=vt-v0        #Error
tn=int(tt/dt)   #Total Number Of Time Steps

#Arrays
T=np.zeros(tn+1)  #Array for all Times
v=np.zeros(tn+1)  #Array for all Velocities
v[0]=v0
Vt=np.zeros(tn+1) #Array to plot Target Veolcity\
Vt[0]=vt
e=np.zeros(tn+1)  #Array for all Errors
e[0]=en
mot=np.zeros(tn+1)#Array for all Motor forces

t=0 #Current time
n=0 #Current step

en = vt - vn

#This while loop will Calculate motn making vn-1 0 if n==0 and then find the new vn
#Then n increases by 1 and t increases by dt
#Then T v and e have their current value in the current n
while True:
    vn=v[n]
    if n==0:
        motn = (mu * m * g + cd * (vt ** 2)) + (kp * en + ki * e.sum() * dt + kd * en / dt) * k
    else:
        motn=(mu * m * g + cd * (vt  ** 2)) + (kp * en + ki * e.sum() * dt  + kd * ( en - e[n-1] )/dt ) * k
    if vn>=0:
        vn = vn + dt * (motn -  (mu * m * g * math.cos(theta) + cd * (vn ** 2)) - math.sin(theta)*m*g) / m
    else:
        vn = vn + dt * (motn +  (mu * m * g * math.cos(theta) + cd * (vn ** 2)) - math.sin(theta)*m*g) / m

    n=n+1
    t=t+dt
    mot[n]=motn
    v[n]=vn
    Vt[n]=vt
    en = vt-vn


    e[n]=en
    T[n]=t
    if n>=tn:
        break
#End with

#Find settling time
sT=0

for i in range(tn):

    if abs(e[i]) < 0.1:
        sT=T[i]
        break
    if i==tn-1:
        print("Wont settle in the given time")

overshoot=np.max(v)-vt
print("Overshoot: ")
print(overshoot)
print("Settling time")
print(sT)
plt.plot(T, e)
plt.plot(T, v)
plt.plot(T, Vt)
plt.title("Velocity Vs Time")
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.show()