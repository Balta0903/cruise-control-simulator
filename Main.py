"""
Cruise control simulation for a vehicle using a PID-style controller.

The model includes rolling resistance, aerodynamic drag, road incline,
and motor force. The goal is to compare the vehicle velocity against a
target velocity and evaluate overshoot and settling time.
"""
# Baltazar Villasol




#Imports
import math
import numpy as np
import matplotlib.pyplot as plt


#Constants and variables
mass=50000
initial_velocity =0
drag_coefficient =0.5
friction_coefficient =0.1
g=9.8
time_step =0.1
total_time =2000
theta=math.radians(5) # Road angle in radians
target_velocity = 50

velocity_n= initial_velocity
error_n  = target_velocity  - velocity_n

kp=30    #Proportional Controller
kd=10    #Derivative Controller
ki=1     #Integral Controller
k=10     #Constant that can turn on or off all controllers and scale all of them together

step_number=int( total_time / time_step )   #Total Number Of Time Steps



#Arrays
T=np.zeros(step_number+1)  #Array for all Times
Velocity=np.zeros(step_number+1)
Velocity[0]= initial_velocity 
target_Velocity = np.zeros(step_number+1) #Array to plot Target Velocity
target_Velocity[0]=target_velocity
Error=np.zeros(step_number+1)
Error[0] = error_n 
MotorForce=np.zeros(step_number+1)
t=0 #Current time



## This loop calculates the motor force and updates the vehicle velocity at each time step.
for n in range(step_number) :
    velocity_n=Velocity[n]
    #calculate controllers
    proportional = kp * error_n
    integral = ki * Error.sum() *  time_step

    #There is no error n-1 when n=0 so it is assumed to be 0
    if n == 0:
        derivative = kd * error_n / time_step
    else:
        derivative = kd * (error_n - Error[n - 1]) / time_step

    motor_force_n = (friction_coefficient  * mass * g +  drag_coefficient  * (target_velocity  ** 2)) + k * ( proportional + integral  + derivative )


    if velocity_n>=0: #When velocity is negative drag and frictions do work forward this if takes it into account
        velocity_n = velocity_n +  time_step  * (motor_force_n -  (friction_coefficient  * mass * g * math.cos(theta) +  drag_coefficient  * (velocity_n ** 2)) - math.sin(theta)*mass*g) /mass 
    else:
        velocity_n = velocity_n +  time_step  * (motor_force_n +  (friction_coefficient  * mass * g * math.cos(theta) +  drag_coefficient  * (velocity_n ** 2)) - math.sin(theta)*mass*g) /mass 



    #Store the calculated values at the next time step
    n_next=n+1
    t=t+ time_step 
    MotorForce[n_next]=motor_force_n
    Velocity[n_next]=velocity_n
    target_Velocity[n_next] = target_velocity
    error_n  = target_velocity -velocity_n
    Error[n_next]=error_n
    T[n_next]=t

#End For




#Checks when the error is smaller than 5% of the total velocity
for i in range(step_number):

    if abs(Error[i]) < target_velocity *0.05:
        settling_time=T[i]
        break
    if i==step_number-1:
        print("Wont settle in the given time")
        settling_time = "NA"

#Prints
overshoot=np.max(Velocity)-target_velocity 
print("Overshoot: ")
print(overshoot)
print("Settling time")
print(settling_time)


#Plots
plt.plot(T, Error,label='Error')
plt.plot(T, Velocity,label='Velocity')
plt.plot(T, target_Velocity , label='Target Velocity')
plt.title("Velocity Vs Time")
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.legend()
plt.show()
