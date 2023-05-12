#!/usr/bin/env python
import numpy as np


last10_rolls =  [1]*10
last10_pitchs =  [1]*10
max_tilt_angle = int(30)



def compute_limited_angle(theta_temp):
#function to check if angle is not bigger than allowed
    if(int(theta_temp) > max_tilt_angle):
        return max_tilt_angle
    elif(theta_temp < - max_tilt_angle):
        return -max_tilt_angle
    else :
        return theta_temp

def rad_to_deg(rad_angle):
#function to convert RAD to DEG 
    return (float(rad_angle)*180.0)/np.pi

def compute_roll(acc):
#ROLL angle - angle around X axis
    theta = float(np.arctan(acc[1]/(np.sqrt(pow(acc[0],2)+pow(acc[2],2)))))
    #print("theta",theta)
    theta_temp = rad_to_deg(theta)
    #print("theta_temp",theta_temp)
    #print("limited_angle_theta",compute_limited_angle(theta_temp))
    return compute_limited_angle(theta_temp)

def compute_pitch(acc):
#PITCH angle - angle around Y axis
    theta = float(np.arctan(acc[0]/(np.sqrt(pow(acc[1],2)+pow(acc[2],2)))))
    #print("theta",theta)
    theta_temp = (rad_to_deg(theta))
    #print("theta_temp",theta_temp)
    #print("compute_limited_angle",compute_limited_angle(theta_temp))
    return compute_limited_angle(theta_temp)

def  set_last10_rolls_pitchs(last10_rolls_temp,last10_pitchs_temp):
#function called from lsm6dsl.py where is initial fulfillment of arrays when initializing
    for i in range(10):
        last10_rolls[i] = last10_rolls_temp[i]
        last10_pitchs[i] = last10_pitchs_temp[i]
    #print("last10_pitchs",last10_pitchs)
    #print("last10_rolls",last10_rolls)

def compute_filtered_roll(acc):
    #function to compute filtered ROLL angle

    #adding new roll item to the 10th position 
    for i in range(9):
        last10_rolls[i] = last10_rolls[i+1]
    last10_rolls[9] = compute_roll(acc)
    #weighted sum - last element (added 2lines above) has weight 10,
    #where 10th element from the past has weight 1 - this forms filter with total weight of 55

    sum_roll = 0

    for i in range(10):
        sum_roll += last10_rolls[i-1] * i
    
    return (sum_roll/55)


def compute_filtered_pitch(acc):
    #function to compute filtered ROLL angle

    #adding new roll item to the 10th position
    for  i in range(9):
        last10_pitchs[i] = last10_pitchs[i+1]
    last10_pitchs[9] = compute_pitch(acc)

    #weighted sum - last element (added 2lines above) has weight 10
    # where 10th element from the past has weight 1 - this forms filter with total weight of 55
    sum_pitch = 0
    for i in range(11):
        sum_pitch += last10_pitchs[i-1] *i
    
    return (sum_pitch/55)