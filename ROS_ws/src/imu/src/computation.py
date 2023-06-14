#!/usr/bin/env python3
import numpy as np

#define buffer sizes
buffer_size = 20

#init buffers
buffer_rolls =  [1]*buffer_size
buffer_pitchs =  [1]*buffer_size


def compute_roll(acc):
    """!
    @brief compute roll of the imu in radians based on the linear acceleration in x,y, and z direction measured in g's 

    Parameters : 
        @param acc => linear acceleration in x,y, and z direction measured in g's represented in a list [x,y,z]
    @return roll => roll of imu in radians 

    """

    roll = float(np.arctan(acc[1]/(np.sqrt(pow(acc[0],2)+pow(acc[2],2)))))
    return roll

def compute_pitch(acc):
    """!
    @brief compute the pitch of the imu in radians based on the linear acceleration in x,y, and z direction measured in g's

    Parameters : 
        @param acc => linear acceleration in x,y, and z direction measured in g's represented in a list [x,y,z]
    @return pitch => roll of imu in radians 

    """
    pitch = float(np.arctan(acc[0]/(np.sqrt(pow(acc[1],2)+pow(acc[2],2)))))
    return pitch

def  set_buffer_rolls_pitchs(buffer_rolls_temp,buffer_pitchs_temp):
    """!
    @brief write all elements of params buffer_rolls_temp and buffer_pitchs_temp respectievly to the global buffers buffer_rolls and buffer_pitchs

    Parameters : 
        @param buffer_rolls_temp => list of length buffer_size containing roll values in radians
        @param buffer_pitchs_temp => list of length buffer_size containing roll values in radians

    """
    for i in range(len(buffer_rolls_temp)):
        buffer_rolls[i] = buffer_rolls_temp[i]
        buffer_pitchs[i] = buffer_pitchs_temp[i]

def compute_filtered_roll(acc):
    """!
    @brief compute the moving average of the roll value with a window of buffer_size, by adding a newly calculated roll value to the buffer

    Parameters : 
        @param acc => linear acceleration in x,y, and z direction measured in g's represented in a list [x,y,z]
    @return => moving average of the roll values with a window of buffer_size in radians 
    """
    for i in range(len(buffer_rolls)-1):
        buffer_rolls[i] = buffer_rolls[i+1]
    buffer_rolls[len(buffer_rolls)-1] = compute_roll(acc)
    
    sum_roll = 0

    for i in range(len(buffer_rolls)+1):
        sum_roll += buffer_rolls[i-1] * i
    
    return (sum_roll/210)


def set_roll_and_pitch_buffer(acc):
    """!
    @brief compute the moving average of the pitch value with a window of buffer_size, by adding a newly calculated pitch value to the buffer

    Parameters : 
        @param acc => linear acceleration in x,y, and z direction measured in g's represented in a list [x,y,z] 
    @return => moving average of the pitch values with a window of buffer_size in radians
    """
    for  i in range(len(buffer_pitchs)-1):
        buffer_pitchs[i] = buffer_pitchs[i+1]
    buffer_pitchs[len(buffer_pitchs)-1] = compute_pitch(acc)

    sum_pitch = 0
    for i in range(len(buffer_pitchs)+1):
        sum_pitch += buffer_pitchs[i-1] *i
    
    return (sum_pitch/210)