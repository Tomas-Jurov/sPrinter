#!/usr/bin/env python3
import numpy as np

#define buffer sizes
buffer_size = 20

#init buffers
buffer_rolls =  [1]*buffer_size
buffer_pitchs =  [1]*buffer_size

def compute_roll(acc):
    roll = float(np.arctan(acc[1]/(np.sqrt(pow(acc[0],2)+pow(acc[2],2)))))
    return roll

def compute_pitch(acc):
    pitch = float(np.arctan(acc[0]/(np.sqrt(pow(acc[1],2)+pow(acc[2],2)))))
    return pitch

def  set_buffer_rolls_pitchs(buffer_rolls_temp,buffer_pitchs_temp):
    for i in range(len(buffer_rolls_temp)):
        buffer_rolls[i] = buffer_rolls_temp[i]
        buffer_pitchs[i] = buffer_pitchs_temp[i]

def compute_filtered_roll(acc):
    for i in range(len(buffer_rolls)-1):
        buffer_rolls[i] = buffer_rolls[i+1]
    buffer_rolls[len(buffer_rolls)-1] = compute_roll(acc)
    
    sum_roll = 0

    for i in range(len(buffer_rolls)+1):
        sum_roll += buffer_rolls[i-1] * i
    
    return (sum_roll/210)


def set_roll_and_pitch_buffer(acc):
    for  i in range(len(buffer_pitchs)-1):
        buffer_pitchs[i] = buffer_pitchs[i+1]
    buffer_pitchs[len(buffer_pitchs)-1] = compute_pitch(acc)

    sum_pitch = 0
    for i in range(len(buffer_pitchs)+1):
        sum_pitch += buffer_pitchs[i-1] *i
    
    return (sum_pitch/210)