#!/usr/bin/env python3
import numpy as np

last10_rolls =  [1]*10
last10_pitchs =  [1]*10

def compute_roll(acc):
    roll = float(np.arctan(acc[1]/(np.sqrt(pow(acc[0],2)+pow(acc[2],2)))))
    return roll

def compute_pitch(acc):
    pitch = float(np.arctan(acc[0]/(np.sqrt(pow(acc[1],2)+pow(acc[2],2)))))
    return pitch

def  set_last10_rolls_pitchs(last10_rolls_temp,last10_pitchs_temp):
    for i in range(10):
        last10_rolls[i] = last10_rolls_temp[i]
        last10_pitchs[i] = last10_pitchs_temp[i]

def compute_filtered_roll(acc):
    for i in range(9):
        last10_rolls[i] = last10_rolls[i+1]
    last10_rolls[9] = compute_roll(acc)
    
    sum_roll = 0

    for i in range(11):
        sum_roll += last10_rolls[i-1] * i
    
    return (sum_roll/55)


def compute_filtered_pitch(acc):
    for  i in range(9):
        last10_pitchs[i] = last10_pitchs[i+1]
    last10_pitchs[9] = compute_pitch(acc)

    sum_pitch = 0
    for i in range(11):
        sum_pitch += last10_pitchs[i-1] *i
    
    return (sum_pitch/55)