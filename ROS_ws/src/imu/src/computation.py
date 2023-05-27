#!/usr/bin/env python3
import numpy as np

last20_rolls =  [1]*20
last20_pitchs =  [1]*20

def compute_roll(acc):
    roll = float(np.arctan(acc[1]/(np.sqrt(pow(acc[0],2)+pow(acc[2],2)))))
    return roll

def compute_pitch(acc):
    pitch = float(np.arctan(acc[0]/(np.sqrt(pow(acc[1],2)+pow(acc[2],2)))))
    return pitch

def  set_last20_rolls_pitchs(last20_rolls_temp,last20_pitchs_temp):
    for i in range(20):
        last20_rolls[i] = last20_rolls_temp[i]
        last20_pitchs[i] = last20_pitchs_temp[i]

def compute_filtered_roll(acc):
    for i in range(19):
        last20_rolls[i] = last20_rolls[i+1]
    last20_rolls[19] = compute_roll(acc)
    
    sum_roll = 0

    for i in range(21):
        sum_roll += last20_rolls[i-1] * i
    
    return (sum_roll/210)


def compute_filtered_pitch(acc):
    for  i in range(19):
        last20_pitchs[i] = last20_pitchs[i+1]
    last20_pitchs[19] = compute_pitch(acc)

    sum_pitch = 0
    for i in range(21):
        sum_pitch += last20_pitchs[i-1] *i
    
    return (sum_pitch/210)