#!/usr/bin/env python3
import smbus
import numpy as np
import time as t 
import computation as comp
bus = smbus.SMBus(1)

#adresses 
LSM6DSL_DEVICE_ADDRESS_0 = 0x6A
LSM6DSL_DEVICE_ADDRESS_1 = 0x6B

LSM6DSL_WHO_AM_I_VALUE = 0x6A
LSM6DSL_WHO_AM_I_ADDRESS = 0x0F

#acc 
LSM6DSL_ADDRESS_CTRL1 = 0x10
LSM6DSL_ADDRESS_ACCX = 0x28
LSM6DSL_ADDRESS_ACCY = 0x2A
LSM6DSL_ADDRESS_ACCZ = 0x2C

#gyro 
LSM6DSL_ADDRESS_CTRL2 = 0x11
LSM6DSL_ADDRESS_GYROX = 0x22
LSM6DSL_ADDRESS_GYROY = 0x24
LSM6DSL_ADDRESS_GYROZ = 0x26




LSM6DS0_ADDRESS_TEMP_L = 0x20

data = bus.read_i2c_block_data(LSM6DSL_DEVICE_ADDRESS_1, LSM6DSL_WHO_AM_I_ADDRESS, 2)

#print("Data:", data)

adress = LSM6DSL_DEVICE_ADDRESS_1;

def lsm6dsl_read_byte(reg_addr):
    data = data = bus.read_i2c_block_data(adress, reg_addr, 1)
    #print(data[0])
    return data[0]

def lsm6dsl_write_byte(reg_addr,value):
    bus.write_i2c_block_data(adress, reg_addr, value)

def lsm6dsl_readArray(reg,length):
    return bus.read_i2c_block_data(adress, reg, length)
    #print("data readArray",data)


def lsm6ds0_get_acc():
    #data = [1]*6
   

    #get current scale and use it for final calculation
    data = lsm6dsl_readArray(LSM6DSL_ADDRESS_ACCX, 6)
    #print("data in get acc",data)
    xx = ((np.uint16(data[1]))) << 8 | np.uint8(data[0]) 
    yy = ((np.uint16(data[3]))) << 8 | np.uint8(data[2])    
    zz = ((np.uint16(data[5]))) << 8 | np.uint8(data[4])
    #print("xx",xx,"yy",yy,"zz",zz)
    x = float((xx>>4)/256.0)
    y = float((yy>>4)/256.0)
    z = float((zz>>4)/256.0)
    #print("x",x,"y",y,"z",z)
    return [x,y,z]

def lsm6sl_get_gyro():
    data = lsm6dsl_readArray(LSM6DSL_ADDRESS_GYROX,6)

    xx = ((np.uint16(data[1]))) << 8 | np.uint8(data[0]) 
    yy = ((np.uint16(data[3]))) << 8 | np.uint8(data[2])    
    zz = ((np.uint16(data[5]))) << 8 | np.uint8(data[4])
    
    #print("xx",xx,"yy",yy,"zz",zz)
    
    roll = float((xx>>4)/1000.0)
    pitch = float((yy>>4)/1000.0)
    yaw = float((zz>>4)/1000.0)
    #print("x",x,"y",y,"z",z)
    return [roll,pitch,yaw]
    

def fulfill_last10_rolls_pichs():
    last10_rolls_temp =  [1.2]*10
    last10_pitchs_temp = [1.2]*10
    #acc_local = [1.2]*3
    for i in range(10):
        acc_local = lsm6ds0_get_acc()
        #print("acc_local",acc_local)
        last10_pitchs_temp[i] = comp.compute_roll(acc_local)
        last10_rolls_temp[i] = comp.compute_pitch(acc_local)


    #print("last10_pitchs_temp",last10_pitchs_temp)
    #print("last10_rolls_temp",last10_rolls_temp)
    comp.set_last10_rolls_pitchs(last10_rolls_temp,last10_pitchs_temp) 

def lsm6dsl_init():
    status = 1
   
    #LIS3MDL_ACC_ON

    t.sleep(100/1000)

    val = (lsm6dsl_read_byte(LSM6DSL_WHO_AM_I_ADDRESS))
    #print(val)
    if(val == LSM6DSL_WHO_AM_I_VALUE):
        status = 1
    else:
        #if the device is not found on one address, try another one
        adress = LSM6DSL_DEVICE_ADDRESS_0

        val = (lsm6dsl_read_byte(LSM6DSL_WHO_AM_I_ADDRESS))
        #print(val)
        if(val == LSM6DSL_WHO_AM_I_VALUE):
            status = 1
        else:
            status = 0
    

    #acc device config 
    ctrl11 = np.uint8(lsm6dsl_read_byte(LSM6DSL_ADDRESS_CTRL1))
    ctrl11 &= ~0xFC
    ctrl11 |= 0xFC
    ctrl11_l = [int(ctrl11)]
    lsm6dsl_write_byte(LSM6DSL_ADDRESS_CTRL1,ctrl11_l)

    #gyro device config 
    ctrl12 = np.uint8(lsm6dsl_read_byte(LSM6DSL_ADDRESS_CTRL2))
    ctrl12 &= ~0xFF
    ctrl12 |= 0x70
    ctrl12_l = [int(ctrl12)]
    lsm6dsl_write_byte(LSM6DSL_ADDRESS_CTRL2,ctrl12_l)

    fulfill_last10_rolls_pichs()

    return status