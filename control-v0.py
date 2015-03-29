#!/usr/bin/python
#
#
#

#----------------------------------------------------------
# imports
#
import smbus
import math

#----------------------------------------------------------
# definitions
#

# PID constants
KP = 2
KI = 0
KD = 0

# balancing constants
LEAN = -8.0         # deg lean off of 0[deg]
CUTOFF = 45         # cutoff angle to stop motors
MOTOR_MIN = 0xe0    # minimum motor speed PWM value
MOTOR_MAX = 0xff    # maximum motor speed PWM value

# I2C addresses
GY_ADDRESS  = 0x68
AVR_ADDRESS = 0x10

# MPU-6050 power management registers
PWR_MGT_1   = 0x6b
PWR_MGT_2   = 0x6c

# AVR commands
TWI_CMD_PWM_R   = 1     # PWM data for right motor follows, data:0 to 255
TWI_CMD_PWM_L   = 2     # PWM data for left motor follows, data: 0 to 255
TWI_CMD_DIR     = 3     # motor turn direction follows, data: see Port B definition for b0 to b3
TWI_CMD_RD_BATT = 4     # return battery voltage from ADC, data: 0 to 255
TWI_CMD_RD_BITS = 5     # return switch bit state, data=see Port B definition for b4 and b7 (all other bits also returned)
TWI_CMD_WR_BITS = 6     # set indicator bits, data=see Port B definitions for b5 and b6
TWI_CMD_MOTORS  = 7     # set 3 bytes: PWM right, PWM left, direction bits all at once

# motor and general command register bit assignments
#
# b7 b6 b5 b4 b3 b2 b1 b0
# |  |  |  |  |  |  |  |
# |  |  |  |  |  |  |  +--- 'o' right motor dir \ 0, 3=stop, 1=fwd, 2=rev
# |  |  |  |  |  |  +------ 'o' right motor dir /
# |  |  |  |  |  +--------- 'o' left motor dir  \ 0, 3=stop, 1=fwd, 2=rev
# |  |  |  |  +------------ 'o' left motor dir  /
# |  |  |  +--------------- 'i' go/no-go          PUP enabled, 0=no-go, 1=go
# |  |  +------------------ 'o' batt. ok/low      0=batt.ok,1=batt.low (red LED)
# |  +--------------------- 'o' run/not-run       0=not running,1=running (green LED)
# +------------------------ 'o' reserved
#

MOTOR_R_REV     = 1     # motor cmmand bits
MOTOR_R_FWD     = 2
MOTOR_R_STOP    = 0
MOTOR_L_REV     = 4
MOTOR_L_FWD     = 8
MOTOR_L_STOP    = 0

BATT_LOW_LED    = 0x20  # battery low LED indicator
GO_BIT_LED      = 0x40  # go/np-go LED indicator

#----------------------------------------------------------
# functions
#

#
# read_byte()
#   read a byte from I2C slave address 'adr'
#
def read_byte(adr, cmd):
    return bus.read_byte_data(adr, cmd)

#
# write_byte()
#   write a byte to I2C slave
#
def write_byte(adr, cmd, byte):
    bus.write_byte_data(adr, cmd, byte)
#
# read_word()
#   read word from I2C slave address 'adr'
#
def read_word(adr, cmd):
    high = bus.read_byte_data(adr, cmd)
    low = bus.read_byte_data(adr, cmd+1)
    val = (high << 8) + low
    return val

#
# read_word_2c()
#   read word from I2C slave address 'adr' and return a 2's complement
#   integer
#
def read_word_2c(adr, cmd):
    val = read_word(adr, cmd)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

#
# write_n_bytes()
#   write one or more bytes to I2C slave address 'adr'
#
def write_n_bytes(adr, cmd, vals):
    bus.write_i2c_block_data(adr, cmd, vals)

#
# is_go_bit()
#   read go/no-go bit and return status
#
def is_go_bit():
    bits = read_byte(AVR_ADDRESS, TWI_CMD_RD_BITS) & 0x10
    return bits

#
# get_batt_volts()
#   read battery voltage
#
def get_batt_volts():
    adc0 = read_byte(AVR_ADDRESS, TWI_CMD_RD_BATT) * 4
    volts = (adc0 * 3.32 / 1024) * 3.002
    return volts

#
# set_status_leds()
#   set go/no-go and battery low LED
#
def set_status_leds(batt, go):
    bits = 0
    if ( batt ):
        bits = BATT_LOW_LED                         # battery is low
    if ( go ):
        bits = bits | GO_BIT_LED                    # go bit is set
    write_byte(AVR_ADDRESS, TWI_CMD_WR_BITS, bits)  # write to AVR

#
# get_x_rotation()
#   calculate X axis rotation
#
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
#
# main()
#   main control loop processing:
#   1) read status switches and proceed only of status bit 'go' is asserted
#   2) read battery voltage
#      read gyro/accel sensor
#   3) calculate PID -> PWM/dir values
#   4) output PWM/dir value
#      output battery status
#   5) calculate loop time and delay/repeat from step 1
#
#   test 1: simple endless while loop with write_n_bytes()
#           640uSec cycle on RPi (not stable), 5uSec PWM write cycle on AVR
def main():
    global bus
    
    is_batt_low = 0
    is_go = 0
    
    Kp = KP     # setup PID constants
    Ki = KI
    Kd = KD
    STk = 0
    DTk = 0
    Tk_1 = 0
    
    bus = smbus.SMBus(1) # RPi I2C bus
    
    # setup GT-
    write_byte(GY_ADDRESS, PWR_MGT_1, 0)            # bring MPU-6050 out of sleep mode
                                                    # setup resolution/accuracy/filters?
    # endless control loop
    while True:
        try:
            go_switch = is_go_bit()
            is_go = go_switch
            #batt_volts = get_batt_volts()
        except IOError:
            pass    # do nothing
        except:
            write_n_bytes(AVR_ADDRESS, TWI_CMD_MOTORS, [0xe0, 0xe0, 0x00])  # stop the motors, and
            raise                                                           # re-raise an exception other than IO
        
        if ( is_go ):
            # run control loop
            set_status_leds(0, 1)
            
            # get angle reading
            try:
                accel_xout = read_word_2c(GY_ADDRESS, 0x3b)
                accel_yout = read_word_2c(GY_ADDRESS, 0x3d)
                accel_zout = read_word_2c(GY_ADDRESS, 0x3f)
            except IOError:
                continue    # reset the loop if IO error reading gyro
            except:
                write_n_bytes(AVR_ADDRESS, TWI_CMD_MOTORS, [0xe0, 0xe0, 0x00])  # stop the motors, and
                raise                                                           # re-raise an exception other than IO
                
            accel_xout_scaled = accel_xout / 16384.0
            accel_yout_scaled = accel_yout / 16384.0
            accel_zout_scaled = accel_zout / 16384.0
            Tk = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) + LEAN
            
            # calculate PID
            STk = STk + Tk                  # sum of error
            DTk = Tk - Tk_1                 # different of error
            Tk_1 = Tk                       # save for next cycle
            
            Uk = Kp*Tk + Ki*STk + Kd*DTk
            #print ("Uk=%d\r" % Uk)
            
            # activate motors
            dir = math.copysign(1, Uk)
            Uk = int(abs(Uk) + MOTOR_MIN)
            if ( Uk > MOTOR_MAX ):
                Uk = MOTOR_MAX
            
            if ( dir > 0 ):
                motor_dir = MOTOR_R_FWD + MOTOR_L_FWD
            else:
                motor_dir = MOTOR_R_REV + MOTOR_L_REV
            
            if ( abs(Tk) > CUTOFF ):
                motor_dir = MOTOR_R_STOP + MOTOR_R_STOP
            
            write_n_bytes(AVR_ADDRESS, TWI_CMD_MOTORS, [Uk, Uk, int(motor_dir)])
        else:
            # do nothing / wait
            set_status_leds(0, 0)
            
            # stop motors
            write_n_bytes(AVR_ADDRESS, TWI_CMD_MOTORS, [0xe0, 0xe0, 0x00])
            
            # reset variable
            STk = 0
            DTk = 0
            Tk_1 = 0

#----------------------------------------------------------
# start
#
if __name__ == '__main__':
    main()
