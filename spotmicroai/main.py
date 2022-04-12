#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Arnaud Villeneuve
"""

"""
type %matplotlib qt  on console to enable animation
"""
##import board
##import busio
##from adafruit_servokit import ServoKit
#import adafruit_ads1x15.ads1015 as ADS
#from adafruit_ads1x15.analog_in import AnalogIn

import matplotlib.pyplot as plt
from spotmicroai.controller import Spotmicro_Animate_lib_009
from spotmicroai.controller import Spotmicro_Gravity_Center_lib_007
from spotmicroai.controller import Spotmicro_lib_020
from spotmicroai.controller.Spotmicro_Lcd_lib import LCDScreenController
from spotmicroai.utilities.log import Logger

#import adafruit_pca9685
#import adafruit_mpu6050
import array
from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import numpy as np
import pygame

def setRGB(r, g, b):
    logController.info("RGB : %s" % (','.join([str(r),str(g),str(b)])))

    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([0x00,0x00]),stop=False)
    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([0x01,0x00]),stop=False)
    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([0x08,0xaa]),stop=False)
    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([4,r]),stop=False)
    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([3,g]),stop=False)
    ##i2c.writeto (DISPLAY_RGB_ADDR,bytes([2,b]),stop=False)

def setLcdScreen(lcdController, status):
    lcdController.status    = status
    lcdController.update_lcd_creen()
    logController.info("%s" % status)

""" Angle measurement Complementary filter """
def comp_filter(angle, t, T):
    # Complementary filter calculates body angles around xt and y axis from IMU data
    ## acc = mpu.acceleration
    ## gyr = mpu.gyro
    acc = [0, 0, 0]
    gyr = [0, 0]

    denb = sqrt(acc[1]**2+acc[2]**2)
    dena = sqrt(acc[2]**2+acc[0]**2)

    if (dena == 0):
        alpha = 0
    else:
        alpha = atan(acc[1]/dena)

    if (denb == 0):
        beta = 0
    else:
        beta = atan(acc[0]/denb)

    A = T/(T+t)

    anglex = A*(angle[0]+t*gyr[0]/180*pi)+(1-A)*alpha
    angley = A*(angle[1]+t*gyr[1]/180*pi)+(1-A)*beta

    return [anglex, angley]

def servo_moving(pos, move):

    global orgthetalf
    global orgthetarf
    global orgthetarr
    global orgthetalr

    thetalf_reply   = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0]+xtlf, pos[1]+ytlf, pos[2]+ztlf, 1)
    thetarf_reply   = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3]+xtrf, pos[4]+ytrf, pos[5]+ztrf, -1)
    thetarr_reply   = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6]+xtrr, pos[7]+ytrr, pos[8]+ztrr, -1)
    thetalr_reply   = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9]+xtlr, pos[10]+ytlr, pos[11]+ztlr, 1)

    thetalf         = thetalf_reply[0]
    thetarf         = thetarf_reply[0]
    thetarr         = thetarr_reply[0]
    thetalr         = thetalr_reply[0]

    if move == True:
        if (thetalf_reply[1] == False):
            try:
                ShoulderAngle   = int(thetalf[0]/pi*180 * Spot.angle_scale_factor_lf1*Spot.dir01+Spot.zero01)
                LegAngle        = int(thetalf[1]/pi*180 * Spot.angle_scale_factor_lf2*Spot.dir02+Spot.zero02)
                FeetAngle       = int(thetalf[2]/pi*180 * Spot.angle_scale_factor_lf3*Spot.dir03+Spot.zero03)
                if (orgthetalf[0] == ShoulderAngle and orgthetalf[1] == LegAngle and orgthetalf[2] == FeetAngle):
                    return
                #else:
                #    print('Front_LEFT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                orgthetalf      = [ShoulderAngle, LegAngle, FeetAngle]
            except ValueError:
                print('Angle out of Range')

        if (thetarf_reply[1] == False):
            try:
                ShoulderAngle   = int(thetarf[0]/pi*180 * Spot.angle_scale_factor_rf1*Spot.dir04+Spot.zero04)
                LegAngle        = int(thetarf[1]/pi*180 * Spot.angle_scale_factor_rf2*Spot.dir05+Spot.zero05)
                FeetAngle       = int(thetarf[2]/pi*180 * Spot.angle_scale_factor_rf3*Spot.dir06+Spot.zero06)
                if (orgthetarf[0] == ShoulderAngle and orgthetarf[1] == LegAngle and orgthetarf[2] == FeetAngle):
                    return
                #else:
                #    print('Front_RIGHT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                orgthetarf      = [ShoulderAngle, LegAngle, FeetAngle]
            except ValueError:
                print('Angle out of Range')

        if (thetarr_reply[1] == False):
            try:
                ShoulderAngle   = int(thetarr[0]/pi*180 * Spot.angle_scale_factor_rr1*Spot.dir07+Spot.zero07)
                LegAngle        = int(thetarr[1]/pi*180 * Spot.angle_scale_factor_rr2*Spot.dir07+Spot.zero08)
                FeetAngle       = int(thetarr[2]/pi*180 * Spot.angle_scale_factor_rr3*Spot.dir09+Spot.zero09)
                
                if (orgthetarr[0] == ShoulderAngle and orgthetarr[1] == LegAngle and orgthetarr[2] == FeetAngle):
                    return
                #else:
                #    print('REAR_RIGHT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                orgthetarr      = [ShoulderAngle, LegAngle, FeetAngle]
            except ValueError:
                print('Angle out of Range')

        if (thetalr_reply[1] == False):
            try:
                ShoulderAngle   = int(thetalr[0]/pi*180 * Spot.angle_scale_factor_lr1*Spot.dir10+Spot.zero10)
                LegAngle        = int(thetalr[1]/pi*180 * Spot.angle_scale_factor_lr2*Spot.dir11+Spot.zero11)
                FeetAngle       = int(thetalr[2]/pi*180 * Spot.angle_scale_factor_lr3*Spot.dir12+Spot.zero12)
                
                if (orgthetalr[0] == ShoulderAngle and orgthetalr[1] == LegAngle and orgthetalr[2] == FeetAngle):
                    return
                #else:
                #    print('REAR_LEFT : ', ShoulderAngle, ', ', LegAngle, ', ', FeetAngle)

                orgthetalr      = [ShoulderAngle, LegAngle, FeetAngle]
            except ValueError:
                print('Angle out of Range')

        print(orgthetalf, orgthetarf, orgthetarr, orgthetalr)


pygame.init()
anim            = True      # animation display
SERVO_Move      = False     # servo move activation
IMU_Comp        = False

logController   = Logger().setup_logger()
logController.info("SpotMicro starting...")


""" Joystick Init """
#pygame.event.set_grab(True)
#pygame.joystick.init()
#joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
#for joy in joysticks:
#    print(joy.get_name(), joy.get_id(), joy.get_guid(), joy.get_instance_id())
numaxes             = 0
numbuttons          = 0
hats                = 0

joystick1           = pygame.joystick.get_count()
if joystick1 == 0:
    # No joysticks!
    print("Sorry, Game have found that no joystick is attached.")
else:
    joystick        = pygame.joystick.Joystick(0)
    joystick.init()
    numaxes         = joystick.get_numaxes()
    numbuttons      = joystick.get_numbuttons()
    hats            = joystick.get_numhats()

""" Display Management """
#DISPLAY_TEXT_ADDR   = 0x3e
LCD_I2C_ADDR    = 0
DISPLAY_RGB_ADDR    = 0x60

lcdController   = LCDScreenController(LCD_I2C_ADDR, logController, joystick1)
Spot            = Spotmicro_lib_020.Spot()
SpotCG          = Spotmicro_Gravity_Center_lib_007.SpotCG(Spot)

if anim == True:
    SpotAnim    = Spotmicro_Animate_lib_009.SpotAnim(Spot)

""" Walking parameters """
b_height        = 200
h_amp2          = 100   # was 100
v_amp2          = 20    # was 50
track2          = 58
h_amp4          = 100   # was 100
v_amp4          = 30    # was 50
track4          = 58
x_offset        = 0     # 40

ra_longi        = 30    # 30
ra_lat          = 40    # 20

bend_angle      = 0

steering        = 200  # Initial steering radius (arbitrary)
walk_direction  = 90/180*pi  # Initial steering angle (arbitrary)
center_x        = steering*cos(walk_direction)
center_y        = steering*sin(walk_direction)

stepl2          = 0.16
stepl4          = 0.125
tstep2          = stepl2/8  # Timing/clock step 0.00666666666666
tstep4          = 0.006666666666

height          = b_height

"""Initialization to 4 steps walk """
stepl           = stepl4
h_amp           = h_amp4
v_amp           = v_amp4
track           = track4
tstep           = tstep4

""" Joystick settings """
but_walk_trot   = 8
but_walk_crawl  = 7
but_sit         = 2
but_lie         = 3
but_twist       = 1
but_pee         = 0
but_move        = 5
but_anim        = 4

pos_frontrear   = 3
pos_leftright   = 2
pos_turn        = 0
pos_rightpaw    = 4
pos_leftpaw     = 5  # also used for hind leg lifting when peeing

joypos          = np.zeros(6)
joybut          = np.zeros(15)

joype           = -1  # Initial joysick value for peeing
joypar          = -1  # Initial joysick value for pawing right
joypal          = -1  # Initial joysick value for pawing left
joybendover     = -1  # Initial joystick value for bending over
joyleanback     = -1  # Initial joystick value for pawing left


""" Data logging intialization"""
distance        = []  # distance to sustentation limit record
balance         = []  # balance status (True or False)
timing          = []  # timing to plot distance
xCG             = []
yCG             = []
xCenter         = []
yCenter         = []
xZMP            = []
yZMP            = []
Integral_Angle  = [0, 0]
sCG             = [0, 0]  # Center of Gravity speed
aCG             = [0, 0]  # Center of Gravity acceleration
tt              = 0  # time to calculate speed and acceleration
dtt             = 0  # real time step
ZMP             = [0, 0]


""" States initialization"""
Free            = True         # Spot is ready to receive new command
sitting         = False
walking         = False     # walking sequence activation
lying           = False
twisting        = False
pawing          = False
shifting        = False
peeing          = False
trot            = False

stop            = False    # walking stop sequence activation
lock            = False    # locking start key temporarily in order to avoid start and stop if start key is pressed too long

#lockmouse   = False
#mouseclick  = False

stance          = [True, True, True, True]
cw              = 1
walking_speed   = 0
walk_direction  = 0
steeering       = 1e6

""" Complementary filter (IMU) initialization """
module          = 0
iangle          = 0
anglex_buff     = np.zeros(10)
angley_buff     = np.zeros(10)
zeroangle_x     = 0.0338
zeroangle_y     = -0.0594
angle_count     = 1  # number of samples to calculate average angles
Tcomp           = 0.02
angle           = np.zeros(2)
Angle           = np.zeros(2)
Angle_old       = np.zeros(2)

""" Counter for Battery check initialization """
Bat             = 0  # counter for battery check

"""Horizontal Compensation PID Gains """
Kp              = 4
Ki              = 22  # was up to 22
Kd              = 0  # was up to 0.08

"""Servo trimming """
xtlf            = 14
ytlf            = 0
ztlf            = 0

xtrf            = 14
ytrf            = 0
ztrf            = 3

xtrr            = 14
ytrr            = 0
ztrr            = 0

xtlr            = 14
ytlr            = 0
ztlr            = 0

orgthetalf      = [0,0,0]
orgthetarf      = [0,0,0]
orgthetarr      = [0,0,0]
orgthetalr      = [0,0,0]

""" Main loop intialization """
continuer       = True
clock           = pygame.time.Clock()
t               = 0  # Initializing timing/clock
tstart          = 1  # End of start sequence
tstop           = 1000  # Start of stop sequence by default
trans           = 0
transtep        = 0.025


""" i2C Initialization"""
##i2c= busio.I2C(board.SCL, board.SDA)
##pca = adafruit_pca9685.PCA9685(i2c)
##mpu = adafruit_mpu6050.MPU6050(i2c)
##pca.frequency = 50
#kitF = ServoKit(channels=16, address=0x40)
#kitB = ServoKit(channels=16, address=0x41)

##ads = ADS.ADS1015(i2c)
##ads.gain = 2/3

"""PWM range setting """
# for i in range(0,6):
##    kitF.servo[Spot.servo_table[i]].set_pulse_width_range(500, 2500)
# for i in range(0,6):
##    kitB.servo[Spot.servo_table[i]].set_pulse_width_range(500, 2500)

""" """
""" Main Program """
""" """


"""set screen background color"""
setRGB(127, 127, 255)


""" Initialize legs and body positions """
x_spot      = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr, 0, 0, 0, 0]
y_spot      = [0, 0, Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track, 0, 0, 0, 0]
z_spot      = [0, b_height, 0, 0, 0, 0, 0, 0, 0, 0]
theta_spot  = [0, 0, 0, 0, 0, 0]


"""theta_spot = [x angle ground, y angle ground, z angle body in space, x angle body, y angle body, z angle body] """
# theta xyz of ground then theta xyz of frame/body
pos_init    = [-x_offset, track, -b_height, -x_offset, -track, -b_height, -x_offset, -track, -b_height, -x_offset, track, -b_height]

thetalf     = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
thetarf     = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
thetarr     = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
thetalr     = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]

CG          = SpotCG.CG_calculation(thetalf, thetarf, thetarr, thetalr)
ZMP         = [CG[0], CG[1]]  # ZMP is initialized to CG position

# Calculation of CG absolute position
M           = Spot.xyz_rotation_matrix( theta_spot[0], theta_spot[1], theta_spot[2], False)
CGabs       = Spot.new_coordinates( M, CG[0], CG[1], CG[2], x_spot[1], y_spot[1], z_spot[1])
dCG         = SpotCG.CG_distance( x_spot[2:6], y_spot[2:6], z_spot[2:6], CGabs[0], CGabs[1], stance)

x_spot      = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr, CG[0], CGabs[0], dCG[1], ZMP[0]]
y_spot      = [0, 0, Spot.ylf+track, Spot.yrf-track, Spot.yrr - track, Spot.ylr+track, CG[1], CGabs[1], dCG[2], ZMP[1]]
z_spot      = [0, b_height, 0, 0, 0, 0, CG[2], CGabs[2], dCG[3], CGabs[2]]

pos         = [-x_offset, track, -b_height, -x_offset, -track, -b_height, -x_offset, -track, -b_height, -x_offset, track, -b_height, theta_spot, x_spot, y_spot, z_spot]

"""
Main Loop
"""
tt                  = time()  # initialize time for speed and acceleration calculation
joystart_rightpaw   = True
joystart_leftpaw    = True
clock_tick          = 100
attention_ready     = True
board_temperature   = 0

setLcdScreen(lcdController, 'Ready')
logController.info('SpotMicro Loop...')

while (continuer):

    clock.tick(clock_tick)
    angle               = comp_filter(angle, tstep, Tcomp)
    anglex_buff[iangle] = angle[0]+zeroangle_x
    angley_buff[iangle] = angle[1]+zeroangle_y
    Angle_old           = Angle
    Angle               = [np.mean(anglex_buff), np.mean(angley_buff)]
    iangle              = iangle+1
    if (iangle == angle_count):
        iangle = 0

    if (joystick1 > 0):
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.QUIT:
                continuer = False

            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
            # JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.", event)
                if (event.joy == 1 and event.button == 11): # START
                    print('event.button START')
                elif (event.joy == 1 and event.button == 5): # LIST
                    print('event.button list')
                elif (event.joy == 1 and event.button == 9): # return
                    print('event.button RETURN')
                elif (event.joy == 1 and event.button == 8): # home
                    print('event.button HOME')

                elif (event.joy == 3 and event.button == 10): # SELECT
                    print('event.button SELECT')
                elif (event.joy == 3 and event.button == 11): # START
                    print('event.button START')

                elif (event.joy == 3 and event.button == 0): # A
                    print('event.button A [ LIE ]')
                    if (stop == True):
                        stop    = False
                    else:
                        stop    = True
                    joybut[but_lie] = int(stop == True)
                elif (event.joy == 3 and event.button == 1): # B
                    print('event.button B - [ but_twist ]')
                    if (stop == True):
                        stop    = False
                    else:
                        stop    = True
                    joybut[but_twist] = int(stop == True)
                elif (event.joy == 3 and event.button == 3): # X
                    print('event.button X - [Infinity Standstill]')
                    if (stop == True):
                        stop    = False
                    else:
                        stop    = True
                    joybut[but_walk_crawl] = int(stop == True)
                elif (event.joy == 3 and event.button == 4): # Y
                    print('event.button Y - [SITDOWN]')
                    joybut[but_sit]     = 1
                    if (stop == True):
                        stop    = False
                    else:
                        stop    = True
                    joybut[but_sit]     = int(stop == True)
                    #joypos[pos_turn]    = 1
                    #but_walk_trot   = 8
                    #but_walk_crawl  = 7
                    #but_sit         = 2
                    #but_lie         = 3
                    #but_twist       = 1
                    #but_pee         = 0
                    #but_move        = 5
                    #but_anim        = 4

                    #pos_frontrear   = 3
                    #pos_leftright   = 2
                    #pos_turn        = 0
                    #pos_rightpaw    = 4
                    #pos_leftpaw     = 5  # also used for hind leg lifting when peeing

                elif (event.joy == 3 and event.button == 6): # L1
                    # SPEED UP
                    clock_tick  = clock_tick + 50
                elif (event.joy == 3 and event.button == 8): # L2
                    # SPEED DOWN
                    clock_tick  = clock_tick - 50
                elif (event.joy == 3 and event.button == 7): # R1
                    print('event.button R1')
                elif (event.joy == 3 and event.button == 9): # R2
                    print('event.button R2')

            elif event.type == pygame.JOYBUTTONUP:
                for i in range(0, numaxes):
                    joypos[i] = 0
                for i in range(0, numbuttons):
                    joybut[i] = 0

            elif event.type == pygame.JOYHATMOTION:
                print('JOYHATMOTION', event)
            elif event.type == pygame.JOYAXISMOTION:
                # LEFT  : 0, 1, 2
                # RIGHT : 3, 4, 5
                print(event)
                if (event.axis == 0 and event.value > 0):
                    joypos[pos_leftright] = -1
                elif (event.axis == 0 and event.value < 0):
                    joypos[pos_leftright] = 1
                elif (event.axis == 1 or event.axis == 2):
                    joybut[but_walk_crawl] = 1
                    joypos[pos_frontrear]  = event.value

                if (event.value == 0):
                    print('JOYAXISMOTION Release')
                    for i in range(0, numaxes):
                        joypos[i] = 0
                    for i in range(0, numbuttons):
                        joybut[i] = 0
            else:
                #print('all Release')
                for i in range(0, numaxes):
                    joypos[i] = 0
                for i in range(0, numbuttons):
                    joybut[i] = 0               
    else:
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.QUIT:  # If user clicked close.
                continuer = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    joybut[but_walk_crawl] = 1
                    joypos[pos_leftright] = -1
                if event.key == pygame.K_RIGHT:
                    joybut[but_walk_crawl] = 1
                    joypos[pos_leftright] = 1
                if event.key == pygame.K_UP:
                    joybut[but_walk_crawl] = 1
                    joypos[pos_frontrear] = -1
                if event.key == pygame.K_DOWN:
                    joybut[but_walk_crawl] = 1
                    joypos[pos_frontrear] = 1
            elif event.type == pygame.KEYUP:
                for i in range(0, 6):
                    joypos[i] = 0
                for i in range(0, 15):
                    joybut[i] = 0

    """Animation"""
    if (joybut[but_walk_crawl] == 0) & (joybut[but_sit] == 0) & (joybut[but_pee] == 0) & (joybut[but_lie] == 0) & (joybut[but_twist] == 0) & (joybut[but_move] == 0) & (joybut[but_anim] == 0) & (lock == True):
        lock = False

    # WALKING
    if (joybut[but_walk_crawl] == 1) & (walking == True) & (stop == False) & (lock == False):  # Quit walk mode
        stop = True
        lock = True
        if (abs(t-int(t)) <= tstep):
            tstop = int(t)
        else:
            tstop = int(t)+1

        if (t == 0):
            tstop = 1

    if (joybut[but_walk_crawl] == 1) & (walking == False) & (Free == True):  # Enter in walk mode
        walking = True
        stop    = False
        Free    = False
        t       = 0
        tstart  = 1
        tstop   = 1000
        lock    = True
        trec    = int(t)
        setRGB(146, 208, 80)
        setLcdScreen(lcdController, 'Walking')

     # SITTING and GIVING PAW
    if (joybut[but_sit] == 1) & (sitting == False) & (Free == True):  # Enter in sitting mode
        sitting = True
        stop    = False
        Free    = False
        t       = 0
        lock    = True
        setRGB(255, 153, 51)
        setLcdScreen(lcdController, 'Sitting')

    if (joybut[but_sit] == 1) & (sitting == True) & (stop == False) & (lock == False):  # Quit sitting mode
        stop        = True
        lock        = True

    #SHIFTING and PEEING
    if (joybut[but_pee] == 1) & (shifting == False) & (Free == True):  # Enter in sitting mode
        shifting    = True
        stop        = False
        Free        = False
        t           = 0
        lock        = True
        setRGB(255, 255, 255)
        setLcdScreen(lcdController, 'Peeing')

    if (joybut[but_pee] == 1) & (shifting == True) & (stop == False) & (lock == False):  # Quit sitting mode
        stop = True
        lock = True

    # LYING
    if (joybut[but_lie] == 1) & (lying == False) & (Free == True):  # Enter in sitting mode
        lying = True
        stop = False
        Free = False
        t = 0
        lock = True
        setRGB(204, 102, 255)
        setLcdScreen(lcdController, 'Lying')

    if (joybut[but_lie] == 1) & (lying == True) & (stop == False) & (lock == False):  # Quit sitting mode
        stop = True
        lock = True

    # TWISTING
    if (joybut[but_twist] == 1) & (twisting == False) & (Free == True):  # Enter in sitting mode
        twisting = True
        Free = False
        t = 0
        lock = True
        setRGB(255, 0, 0)
        setLcdScreen(lcdController, 'Twisting')

    if (walking == True):
        coef = 1.2
        # set walking direction and speed
        # set steering radius
        if (abs(joypos[pos_leftright]) > 0.2) | (abs(joypos[pos_frontrear]) > 0.2) | (stop == True):
            t = t+tstep
            trec = int(t)+1

            module_old = module
            walk_direction_old = walk_direction
            steering_old = steering

            x_old = module_old*cos(walk_direction_old)
            y_old = module_old*sin(walk_direction_old)

            # update request
            module = sqrt(joypos[pos_leftright]**2 + joypos[pos_frontrear]**2)
            walk_direction = (atan2(-joypos[pos_leftright], -joypos[pos_frontrear]) % (2*pi)+pi/2) % (2*pi)

            x_new = module*cos(walk_direction)
            y_new = module*sin(walk_direction)

            # steering update
            if (abs(joypos[pos_turn]) < 0.2):
                cw = 1
                if (steering < 2000):
                    steering = min(1e6, steering_old*coef)
                else:
                    steering = 1e6
            else:
                steering = 2000-(abs(joypos[pos_turn])-0.2)*2000/0.8+0.001
                if ((steering/steering_old) > coef):
                    steering = steering_old*coef
                if ((steering_old/steering) > coef):
                    steering = steering_old/coef
                    if (steering < 0.001):
                        steering = 0.001
                cw = -np.sign(joypos[pos_turn])

            gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)

            if (gap > 0.01):
                x_new = x_old + (x_new-x_old)/gap*0.01
                y_new = y_old + (y_new-y_old)/gap*0.01
                module = sqrt(x_new**2+y_new**2)
                walk_direction = atan2(y_new, x_new)

            # reduces speed sideways and backwards
            min_h_amp = h_amp*(1/2e6*steering+1/2)
            xa = 1+cos(walk_direction-pi/2)
            walking_speed = min(1, module) * min(h_amp, min_h_amp) * (1/8*xa**2+1/8*xa+1/4)

        if ((abs(joypos[pos_leftright]) < 0.2) & (abs(joypos[pos_frontrear]) < 0.2)) & (stop == False):
            t = t+tstep
            module = max(0, module-0.01)
            walking_speed = module * h_amp * ((1+cos(walk_direction-pi/2))/2*0.75+0.25)
            if (steering < 2000):
                steering = min(1e6, steering*coef)
            else:
                steering = 1e6
            cw = 1
            if (t > trec):
                t = trec

        if (joypos[pos_rightpaw] >= joybendover):
            joybendover = min(joypos[pos_rightpaw], joybendover+0.1)
        else:
            joybendover = max(joypos[pos_rightpaw], joybendover-0.1)

        if (joypos[pos_leftpaw] >= joyleanback):
            joyleanback = min(joypos[pos_leftpaw], joyleanback+0.1)
        else:
            joyleanback = max(joypos[pos_leftpaw], joyleanback-0.1)

        front_height = (1-joybendover)*25+b_height-50
        rear_height = (1-joyleanback)*25+b_height-50

        height = (front_height+rear_height)/2
        bend_angle = atan((front_height-rear_height)/Spot.Lb)

        if (joybut[but_sit] == 1) & (lock == False):  # toggle angle compensation with IMU
            IMU_Comp = not (IMU_Comp)
            Integral_Angle = [0, 0]
            lock = True

        if (joybut[but_lie] == 1) & (lock == False) & (t % 1 == 0):
            if (trot == False):
                trot = True
                stepl = stepl2
                h_amp = h_amp2
                v_amp = v_amp2
                track = track2
                tstep = tstep2
            else:
                trot = False
                stepl = stepl4
                h_amp = h_amp4
                v_amp = v_amp4
                track = track4
                tstep = tstep4
            lock = True

        if (trot == True):
            trans = min(1, trans+transtep)

        if (trot == False):
            trans = max(0, trans-transtep)

        if (IMU_Comp == True):
            Integral_Angle[0] = Integral_Angle[0]+Angle[0]*dtt
            Integral_Angle[1] = Integral_Angle[1]+Angle[1]*dtt
            if (dtt != 0):
                Derivative_AngleX = (Angle[0]-Angle_old[0])/dtt
                Derivative_AngleY = (Angle[1]-Angle_old[1])/dtt
            else:
                Derivative_AngleX = 0
                Derivative_AngleY = 0

            theta_spot[3]   = -(Kp*Angle[0]+Ki*Integral_Angle[0] + Kd*Derivative_AngleX)
            theta_spot[4]   = (Kp*Angle[1]+Ki*Integral_Angle[1] + Kd*Derivative_AngleY) + bend_angle
            theta_spot[0]   = Angle[0]
            theta_spot[1]   = -Angle[1]
            x_offset        = (height+CG[2])*sin(Kp*Angle[1] + Ki*Integral_Angle[1]+Kd*Derivative_AngleY)
        else:
            theta_spot[3]   = 0
            theta_spot[4]   = bend_angle
            theta_spot[0]   = 0
            theta_spot[1]   = 0
            x_offset        = 0

        if (t < tstart):
            """ start """
            phase = 1+trans
        else:
            if (t < tstop):
                """ walk """
                phase = 3+trans
            else:
                """ stop """
                phase = 5+trans
        pos = Spot.start_walk_stop(track, x_offset, steering, walk_direction, cw, walking_speed, v_amp, height, stepl, t, tstep, theta_spot, x_spot, y_spot, z_spot, phase)
        theta_spot = pos[12]
        x_spot = pos[13]
        y_spot = pos[14]
        z_spot = pos[15]

        if (t > (tstop+1-tstep)):
            stop = False
            walking = False
            Free = True
            stepl = stepl4
            h_amp = h_amp4
            v_amp = v_amp4
            track = track4
            tstep = tstep4
            trans = 0
            trot = False
            attention_ready = True

            setRGB(127, 127, 255)
            setLcdScreen(lcdController, 'Walking Wait')

    if (sitting == True):
        alpha_sitting = -30/180*pi
        alpha_pawing = 0/180*pi
        L_paw = 220

        x_end_sitting = Spot.xlr-Spot.L2 + Spot.L1 * cos(pi/3) + Spot.Lb/2*cos(-alpha_sitting) - Spot.d*sin(-alpha_sitting)
        z_end_sitting = Spot.L1 * sin(pi/3) + Spot.Lb/2*sin(-alpha_sitting) + Spot.d*cos(-alpha_sitting)
        # x,y,z rotations then translations
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]

        # end_frame_pos = [0,0,0,x_offset,0,b_height-20] # x,y,z rotations then translations
        end_frame_pos = [0, alpha_sitting, 0, x_end_sitting, 0, z_end_sitting]  # x,y,z rotations then translations
        pos = Spot.moving(t, start_frame_pos, end_frame_pos, pos)

        if (t == 1) & (pawing == False):
            pos_sit_init = pos

        if (t == 1):  # pawing is possible
            # paw position is define by joystick 2
            if (pawing == True):
                #print (pos_sit_init[3],pos_sit_init[5])
                pos[3] = pos_sit_init[3] + (L_paw*cos(alpha_pawing) - pos_sit_init[3])*(joypar+1)/2
                pos[5] = pos_sit_init[5] + (-Spot.d-L_paw*sin(alpha_pawing) - pos_sit_init[5])*(joypar+1)/2

                pos[0] = pos_sit_init[0] + (L_paw*cos(alpha_pawing) - pos_sit_init[0])*(joypal+1)/2
                pos[2] = pos_sit_init[2] + (-Spot.d-L_paw*sin(alpha_pawing) - pos_sit_init[2])*(joypal+1)/2

                thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], -1)[0]
                # update of right front leg absolute position
                legrf = Spot.FK(thetarf, -1)
                leglf = Spot.FK(thetalf, -1)
                xlegrf = Spot.xrf+pos[3]
                ylegrf = Spot.yrf+pos[4]
                zlegrf = pos[5]
                xleglf = Spot.xlf+pos[0]
                yleglf = Spot.ylf+pos[1]
                zleglf = pos[2]

                theta_spot_sit = pos[12]

                x_spot_sit = pos[13]
                y_spot_sit = pos[14]
                z_spot_sit = pos[15]

                M = Spot.xyz_rotation_matrix(theta_spot_sit[3], theta_spot_sit[4], theta_spot_sit[2]+theta_spot_sit[5], False)

                paw_rf = Spot.new_coordinates( M, xlegrf, ylegrf, zlegrf, x_spot_sit[1], y_spot_sit[1], z_spot_sit[1])
                paw_lf = Spot.new_coordinates( M, xleglf, yleglf, zleglf, x_spot_sit[1], y_spot_sit[1], z_spot_sit[1])

                x_spot_sit[3] = paw_rf[0]
                y_spot_sit[3] = paw_rf[1]
                z_spot_sit[3] = paw_rf[2]
                x_spot_sit[2] = paw_lf[0]
                y_spot_sit[2] = paw_lf[1]
                z_spot_sit[2] = paw_lf[2]

                pos[13] = x_spot_sit
                pos[14] = y_spot_sit
                pos[15] = z_spot_sit

            joypar_old = joypar
            if (joypal == -1):
                if (((joypos[pos_rightpaw] != 0) & (joypos[pos_rightpaw] != -1)) | (joypar != -1)):
                    pawing = True
                    if (joypos[pos_rightpaw] >= joypar):
                        joypar = min(joypos[pos_rightpaw], joypar+0.05)
                    else:
                        joypar = max(joypos[pos_rightpaw], joypar-0.05)
                else:
                    pawing = False

            if (joypar_old == -1):
                if (((joypos[pos_leftpaw] != 0) & (joypos[pos_leftpaw] != -1)) | (joypal != -1)):
                    pawing = True
                    if (joypos[pos_leftpaw] >= joypal):
                        joypal = min(joypos[pos_leftpaw], joypal+0.05)
                    else:
                        joypal = max(joypos[pos_leftpaw], joypal-0.05)
                else:
                    pawing = False

        if (stop == False):
            t = t+4*tstep
            if (t >= 1):
                t = 1
        elif (pawing == False):
            t = t-4*tstep
            if (t <= 0):
                t = 0
                stop        = False
                sitting     = False
                Free        = True
                setRGB(127, 127, 255)
                setLcdScreen(lcdController, 'Sitting Wait')

    if (shifting == True):
        x_end_shifting = ra_longi
        y_end_shifting = -ra_lat
        # x,y,z rotations then translations
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        # x,y,z rotations then translations
        end_frame_pos = [0, 0, 0, x_end_shifting + x_offset, y_end_shifting, b_height]
        pos = Spot.moving(t, start_frame_pos, end_frame_pos, pos)

        if (t == 1) & (peeing == False):
            pos_shift_init = pos

        if (t == 1):  # pawing is possible
            # paw position is define by joystick 2

            if (peeing == True):
                print(joype)
                pos[9] = pos_shift_init[9] + (0-pos_shift_init[9])*(joype+1)/2
                pos[10] = pos_shift_init[10] + (130-pos_shift_init[10])*(joype+1)/2
                pos[11] = pos_shift_init[11] + (-20-pos_shift_init[11])*(joype+1)/2

                thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                # update of right front leg absolute position
                leglr = Spot.FK(thetalr, 1)
                xleglr = Spot.xlr+pos[9]
                yleglr = Spot.ylr+pos[10]
                zleglr = pos[11]
                theta_spot_shift = pos[12]
                x_spot_shift = pos[13]
                y_spot_shift = pos[14]
                z_spot_shift = pos[15]
                M = Spot.xyz_rotation_matrix(theta_spot_shift[3], theta_spot_shift[4], theta_spot_shift[2]+theta_spot_shift[5], False)
                pee_lr = Spot.new_coordinates(M, xleglr, yleglr, zleglr, x_spot_shift[1], y_spot_shift[1], z_spot_shift[1])

                x_spot_shift[5] = pee_lr[0]
                y_spot_shift[5] = pee_lr[1]
                z_spot_shift[5] = pee_lr[2]
                pos[13] = x_spot_shift
                pos[14] = y_spot_shift
                pos[15] = z_spot_shift
                setRGB(255, 255, int(255*(1-joype)/2))

            # if (joypos[pos_leftpaw] == -1)&(joype == -1):
                #peeing = False

            if ((joypos[pos_leftpaw] != 0) & (joypos[pos_leftpaw] != -1)) | (joype != -1):
                peeing = True
                if (joypos[pos_leftpaw] >= joype):
                    joype = min(joypos[pos_leftpaw], joype+0.1)
                else:
                    joype = max(joypos[pos_leftpaw], joype-0.1)
            else:
                peeing = False

        if (stop == False):
            t = t+4*tstep
            if (t >= 1):
                t = 1
        elif (peeing == False):
            t = t-4*tstep
            if (t <= 0):
                t = 0
                stop = False
                shifting = False
                Free = True
                setRGB(127, 127, 255)
                setLcdScreen(lcdController, 'Shifting Wait')

    if (lying == True):
        angle_lying = 40/180*pi
        x_end_lying = Spot.xlr-Spot.L2 + Spot.L1*cos(angle_lying)+Spot.Lb/2
        z_end_lying = Spot.L1*sin(angle_lying)+Spot.d
        # x,y,z rotations then translations
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        # x,y,z rotations then translations
        end_frame_pos = [0, 0, 0, x_end_lying, 0, z_end_lying]
        pos = Spot.moving(t, start_frame_pos, end_frame_pos, pos)
        if (stop == False):
            t = t+3*tstep
            if (t >= 1):
                t = 1
        else:
            t = t-3*tstep
            if (t <= 0):
                t = 0
                stop = False
                lying = False
                Free = True
                setRGB(127, 127, 255)
                setLcdScreen(lcdController, 'Lying Wait')

    if (twisting == True):
        x_angle_twisting = 0/180*pi
        y_angle_twisting = 0/180*pi
        z_angle_twisting = 30/180*pi
        # x,y,z rotations then translations
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]

        t = t+4*tstep
        if (t >= 1):
            t = 1
            twisting = False
            Free = True
            setRGB(127, 127, 255)
            setLcdScreen(lcdController, 'Twisting Wait')

        if (t < 0.25):
            end_frame_pos = [x_angle_twisting, y_angle_twisting, z_angle_twisting, x_offset, 0, b_height]  # x,y,z rotations then translations
            pos = Spot.moving(t*4, start_frame_pos, end_frame_pos, pos)
        if (t >= 0.25) & (t < 0.5):
            end_frame_pos = [x_angle_twisting, y_angle_twisting, z_angle_twisting, x_offset, 0, b_height]  # x,y,z rotations then translations
            pos = Spot.moving((t-0.25)*4, end_frame_pos, start_frame_pos, pos)
        if (t >= 0.5) & (t < 0.75):
            end_frame_pos = [-x_angle_twisting, -y_angle_twisting, - z_angle_twisting, x_offset, 0, b_height]
            pos = Spot.moving((t-0.5)*4, start_frame_pos, end_frame_pos, pos)
        if (t >= 0.75) & (t <= 1):
            end_frame_pos = [-x_angle_twisting, -y_angle_twisting, - z_angle_twisting, x_offset, 0, b_height]
            pos = Spot.moving((t-0.75)*4, end_frame_pos, start_frame_pos, pos)

    xc = steering * cos(walk_direction)
    yc = steering * sin(walk_direction)

    # absolute center x position
    center_x = x_spot[0]+(xc*cos(theta_spot[2])-yc*sin(theta_spot[2]))
    # absolute center y position
    center_y = y_spot[0]+(xc*sin(theta_spot[2])+yc*cos(theta_spot[2]))

    thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
    thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
    thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
    thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]

    stance = [False, False, False, False]
    if (pos[15][2] < 10.01):
        stance[0] = True
    if (pos[15][3] < 10.01):
        stance[1] = True
    if (pos[15][4] < 10.01):
        stance[2] = True
    if (pos[15][5] < 10.01):
        stance[3] = True

    if (anim == True):
        SpotAnim.animate(pos, t, pi/12, -135/180*pi, Angle, center_x, center_y, thetalf, thetarf, thetarr, thetalr, walking_speed, walk_direction, steering, stance)
        #SpotAnim.animate(pos,t,pi/2,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walk_direction,steering,stance)
        #SpotAnim.animate(pos,t,0,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walk_direction,steering,stance)

    if (joybut[but_anim] == 1) & (lock == False):
        anim = not(anim)
        lock = True

    if (joybut[but_move] == 1) & (Free == True) & (lock == False):
        SERVO_Move = not(SERVO_Move)
        lock = True

    if (anim == True):
        pygame.display.flip()

    if (Free == True):
        sleep(0.1)

    """ CG update """
    CG      = SpotCG.CG_calculation(thetalf, thetarf, thetarr, thetalr)
    # Calculation of CG absolute position
    M       = Spot.xyz_rotation_matrix( theta_spot[0], theta_spot[1], theta_spot[2], False)
    CGabs   = Spot.new_coordinates( M, CG[0], CG[1], CG[2], x_spot[1], y_spot[1], z_spot[1])

    """ Data Logging """
    ta      = time()
    dtt     = ta-tt
    tt      = ta
    sCGo    = sCG
    sCG     = [(CG[0]-pos[13][6])/dtt, (CG[1]-pos[14][6])/dtt]  # in mm/s
    aCG     = [(sCG[0]-sCGo[0])/dtt, (sCG[1]-sCGo[1])/dtt]  # in mm/s²
    ZMP     = [CG[0]-CGabs[2]/9810*aCG[0], CG[1]-CGabs[2]/9810*aCG[1]]  # in mm
    dCG     = SpotCG.CG_distance(x_spot[2:6], y_spot[2:6], z_spot[2:6], CGabs[0], CGabs[1], stance)

    if (sum(stance) > 2):
        xZMP.append(0)
        yZMP.append(0)
        xCG.append(0)
        yCG.append(0)

    else:
        xZMP.append(ZMP[0])
        yZMP.append(ZMP[1])
        xCG.append(CG[0])
        yCG.append(CG[1])

    # Support line center calculation
    xce = 0
    yce = 0
    if (stance[0] == False) | (stance[2] == False):
        xce = (Spot.xrf+pos[3]+Spot.xlr+pos[9])/2
        yce = (Spot.yrf+pos[4]+Spot.ylr+pos[10])/2

    if (stance[1] == False) | (stance[3] == False):
        xce = (Spot.xlf+pos[0]+Spot.xrr+pos[6])/2
        yce = (Spot.ylf+pos[1]+Spot.yrr+pos[7])/2

    xCenter.append(xce)
    yCenter.append(yce)

    """Center of Gravity Data update """
    pos[13][6] = CG[0]  # x
    pos[14][6] = CG[1]  # y
    pos[15][6] = CG[2]  # z

    pos[13][7] = CGabs[0]  # x
    pos[14][7] = CGabs[1]  # y
    pos[15][7] = CGabs[2]  # z

    pos[13][8] = dCG[1]  # xint
    pos[14][8] = dCG[2]  # yint
    pos[15][8] = dCG[3]  # balance

    pos[13][9] = ZMP[0]  # xint
    pos[14][9] = ZMP[1]  # yint
    pos[15][9] = CGabs[2]  # balance

    distance.append(dCG[0])
    timing.append(t)

    servo_moving(pos, SERVO_Move)

setRGB(0, 0, 0)
setLcdScreen(lcdController, 'Exit')

pygame.quit()
plt.plot(timing, distance)