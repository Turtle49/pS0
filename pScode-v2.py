##MicroRecycling Systems, 2023 -Confidential-
##pScode
# This program monitors a sensor output for particles (changes in signal), classifies particles,
# and sensds a signal (to a pneumatic valve) to open according to timming and particle type.
# The sensors are 3 TCS34725 RGB light sensors, connected via TCA9548A I2C multiplexer.
# Version log:
# v0: ported from TalkingValvingMultiRGB. Added switch for reporting data. Removed CDC.
# v0.1:
# v0.2: stable
# v0.3: display support, 'break' button, various fixes
# v0.4: push/hold baseline
# v0.5: CDC enabled
# v1: discrimination from angle alone, no normalization
# v2: general cleanup; IsItTrash instead of 'particle'; added vibratory bowl capabilities

import usb_cdc
import time
import math
import board
import digitalio
import mrs_tcs34725
import adafruit_tca9548a
import supervisor
import pwmio

#Disable autoreload when save
supervisor.runtime.autoreload = False

##Parameters##
#Set valve timing parameters
#Transit time, in seconds: time for a particle to travel from sensor to window of sorting
transit = 0
#Window time, in seconds: window of time a particle travels through the sorting junction
window = .4; particle_arrival = 0 #create arrival timing variable

#RGB sensors SDA and setup parameters
multi_port = [0, 2, 3]
#Sensor integration time, in ms. 614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
RGB_int_time = 20
#Gain of 1, 4, 16, 60
RGB_gain = 60

#Sorting parameters
base_weight = RGB_int_time/1000 #Data moving average; weight of new data for for moving average; inverse of number of points in moving average
min_threshold = 10 #Minimum size of signal vectors, in bits, to be considered signal, not noise
#Color reference list: [label, ID vector]
#data_ref = [['Blue', [-0.456,0.115,0.870]], ['Yellow', [0.716,0.631,0.297]],['Red', [0.782,-0.371,-0.238]],['Black', [-0.657,-0.568,-0.496]]]
color_ref = [[0.716,0.631,0.297],[0.782,-0.371,-0.238]]
#Tolerance in color determination
color_error = 1 #max angle, in radians, between data and reference

# Vibratory bowl parameters
_frequency = 65
_displacement = 20  # Maximum number of rotaional steps, in breaks of 1.8 degree as per stepper motor
_microstepping = 1  # Set to match microsteps per full step on the board (i.e 1 = no microstepping, 2 = 1/2 step; 1/Microstepping)

## Define Functions
#CDC USB com Setup
def node_setup():
    comm_device = usb_cdc.data#[1]
    return comm_device

#Write(sends) data to node
def node_write(data,comm_device):
    comm_device.write(bytes(data, 'utf-8'))

#RGB Digital sensor function
def digitalRGB_sensor(i):
    return list(sensorRGB[i].color_raw) #For TCS34725

#Check RGB vector to reference values
#This function measures the angle between the 'vector' data and multiple reference 'vectors' in the list reference (e.g., RGB_ref) and returns the idex of the lowest value
def is_it_color(data,reference,error):
    angles = []
    for ref in reference:
        angles.append(math.acos( (ref[0]*data[0] + ref[1]*data[1] + ref[2]*data[2]) / (.01 + math.pow((ref[0]**2 + ref[1]**2 + ref[2]**2),.5) * math.pow((data[0]**2 + data[1]**2 + data[2]**2),.5))))
    #now check if it's a sought color
    if error >= min(angles):
        return True
        #print('Particle!')
    else:
        pass
        return False

#Modulus, or size of vector
def distance(a,base):
    d = 0
    for i in range(len(a)):
        d += math.pow(a[i]-base[i],2)
    return math.pow(d,.5)

#Valve checking function
def valve_control(current_time, arrival_time):
    if len(arrival_time)>1 and (current_time - arrival_time[0] > transit) and (current_time - arrival_time[0] < transit + window):
        relay.value = relay_on
        #print('valve on!')
    elif len(arrival_time)>1 and(current_time - arrival_time[0]) > (transit + window):
        relay.value = not(relay_on)
        #particle_arrival = 0
        del arrival_time[0]

## Initialization of components
#Initiate TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(board.I2C())
time.sleep(.02)

#Initiate RGB sensors
sensorRGB = [0]*len(multi_port)
for i, port in enumerate(multi_port):
    sensorRGB[i] = mrs_tcs34725.TCS34725(tca[port]) ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial
    time.sleep(0.02)
    sensorRGB[i].integration_time = RGB_int_time #614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
    sensorRGB[i].gain = RGB_gain

#Initialize vibratory bowl, direction, step, and halt (input)
dir_pin=board.D1; dir_  = digitalio.DigitalInOut(dir_pin); dir_.direction  = digitalio.Direction.OUTPUT; dir_.value = False
step_pin=board.D4; pw_step = pwmio.PWMOut(step_pin, frequency=int(_displacement * _frequency/2), duty_cycle=int(65536/2))
halt_pin=board.D12; halt_ = digitalio.DigitalInOut(halt_pin); halt_.direction = digitalio.Direction.INPUT; halt_.pull = digitalio.Pull.UP

#Initiate CDC USB communication
comm_device = node_setup() ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial

##Initialize buttons
#Initialize printout switch
switch_print = digitalio.DigitalInOut(board.D6); switch_print.direction = digitalio.Direction.INPUT; switch_print.pull = digitalio.Pull.UP
#Initailizase baseline reset and capture
switch_baseline = digitalio.DigitalInOut(board.D9); switch_baseline.direction = digitalio.Direction.INPUT; switch_baseline.pull = digitalio.Pull.UP
#Initialize exit button
switch_exit = digitalio.DigitalInOut(board.D10); switch_exit.direction = digitalio.Direction.INPUT; switch_exit.pull = digitalio.Pull.UP
#Initailizase valve test
switch_valve = digitalio.DigitalInOut(board.D5); switch_valve.direction = digitalio.Direction.INPUT; switch_valve.pull = digitalio.Pull.UP

## Initialize relays
#On/Off value of relay
relay_on = True
#Relay map for jet sorting with Feather M4
#First relay use D13 in Feather M4. For a second relay use pin D24
relay = digitalio.DigitalInOut(board.D13); relay.direction = digitalio.Direction.OUTPUT
relay.value = relay_on; time.sleep(.3); relay.value = not(relay_on)

##Variable initialization
#Initialize baselines
data_base = [0,0,0,0]
data_long_base = [0,0,0,0,0,0,0,0,0,0,0,0]
#initialization counter to capture a baseline automatically
base_init = 0
base_count = 0 #counter for holding baselines

#Initialize time variable for valving
particle_arrival = []

data_raw_multi = [0,0,0,0]*len(multi_port)
data = [0,0,0,0]
data_temp = [0,0,0,0]*len(multi_port)

#Initialize stepper motor timekeeper
step_tic = time.monotonic()

while True:
    data_long = []
    flag_check_multi = [True]*len(multi_port)
    #data_raw_multi = [0,0,0,0]*len(multi_port)
    flag_count_multi = [0]*len(multi_port)

    ##Acquire data
    #Loop for as long as at least one flag remains True
    while True in flag_check_multi:
        #Check all sensors
        for i in range(len(multi_port)):
            #Valve control
            valve_control(time.monotonic(),particle_arrival)
            #Need to check this sensor?
            if flag_check_multi[i] == True:
                flag_count_multi[i] = flag_count_multi[i] +1
                data_raw = digitalRGB_sensor(i)
                # Is the data new? If not, has we asked enough times already before we take the number and go with it.
                if data_raw != data_raw_multi[i] or flag_count_multi[i] > int(RGB_int_time/5): #we will check up to 5 times
                    #Flag, so it won't be read again
                    flag_check_multi[i] = False
                    #Update loop data ckeck
                    data_raw_multi[i] = data_raw
    for i in range(len(multi_port)):
        for j in range(len(data_raw_multi[i])):
            data_long.append(data_raw_multi[i][j])

    ##Data processing and Classification
    #Initialization: loop sets the baseline for the 1st time...it take3s a couple of reads to stabilize
    while base_init < 4:
        #print(distance(data))
        data = [0,0,0,0]
        data_long_base = data_long
        base_init = base_init + 1
        break

    #Discriminate small signals and double-counting slow particles
    if (distance(data_long[0:2],data_long_base[0:2]) > min_threshold) or (distance(data_long[4:6],data_long_base[4:6]) > min_threshold) or (distance(data_long[8:10],data_long_base[8:10]) > min_threshold):
#    if (10) > min_threshold or (2) > min_threshold or (1) > min_threshold:
        #Comparison to reference values for identification
        if  not(is_it_color(data_long[0:3],color_ref,color_error) or is_it_color(data_long[4:6],color_ref,color_error) or is_it_color(data_long[8:10],color_ref,color_error)):
            #Mark time the lastest particle passed by
            particle_arrival.insert(0,time.monotonic())
    else:
        #The signal is below threshold, so it it's ready for particles...flag down
        #Moving average for long data
        for i in range(len(data_long)):
            data_long_base[i] = (1-base_weight) * data_long_base[i] + base_weight * data_long[i]
        #print(data_long_base)

    ##Switches
    # In case of switch A: force basline
    if not switch_baseline.value:
        print('calculating: ',data_long_base)
        base_count = base_count + 1
        for i in range(len(data_long)):
            data_long_base[i] = data_long_base[i] + data_long[i]
        time.sleep(.5)
    else:
        if base_count > 0:
            for i in range(len(data_long_base)):
                data_long_base[i] = data_long_base[i] / (base_count + 1)
            print('Baseline: ',data_long_base)
        base_count = 0
    # In case of switch pedal: print info
    if switch_print.value:
        print(tuple(data_long))
        #print(tuple([data[0], data[1], data[2]]))
        node_write(str(tuple([data[0], data[1], data[2]]))+' \n',comm_device)
        #print(tuple([time.monotonic() , particle_arrival, time.monotonic() - particle_arrival,transit,transit + window, distance(data)]))
        #print(tuple([distance(data),0]))
        if particle_arrival:
            pass
    # In case of switch C: open valve
    if not switch_valve.value:
        relay.value = (relay_on)
    elif switch_valve.value:
        relay.value = not(relay_on)
    # In case of switch B: exit
    if not switch_exit.value:
        break
    #Vibratory bowl direction control
    if (time.monotonic() - step_tic) > (2 * _microstepping/ _frequency):
        dir_.value = not dir_.value
        step_tic = time.monotonic()
    #Vibratory bowl halt
    if not halt_.value:
        print("figure out how to turn off  pw_step = pwmio.PWMOut...")
        
        
