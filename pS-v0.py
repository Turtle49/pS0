##MicroRecycling Systems, 2023 -Confidential-

##pScode
# This program monitors a sensor output for particles (changes in signal), classifies particles,
# and sensds a signal (to a pneumatic valve) to open according to timming and particle type.
# The sensors are 3 TCS34725 RGB light sensors, connected via TCA9548A I2C multiplexer.
# Version log:
# v1 ported from TalkingValvingMultiRGB. Added switch for reporting data. Removed CDC.

#import usb_cdc
import time
import math
import board
import busio
import digitalio
from analogio import AnalogIn
import mrs_tcs34725
import adafruit_tca9548a

import gc

import supervisor
#supervisor.disable_autoreload()
supervisor.runtime.autoreload = False

#Device ID
device_ID = 'TalkingValvingMultiRGB #0'

## Define Functions
#Setup

#RGB Digital sensor function
def digitalRGB_sensor(i):
    return list(sensorRGB[i].color_raw) #For TCS34725

#Check RGB vector to reference values
#This function measures the angle between the 'vector' data and multiple reference 'vectors' in the list reference (e.g., RGB_ref) and returns the idex of the lowest value
def RGB_ref_check(data,reference):
    refangles = []
    for ref in reference:
        refangles.append( math.acos( (ref[0]*data[0] + ref[1]*data[1] + ref[2]*data[2]) / (.01 + math.pow((ref[0]**2 + ref[1]**2 + ref[2]**2),.5) * math.pow((data[0]**2 + data[1]**2 + data[2]**2),.5)))  )
    return refangles.index(min(refangles))

#Check RGB vector to reference values
#This function measures the angle between the 'vector' data and multiple reference 'vectors' in the list reference (e.g., RGB_ref) and returns the idex of the lowest value
def is_it_color(data,reference,error):
    angle = math.acos( (reference[0]*data[0] + reference[1]*data[1] + reference[2]*data[2]) / (.01 + math.pow((reference[0]**2 + reference[1]**2 + reference[2]**2),.5) * math.pow((data[0]**2 + data[1]**2 + data[2]**2),.5)))
    if error >= angle:
        result = True
    else:
        result = False
    return result


#Distance between RGB vectors a (minus baseline b) and vector c
#This function works with any type of list, of any length. It does not worl with integers, float, and other non-lists
def distance(a):
    d = 0
    for i in a:
        d += math.pow(i,2)
    return math.pow(d,.5)

##Valve control
def valve_control():
	##Valve controls
	if particle_buffer[-1] == 0:
		relay[1].value = not(relay_on)
	elif tic-particle_buffer[-1] > transit and tic-particle_buffer[-1]< transit + window:
			relay.value = relay_on
	elif tic-particle_buffer[-1] > transit + window:
		del particle_buffer[-1]
		if particle_buffer[-1] == 0:
			relay[1].value = not(relay_on)
        return

## Initialization of components

#Initiate multi RGB sensors
#Ports used by RGB sensor in tca multiplexer
multi_port = [0, 2, 3]

#Create the TCA9548A object and give it the I2C bus
#Set pause time, in seconds,between data requests, so that the I2C can keep up (main loop adds ~15ms)
serial_pause = 0.02
tca = adafruit_tca9548a.TCA9548A(board.I2C())
time.sleep(serial_pause)

#Initiate RGB sensors
sensorRGB = [0,0,0]
for i, port in enumerate(multi_port):
    sensorRGB[i] = mrs_tcs34725.TCS34725(tca[port]) ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial
    time.sleep(serial_pause)
    sensorRGB[i].integration_time = 100 #614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
    sensorRGB[i].gain = 16 #gain of 1, 4, 16, 60

#Tolerance in color determination
color_error = 0.1 #max angle, in radians, between data and reference

#Set valve timing parameters
#Transit time, in seconds: time for a particle to travel from sensor to window of sorting
transit = 0#For noist pump 0.25. For centrigugal 0.1
#Window time, in seconds: window of time a particle travels through the sorting junction
window = .25#For noist pump 0.5. For centrigugal 0.4

#Initialize switch
enable_switch = digitalio.DigitalInOut(board.D4); enable_switch.direction = digitalio.Direction.INPUT; enable_switch.pull = digitalio.Pull.UP

#Initialize buttons
#baseline_reset = digitalio.DigitalInOut(board.xxx); enable_switch.direction = digitalio.Direction.INPUT; enable_switch.pull = digitalio.Pull.UP
baseline_reset = False

# Initialize relays
#On/Off value of relay
relay_on = False#True

#Relay map for jet sorting with Trinket M0
relay1 = digitalio.DigitalInOut(board.D12); relay1.direction = digitalio.Direction.OUTPUT

relay = [0, relay1] #individual relay variables to be eliminated #leave leading "0" so that index matches reality
relay1.value = relay_on; time.sleep(.2); relay1.value = not(relay_on)

##Sorting parameters
base_weight = 0.25 #Data moving average; weight of new data for for moving average; inverse of number of points in moving average
min_threshold = 10 #Minimum size of signal vectors, in bits, to be considered signal, not noise
#Color reference list: [label, ID vector]
data_ref = [['Blue', [-0.456,0.115,0.870]], ['Yellow', [0.716,0.631,0.297]],['Red', [0.782,-0.371,-0.238]],['Black', [-0.657,-0.568,-0.496]]]
color_ref = data_ref[2][1]

#Map each entry in data_ref, in that order, to a bin number, in the same order.
#bin_ref_map = [1 , 1 , 1 , 1]

##Bin parameters
#bin_value_default = 1 #Bin for default stream
#bin_value = 1 #initialize bin variable

#Initilize particle buffer
particle_flag = False
particle_buffer = [[0]]

##Acquire baselines
data_base = [0,0,0,0]
data_long_base = [0,0,0,0,0,0,0,0,0,0,0,0]

#Initializae time variable for valving
tic = 0

print('a')
while True:
    print(enable_switch.value)
    time.sleep(0.2) #Delay to help with analysis
    while enable_switch.value:
        print('c')
        tic = time.monotonic()
        time.sleep(0.2) #Delay to help with analysis
        #Valve control
        valve_control()
        data_temp = [0,0,0,0,0,0,0,0,0,0,0,0]
        data = [0,0,0,0]
        data_long = []
        ##Acquire data
        for i in range(len(multi_port)):
            data_raw = digitalRGB_sensor(i)
            data_long.append(data_raw[0]);data_long.append(data_raw[1]);data_long.append(data_raw[2]); data_long.append(data_raw[3])
        data_temp = [data_long[i]-data_long_base[i] for i in range(len(data_long))]
        #data = [data_long[0] + data_long[4], data_long[1] + data_long[5], 0]####This line causes the overflow...it still needs to be clompleted
        R = data_temp[0] + data_temp[4] + data_temp[7]
        G = data_temp[1] + data_temp[5] + data_temp[8]
        B = data_temp[2] + data_temp[6] + data_temp[9]
        data = [R/3, G/3, B/3, 0]
        print(tuple(data))
        #print(tuple(data_temp))
        #print(tuple(data_long))
        #print(gc.mem_free())
        #print(time.monotonic()-tic)
        #print(enable_switch.value)
        #Valve control
        valve_control()
        ##Data processing and Classification
        #Discriminate small signals and double-counting slow particles
        if distance(data) > min_threshold and baseline_reset == False:
            particle_flag = True
            #Comparison to reference values for identification
            if is_it_color(data,color_ref,color_error):
                #Add particle to the buffer for actuating valves
                particle_buffer.insert(1,tic)
        elif distance(data) < min_threshold:
            #Moving average for long data
            for i in range(len(data_long)):
                data_long_base[i] = (1-base_weight) * data_long_base[i] + base_weight * data_long[i]
