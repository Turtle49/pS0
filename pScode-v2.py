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
# v2: general cleanup; IsItTrash instead of 'particle'

import usb_cdc
import time
import math
import board
import busio
import digitalio
from analogio import AnalogIn
import mrs_tcs34725
import adafruit_tca9548a
import supervisor
#supervisor.disable_autoreload()
supervisor.runtime.autoreload = False

## Define Functions
#Setup

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
#def RGB_ref_check(data1,data2):
#    refangles = []
#    for ref in data1:
#        refangles.append( math.acos( (ref[0]*data2[0] + ref[1]*data2[1] + ref[2]*data2[2]) / (.01 + math.pow((ref[0]**2 + ref[1]**2 + ref[2]**2),.5) * math.pow((data2[0]**2 + data2[1]**2 + data2[2]**2),.5)))  )
#    return refangles.index(min(refangles))

#Check RGB vector to reference values
#This function measures the angle between the 'vector' data and multiple reference 'vectors' in the list reference (e.g., RGB_ref) and returns the idex of the lowest value
def is_it_color(data,reference,error):
    angles = []
    for ref in reference:
      angles.append(math.acos( (reference[0]*data[0] + reference[1]*data[1] + reference[2]*data[2]) / (.01 + math.pow((reference[0]**2 + reference[1]**2 + reference[2]**2),.5) * math.pow((data[0]**2 + data[1]**2 + data[2]**2),.5))))
    #now check if it's a sought color
      if error >= min(angles):
          result = True
          #print('Particle!')
      else:
          result = False
    return result


#Modulus, or size of vector
#This function works with any type of list, of any length. It does not worl with integers, float, and other non-lists
def distance(a):
    d = 0
    for i in a:
        d += math.pow(i,2)
    return math.pow(d,.5)

##Valve control
#Set valve timing parameters
#Transit time, in seconds: time for a particle to travel from sensor to window of sorting
transit = 0#For noist pump 0.25. For centrigugal 0.1
#Window time, in seconds: window of time a particle travels through the sorting junction
window = .4
particle_arrival = 0
#Valve checking function
def valve_control(current_time, arrival_time):
    if (current_time - arrival_time > transit) and (current_time - arrival_time < transit + window):
        relay.value = relay_on
        #print('valve on!')
    elif (current_time - arrival_time) > (transit + window):
        relay.value = not(relay_on)
        particle_arrival = 0
#return 0

## Initialization of components

#Initiate multi RGB sensors
#Ports used by RGB sensor in tca multiplexer
multi_port = [0, 2, 3]

#Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(board.I2C())
time.sleep(.02)

#Initiate RGB sensors
sensorRGB = [0]*len(multi_port)
RGB_int_time = 20 #sensor integration time, in ms. 614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
for i, port in enumerate(multi_port):
    sensorRGB[i] = mrs_tcs34725.TCS34725(tca[port]) ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial
    time.sleep(0.02)
    sensorRGB[i].integration_time = RGB_int_time #614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
    sensorRGB[i].gain = 4 #gain of 1, 4, 16, 60

#Tolerance in color determination
color_error = 1 #max angle, in radians, between data and reference

#Initiate CDC USB communication
comm_device = node_setup() ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial

##Initialize buttons
#Initialize printout switch
switch_print = digitalio.DigitalInOut(board.D4); switch_print.direction = digitalio.Direction.INPUT; switch_print.pull = digitalio.Pull.UP
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
relay = digitalio.DigitalInOut(board.D13); relay.direction = digitalio.Direction.OUTPUT
relay.value = relay_on; time.sleep(.3); relay.value = not(relay_on)

##Sorting parameters
base_weight = RGB_int_time/1000 #Data moving average; weight of new data for for moving average; inverse of number of points in moving average
min_threshold = 10 #Minimum size of signal vectors, in bits, to be considered signal, not noise
#Color reference list: [label, ID vector]
data_ref = [['Blue', [-0.456,0.115,0.870]], ['Yellow', [0.716,0.631,0.297]],['Red', [0.782,-0.371,-0.238]],['Black', [-0.657,-0.568,-0.496]]]
color_ref = data_ref[2][1]

#Initilize particle buffer
#particle_flag = False
#particle_buffer = [0]

##Acquire baselines
data_base = [0,0,0,0]
data_long_base = [0,0,0,0,0,0,0,0,0,0,0,0]
#initialization counter to capture a baseline automatically
base_init = 0
base_count = 0 #counter for holding baselines

#Initializae time variable for valving
particle_arrival = 0

data_raw_multi = [0,0,0,0]*len(multi_port)
data = [0,0,0,0]
data_temp = [0,0,0,0]*len(multi_port)

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
    ###  DELETE??XXXXX
    while base_init < 4:
        #print(distance(data))
        data = [0,0,0,0]
        data_long_base = data_long
        base_init = base_init + 1
        break

    #Discriminate small signals and double-counting slow particles
    if (distance(data_long[0:2],data_long_base[0:2]) > min_threshold) or (distance(data_long[4:6],data_long_base[4:6]) > min_threshold) or distance(data_long[8:10],data_long_base[8:10]) > min_threshold:
        #Comparison to reference values for identification
        if  XXXHere I need to test against the angle of the walls and the angle of the particle of interest...everything else needs to go:#is_it_color(data,color_ref,color_error):
            #Mark time the lastest particle passed by
            particle_arrival = time.monotonic() XXXXThis needs to be a que/list/pop thingXXXX
    else:
        #The signal is below threshold, so it it's ready for particles...flag down
        #Moving average for long data
        for i in range(len(data_long)):
            data_long_base[i] = (1-base_weight) * data_long_base[i] + base_weight * data_long[i]
        #print(data_long_base)


    #Switches
    # In case of switch A: force basline
    if not switch_baseline.value:
        print('calculating: ',data_long_base)
        base_count = base_count + 1
        for i in range(len(data_long)):
            data_long_base[i] = data_long_base[i] + data_long[i]
        #display.fill(0)
#        display.text('Baseline Acquired', 15, 50, 1)
#        display.show()
        time.sleep(.5)
    else:
        if base_count > 0:
            #data_long_base = data_long_base / base_count
            for i in range(len(data_long_base)):
                data_long_base[i] = data_long_base[i] / (base_count + 1)
            print('Baseline: ',data_long_base)
        base_count = 0


    # In case of switch pedal: print info
    if switch_print.value:
    #    display.text('Data output', 25, 22, 1)
    #    display.show()
        print(tuple([data[0], data[1], data[2]]))
        node_write(str(tuple([data[0], data[1], data[2]]))+' \n',comm_device)
        #print(tuple([time.monotonic() , particle_arrival, time.monotonic() - particle_arrival,transit,transit + window, distance(data)]))
        #print(tuple([distance(data),0]))
        if particle_arrival:
            pass#print("arrival")

    # In case of switch C: open valve
    if not switch_valve.value:
        relay.value = (relay_on)
    elif switch_valve.value:
        relay.value = not(relay_on)

    # In case of switch B: exit
    if not switch_exit.value:
        break
