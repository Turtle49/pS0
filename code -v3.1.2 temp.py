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
# v3.1: implementation in PCB
# v4: streamlined, loop eliminations, checks for values faster

import usb_cdc
import time
import math
import board
import digitalio
import mrs_tcs34725
import adafruit_tca9548a
import supervisor
import pwmio

# Enable/disable autoreload when save
supervisor.runtime.autoreload = True

##Parameters##
# Set valve timing parameters
# Transit time, in seconds: time for a particle to travel from sensor to window of sorting
transit = 0
# Window time, in seconds: window of time a particle travels through the sorting junction
window = 0.1
particle_arrival = 0  # create arrival timing variable

# RGB sensors SDA and setup parameters
multi_port = [1, 2, 3]
# Sensor integration time, in ms. 614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
RGB_int_time = 20
# Gain of 1, 4, 16, 60
RGB_gain = 16

# Sorting parameters
base_weight = RGB_int_time / 1000  # Data moving average; weight of new data for for moving average; inverse of number of points in moving average
min_threshold = 20  # Minimum size of signal vectors, its modulus, to be considered signal, not noise
min_threshold_RGB = 0 # threhold for **each individual channel**
max_threshold = 200  # debugging

# Color reference list: [label, ID vector]
# data_ref = [['Blue', [-0.456,0.115,0.870]], ['Yellow', [0.716,0.631,0.297]],['Red', [0.782,-0.371,-0.238]],['Black', [-0.657,-0.568,-0.496]]]
#color_ref = [[88.791, 78.2471, 75.2412]]  # Sunshine Plastics gray
color_ref = [[45,40,41]]  # Sunshine Plastics gray via Orange
# Tolerance in color determination
color_error = 0.2  # max angle, in radians, between data and reference
bad_counter_threshold = 1  # max number of readings of "bad" particles before the valve is activated
#flag_count_max = 0  # int(RGB_int_time/5) #number of repeat queries to a sensor that keeps giving the same answer as before

# RGB Digital sensor function
def digitalRGB_sensor(i):
    return list(sensorRGB[i].color_raw)  # For TCS34725


# Check RGB vector to reference values
# This function measures the angle between the 'vector' data and multiple reference 'vectors' in the list reference (e.g., RGB_ref) and returns the idex of the lowest value
def is_it_color(data, reference, error):
    angles = []
    for ref in reference:
        angles.append(
            math.acos(
                (ref[0] * (data[0]) + ref[1] * (data[1]) + ref[2] * (data[2])) / ( 0.01 + math.pow((ref[0] ** 2 + ref[1] ** 2 + ref[2] ** 2), 0.5)
                    * math.pow(((data[0]) ** 2 + (data[1]) ** 2 + (data[2]) ** 2), 0.5))))
    if error >= min(angles):
        return True
    else:
        return False

# Modulus, or size of vector
def distance(a, base):
    d = 0
    for i in range(len(a)):
        d += math.pow(a[i] - base[i], 2)
    return math.pow(d, 0.5)

# Valve checking function
def valve_control(current_time, arrival_time):
    if (
        len(arrival_time) > 1
        and (current_time - arrival_time[0] > transit)
        and (current_time - arrival_time[0] < transit + window)
    ):
        relay.value = relay_on
        # print('valve on!')
    elif len(arrival_time) > 1 and (current_time - arrival_time[0]) > (
        transit + window
    ):
        relay.value = not (relay_on)
        # particle_arrival = 0
        del arrival_time[0]

## Initialization of components
# Initiate TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(board.I2C())
time.sleep(0.02)

# Initiate RGB sensors
sensorRGB = [0] * len(multi_port)
for i, port in enumerate(multi_port):
    sensorRGB[i] = mrs_tcs34725.TCS34725(tca[port])  ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial
    time.sleep(0.02)
    sensorRGB[i].integration_time = RGB_int_time  #Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
    sensorRGB[i].gain = RGB_gain

##Initialize buttons
# Initialize printout switch to keyboard pin 3, button #1
switch_print = digitalio.DigitalInOut(board.D10)
switch_print.direction = digitalio.Direction.INPUT
switch_print.pull = digitalio.Pull.UP
# Initailizase baseline reset and capture to keyboard pin 4, button #2
switch_baseline = digitalio.DigitalInOut(board.D9)
switch_baseline.direction = digitalio.Direction.INPUT
switch_baseline.pull = digitalio.Pull.UP
# Initailizase valve test to keyboard pin 1, button #3
switch_valve = digitalio.DigitalInOut(board.D6)
switch_valve.direction = digitalio.Direction.INPUT
switch_valve.pull = digitalio.Pull.UP
# Initialize exit button to keyboard pin 2, button #4
switch_exit = digitalio.DigitalInOut(board.D5)
switch_exit.direction = digitalio.Direction.INPUT
switch_exit.pull = digitalio.Pull.UP

## Initialize relays
# On/Off value of relay
relay_on = True
# Relay map for jet sorting with Feather M4
# First relay use D23 in Feather M4. For a second relay use pin D24
relay = digitalio.DigitalInOut(board.D23)
relay.direction = digitalio.Direction.OUTPUT
relay.value = relay_on
time.sleep(0.3)
relay.value = not (relay_on)

##Variable initialization
# Initialize baselines
data_base_multi = [[0.0, 0.0, 0.0]] * len(multi_port)
data_base_multi_temp = [0.0, 0.0, 0.0]
base_init = True
base_count = 0
#Loop to read initial baselines
for i in range(len(multi_port)):
    data_base_multi[i] = digitalRGB_sensor(i)
# Initialize time variable for valving
particle_arrival = []
data_raw_multi = [[0.0, 0.0, 0.0]] * len(multi_port)
data_multi = [[0.0, 0.0, 0.0]] * len(multi_port)
data_multi_previous = [[0.0, 0.0, 0.0]] * len(multi_port)
#Initialization of good/bad counters
bad_counter = [0.0, 0.0, 0.0] * len(multi_port)
good_counter = [0.0, 0.0, 0.0] * len(multi_port)

while True:
    ##Data processing and Classification
    # Discriminate small signals and double-counting slow particles (this to be moved up to sensor readout loops)
    for k in range(len(multi_port)):
        # Sensor readout
        # Valve control, because this function needs to run frequently and repeadetly
        valve_control(time.monotonic(), particle_arrival)
        data_raw_multi[k] = digitalRGB_sensor(k)
        data_multi[k] = [x1 - x2 for (x1, x2) in zip(data_raw_multi[k], data_base_multi[k])]
        #Check that the min and max threshold are met
        if distance(data_raw_multi[k], data_base_multi[k]) > min_threshold and distance(data_raw_multi[k], data_base_multi[k]) < max_threshold:
            #Check that min threshold is met for each RGB
            if data_multi[k][0] > min_threshold_RGB and data_multi[k][1] > min_threshold_RGB and data_multi[k][2] > min_threshold_RGB:
                #Check to see if any of the RGB fields repeats, which happens with erratic data
                if data_multi[k][0] == data_multi_previous[k][0] or data_multi[k][1] == data_multi_previous[k][1] or data_multi[k][2] == data_multi_previous[k][2]:
                    pass
                else:
                    #refresh the comparison variable
                    data_multi_previous[k] = [x1 - x2 for (x1, x2) in zip(data_raw_multi[k], data_base_multi[k])]
                    # Comparison to reference values for identification
                    if not is_it_color(data_multi[k], color_ref, color_error):
                        bad_counter[k] = 1
                        good_counter[k] = 0
                        if sum(bad_counter) > bad_counter_threshold:
                            # Mark time for valving
                            particle_arrival.insert(0, time.monotonic())
                            #print(tuple([x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ), ",bad, gray Sunshine", )
                            print(time.monotonic(),",", k+1,",", tuple( [x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ), ",bad,", sum(good_counter),",",sum(bad_counter) ,",", "gray Sunshine", )
                        else:
                            #print(tuple([x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ),",almost bad, gray Sunshine", )
                            print(time.monotonic(),",", k+1,",", tuple( [x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ), ",almost bad,", sum(good_counter),",",sum(bad_counter) ,",", "gray Sunshine", )
                            #pass
                    else:
                        good_counter[k] = 1
                        bad_counter[k] = 0
                        #print(tuple([x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ),",good,gray Sunshine", )
                        print(time.monotonic(),",", k+1,",", tuple( [x1 - x2 for (x1, x2) in zip( data_raw_multi[k], data_base_multi[k] ) ] ), ",good,", sum(good_counter),",",sum(bad_counter) ,",", "gray Sunshine", )
                        # particle is not marked for valving
        else:
            # The signal is below threshold, so it it's ready for particles...flag down
            # Moving average for long data
            data_base_multi[k] = [(base_weight) * x1 + (1 - base_weight) * x2 for (x1, x2) in zip(data_raw_multi[k], data_base_multi[k]) ]

    ##Switches
    # In case of key #1: print
    if not switch_print.value:
        print(tuple(data_multi[0] + data_multi[1] + data_multi[2]))
        # print(tuple(data_raw_multi[0]+data_raw_multi[1]+data_raw_multi[2]))
        # print(tuple(data_base_multi[0]+data_base_multi[1]+data_base_multi[2]))
        # print(tuple([time.monotonic() , particle_arrival, time.monotonic() - particle_arrival,transit,transit + window, distance(data)]))
    # In case of  key #2: acquire a new baseline
    elif not switch_baseline.value:
        while not switch_baseline.value:
            base_count = base_count + 1
            print("**Averaging baseline**  ", data_base_multi)
            for i in range(len(multi_port)):
                for j in range(len(data_raw_multi[i])):
                    data_base_multi[i][j] = data_raw_multi[i][j] + data_base_multi[i][j]
            time.sleep(0.125)
        for i in range(len(multi_port)):
            for j in range(len(data_raw_multi[i])):
                data_base_multi[i][j] = data_base_multi[i][j] / (base_count + 1)
        print("Baseline: ", data_base_multi)
        base_count = 0

    # In case of key #4: exit
    elif not switch_exit.value:
        print('Exiting')
        break

    # In case of  key #3: open valve **Since this case requires the elif to be true most of the time, then it needs to go at the end, otherwise other conditions will not playing

    elif not switch_valve.value:
        relay.value = relay_on
        time.sleep(0.1)
    elif switch_valve.value:
        relay.value = not (relay_on)

    #Garbage collection
    #gc.???
