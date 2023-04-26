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

#import usb_cdc
import time
import math
import board
import busio
import digitalio
from analogio import AnalogIn
import mrs_tcs34725
import adafruit_tca9548a

import adafruit_ssd1306



import supervisor
#supervisor.disable_autoreload()
supervisor.runtime.autoreload = False


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
        print('Particle!')
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
	#print(time.monotonic()-tic)
    ##Valve controls
	if particle_buffer[-1] == 0:
		relay.value = not(relay_on)
	elif tic-particle_buffer[-1] > transit and tic-particle_buffer[-1]< transit + window:
			relay.value = relay_on
	elif tic-particle_buffer[-1] > transit + window:
		del particle_buffer[-1]
		if particle_buffer[-1] == 0:
			relay.value = not(relay_on)
        return

## Initialization of components

#Initiate multi RGB sensors
#Ports used by RGB sensor in tca multiplexer
multi_port = [0, 2, 3]

#Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(board.I2C())
time.sleep(.02)

#Initiate RGB sensors
sensorRGB = [0]*len(multi_port)
RGB_int_time = 250#60 #sensor integration time, in ms. 614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
for i, port in enumerate(multi_port):
    sensorRGB[i] = mrs_tcs34725.TCS34725(tca[port]) ##WARNING: in Trinket M0 the UART object MUST be created before I2C https://learn.adafruit.com/circuitpython-essentials/circuitpython-uart-serial
    time.sleep(0.02)
    sensorRGB[i].integration_time = RGB_int_time #614.4/10 #for integration time 2.4 - 614.4 ms. Make sure the TCS34725 library adafruit_tcs34725.py has been modified, line 224
    sensorRGB[i].gain = 16 #gain of 1, 4, 16, 60

#Tolerance in color determination
color_error = 1 #max angle, in radians, between data and reference

#Initialize pressure sensor
sensor_pressure = AnalogIn(board.A0)
sensor_pressure_top = 2000 #top pressure range value, kPa
sensor_pressure_bot = -2000 #top pressure range value, kPa

#Initiate display
display = adafruit_ssd1306.SSD1306_I2C(128, 64, tca[5])
display.invert(True)
display.fill(0)
display.text('Sorting', 40, 3, 1)
display.show()

#Set valve timing parameters
#Transit time, in seconds: time for a particle to travel from sensor to window of sorting
transit = 0#For noist pump 0.25. For centrigugal 0.1
#Window time, in seconds: window of time a particle travels through the sorting junction
window = .25#For noist pump 0.5. For centrigugal 0.4

##Initialize buttons
#Initialize printout switch
switch_print = digitalio.DigitalInOut(board.D4); switch_print.direction = digitalio.Direction.INPUT; switch_print.pull = digitalio.Pull.UP
#Initailizase baseline reset and capture
switch_baseline = digitalio.DigitalInOut(board.D9); switch_baseline.direction = digitalio.Direction.INPUT; switch_baseline.pull = digitalio.Pull.UP
#switch_baseline = False
switch_exit = digitalio.DigitalInOut(board.D10); switch_exit.direction = digitalio.Direction.INPUT; switch_exit.pull = digitalio.Pull.UP

# Initialize relays
#On/Off value of relay
relay_on = True

#Relay map for jet sorting with Feather M4
relay = digitalio.DigitalInOut(board.D13); relay.direction = digitalio.Direction.OUTPUT

relay.value = relay_on; time.sleep(.2); relay.value = not(relay_on)

##Sorting parameters
base_weight = RGB_int_time/1000 #Data moving average; weight of new data for for moving average; inverse of number of points in moving average
min_threshold = const(10) #Minimum size of signal vectors, in bits, to be considered signal, not noise
#Color reference list: [label, ID vector]
data_ref = [['Blue', [-0.456,0.115,0.870]], ['Yellow', [0.716,0.631,0.297]],['Red', [0.782,-0.371,-0.238]],['Black', [-0.657,-0.568,-0.496]]]
color_ref = data_ref[2][1]


#Initilize particle buffer
particle_flag = False
particle_buffer = [0]

##Acquire baselines
data_base = [0,0,0,0]
data_long_base = [0,0,0,0,0,0,0,0,0,0,0,0]

#Initializae time variable for valving
tic = 0

data_raw_multi = [0,0,0,0]*len(multi_port)
data = [0,0,0,0]
data_temp = [0,0,0,0]*len(multi_port)

while True:
    #time.sleep(0.2) #Delay to help with analysis
    tic = time.monotonic()
    #Valve control
    #valve_control()
    data_long = []
    flag_check_multi = [True]*len(multi_port)
    #data_raw_multi = [0,0,0,0]*len(multi_port)
    flag_count_multi = [0]*len(multi_port)

    #Display data
    display.fill(0)
    display.text(str(data), int(64-6*len(str(data))/2), 20, 1)
    display.text(str(sensor_pressure.value * (sensor_pressure_top-sensor_pressure_bot) / 65536 + sensor_pressure_bot), 40, 40, 1)
    display.show()

    ##Acquire data
    #Loop for as long as at least one flag remains True
    while True in flag_check_multi:
        #Check all sensors
        #print('multi',time.monotonic()-tic)
        valve_control()
        for i in range(len(multi_port)):
            #Need to check this sensor?
            if flag_check_multi[i] == True:
                flag_count_multi[i] = flag_count_multi[i] +1
                data_raw = digitalRGB_sensor(i)
                #print('1')
                #print(i,data_raw, data_raw_multi[i], flag_count_multi)
                # Is the data new? If not, has we asked enough times already before we take the number and go with it.
                if data_raw != data_raw_multi[i] or flag_count_multi[i] > int(RGB_int_time/10): #we will check up to 10 times
                    #print('2', data_raw, data_raw_multi[i], flag_count_multi,flag_check_multi)
                    #Flag, so it won't be read again
                    flag_check_multi[i] = False
                    #Update loop data ckeck
                    data_raw_multi[i] = data_raw
                    #Now we include the data in our sausage
                    #data_long.append(data_raw[0]);data_long.append(data_raw[1]);data_long.append(data_raw[2]); data_long.append(data_raw[3])

    #Now we include the data in our sausage
    #data_long.append(data_raw_multi[i] for i in range(len(multi_port)))
    #print(data_raw_multi)
    for i in range(len(multi_port)):
        #print(data_raw_multi[i])
        for j in range(len(data_raw_multi[i])):
            data_long.append(data_raw_multi[i][j])

    #print(data_long)
    data_temp = [data_long[i]-data_long_base[i] for i in range(len(data_long))]
    #data = [data_long[0] + data_long[4], data_long[1] + data_long[5], 0]####This line causes the overflow...it still needs to be clompleted
    R = data_temp[0] + data_temp[4] + data_temp[7]
    G = data_temp[1] + data_temp[5] + data_temp[8]
    B = data_temp[2] + data_temp[6] + data_temp[9]
    data = [int(R/3), int(G/3), int(B/3)]

    ##Data processing and Classification
    #Discriminate small signals and double-counting slow particles
    if distance(data) > min_threshold:
        particle_flag = True
        #Comparison to reference values for identification
        if  is_it_color(data,color_ref,color_error):
            #Add particle to the buffer for actuating valves
            particle_buffer.insert(1,tic)
    elif distance(data) < min_threshold:
        #print('bip',time.monotonic())
        #Moving average for long data
        for i in range(len(data_long)):
            data_long_base[i] = (1-base_weight) * data_long_base[i] + base_weight * data_long[i]
    # In case of switch to force basline
    if not switch_baseline.value:
        data_long_base = data_long
        #display.fill(0)
        display.text('Baseline Acquired', 15, 50, 1)
        display.show()
        time.sleep(.5)
    #valve_control()

    #print('end')
    #print(time.monotonic()-tic)
    if switch_print.value:
        display.text('Data output', 25, 22, 1)
        display.show()
        print(tuple(data))
        #print(time.monotonic()-tic,tuple(data_long))
        print(time.monotonic()-tic)
        #print(tuple(data_temp))
        #print(tuple(data_long))
        #print(gc.mem_free())
        #print(time.monotonic()-tic)
        #print(enable_switch.value)
        #Valve control
    # Exit routine
    if not switch_exit.value:
        break