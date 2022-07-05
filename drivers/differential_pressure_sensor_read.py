import smbus
import time

#########################
# SensorName: TE4525DO-DS5AI001DP
# Interface: I2C
#########################

# $ sudo i2cdetect -y 1
SENSOR_ADDR = 0x28

# Sensor normalized range
P_MAX = 1
P_MIN = -P_MAX

# Define constants
PSI_TO_Pa = 6894.75729

# Self-defined difference tolerance
DP = 0.005
DT = 0.005

def PrintStatusInfo(status):
    if (status == 0):
        print("Normal operation. Good data packet")
    elif (status == 1):
        print("Reserved")
    elif (status == 2):
        print("Stale data. Data has been fetched since last measurement cycle")
    elif (status == 3):
        print("Fault detected")
    else:
        print("Unknown status!! Status is: ", status)

def GetData(bus):
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0x00, 4)
    status = (data[0] & 0b11000000) >> 6
    pressure_raw = ((data[0] & 0b00111111) << 8) + data[1]
    temp_raw = (data[2] << 3) + ((0b11100000 & data[3]) >> 5)
    temperature = temp_raw * 200.0 / 2047 - 50.0
    pressure = (pressure_raw - 0.1*16383) / (0.8*16383/(P_MAX - P_MIN)) + P_MIN
    return status, pressure, temperature

if __name__ == '__main__':
    
    # Get I2C bus
    bus = smbus.SMBus(1)

    # Today's air density <-- temperature, airpressure, relative humidity 
    air_density = 1.16 # [kg/m3]

    try:
        while(True):
            status1, pressure1, temperature1 = GetData(bus)
            status2, pressure2, temperature2 = GetData(bus)
            
            # Status check
            if (status1!=0 and status1!=1) or (status2!=0 and status2!=1):
                print("===== data status checking... =====")
                PrintStatusInfo(status1)
                PrintStatusInfo(status2)
                print("===================================")

            # Difference check
            if (abs(pressure1 - pressure2) > DP or abs(temperature1 - temperature2) > DT):
                print("WARNING: two data bags don't match")
            
            pressure = (pressure1 + pressure2) / 2 * PSI_TO_Pa
            temperature = (temperature1 + temperature2) / 2

            vel = (2*abs(pressure)/air_density)**0.5
            print("Current velocity: %.3f [m/s]; Current pressure: %.3f [Pa]; Current temperature: %.1f [*C]" %(vel, pressure, temperature))
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        pass

# Documentation
# Data Structure Definition: [Page 15] https://www.sensorway.cn/Upload/2011/1/129388649843437500.pdf
# Status Definition: https://www.azosensors.com/article.aspx?ArticleID=182
# Reading Transfer Equation: [Page 5] https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS4525DO&DocType=DS&DocLang=English
# Code Reference: https://github.com/ArduPilot/ardupilot/blob/5f8733532544a9a73cc6742ee3b21282ba8929eb/libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp
#                 https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/differential_pressure/ms4525do/MS4525DO.cpp




