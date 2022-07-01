import smbus
import time

P_MAX = 1
P_MIN = -P_MAX

DP = 1
DT = 5

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
    data = bus.read_i2c_block_data(0x28, 0x00, 4)
    status = (data[0] & 0b11000000) >> 6
    pressure_raw = ((0b00111111 & data[0]) << 8) + data[1]
    temp_raw = ((0b11100000 & data[3]) >> 5) + (data[2] << 3)
    temperature = temp_raw * 200.0 / 2047 - 50.0
    pressure = (pressure_raw - 0.1*16383) / (0.8*16383/(P_MAX - P_MIN)) + P_MIN
    return status, pressure, temperature

if __name__ == '__main__':
    
    # Get I2C bus
    bus = smbus.SMBus(1)

    while(True):
        status1, pressure1, temperature1 = GetData(bus)
        status2, pressure2, temperature2 = GetData(bus)
        if (status1==2 or status1==3 or status2==2 or status2==3):
            print("===== data status checking... =====")
            PrintStatusInfo(status1)
            PrintStatusInfo(status2)
            print("===== =====")
            # time.sleep(0.5)
            # continue
        if (abs(pressure1 - pressure2) > DP or abs(temperature1 - temperature2) > DT):
            print("WARNING: two data bags don't match")
        
        pressure = (pressure1 + pressure2 )/2
        temperature = (temperature1 + temperature2 )/2
        print("The pressure is %.5f [psi]" %pressure)
        # print("The temperature is %.1f [C*]" %temperature)
        time.sleep(0.1)




