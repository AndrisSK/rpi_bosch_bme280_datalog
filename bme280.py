# Authors:
#        Andrej Trnkoci
#
# Copyright (c) 2018 GPL3 Licensed
#
#
#  GNU GENERAL PUBLIC LICENSE (GPL3)
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#
#  Credit: The compensation algorithms used here were inspired by project
#          by ElektorLabs: 
#          https://github.com/ElektorLabs/bme280-driver/blob/master/BME280.cpp 
#          You may want to reference that project if you are looking for a
#          solution in c/c++.

import smbus
from ctypes import *

def bme280_set_normal_mode(bus, address):
    bus.write_byte_data(address, 0xF4, 0x27)

def bme280_measure_forced(bus, address):
    bus.write_byte_data(address, 0xF4, 0x26) #initiate forced conversion
    conversionInProgress = 0x2
    while conversionInProgress:
        conversionInProgress = bus.read_byte_data(address, 0xF4) & 0x2

def bme280_read_temperature(bus, address):
    #read compensation data
    digT1_l = bus.read_byte_data(address, 0x88)
    digT1_h = bus.read_byte_data(address, 0x89)
    digT2_l = bus.read_byte_data(address, 0x8A)
    digT2_h = bus.read_byte_data(address, 0x8B)
    digT3_l = bus.read_byte_data(address, 0x8C)
    digT3_h = bus.read_byte_data(address, 0x8D)
    #prepare compensation values
    digT1 = float(int.from_bytes([digT1_l, digT1_h], byteorder='little', signed=False))
    digT2 = float(int.from_bytes([digT2_l, digT2_h], byteorder='little', signed=True))
    digT3 = float(int.from_bytes([digT3_l, digT3_h], byteorder='little', signed=True))
    #read temperature data
    temp_msb = bus.read_byte_data(address, 0xFA)
    temp_lsb = bus.read_byte_data(address, 0xFB)
    temp_xlsb = bus.read_byte_data(address, 0xFC)
    #prepare raw temperature value
    adc_T = (temp_msb << 12) | (temp_lsb << 4) | ((temp_xlsb >> 4) & 0xF)
    #compute actual temperature
    v_x1_u32  = ((adc_T) / 16384.0 - (digT1) / 1024.0) * (digT2)
    v_x2_u32  = (((adc_T) / 131072.0 - (digT1) / 8192.0) * ((adc_T) / 131072.0 - (digT1) / 8192.0)) * (digT3)
    t_fine = (v_x1_u32 + v_x2_u32)
    temperature = (v_x1_u32 + v_x2_u32) / 5120.0
    return temperature, t_fine

def bme280_read_humidity(bus, address, t_fine):
    #read humidity compensation data
    dig_H1 = float(bus.read_byte_data(address, 0xA1))
    dig_H2l = bus.read_byte_data(address, 0xE1)
    dig_H2h = bus.read_byte_data(address, 0xE2)
    dig_H2 = float(int.from_bytes([dig_H2l, dig_H2h], byteorder='little', signed=True))
    dig_H3 = float(bus.read_byte_data(address, 0xE3))
    dig_H4l = ((bus.read_byte_data(address, 0xE5) & 0x0F) << 4) #only low 4 bits
    dig_H4h = bus.read_byte_data(address, 0xE4)
    dig_H4 = float(int.from_bytes([dig_H4l, dig_H4h], byteorder='little', signed=True)) / 16
    dig_H5l = bus.read_byte_data(address, 0xE6)
    dig_H5h = bus.read_byte_data(address, 0xE5) & 0xF0 #only high 4 bits
    dig_H5h = dig_H5h | (dig_H5l >> 4)
    dig_H5l = (dig_H5l << 4) & 0xFF
    dig_H5 = float(int.from_bytes([dig_H5l, dig_H5h], byteorder='little', signed=True)) / 16
    dig_H6raw = bus.read_byte_data(address, 0xE7)
    dig_H6 = float(int.from_bytes([dig_H6raw], byteorder='little', signed=True))
    #read humidity raw data
    hum_lsb =  bus.read_byte_data(address, 0xFE)
    hum_msb =  bus.read_byte_data(address, 0xFD)
    raw_hum = float((hum_msb << 8) | hum_lsb)
    #compute relative humidity
    var_h = (t_fine - 76800.0)
    if var_h != 0:
        var_h = (raw_hum - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_h)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_h * (1.0 + dig_H3 / 67108864.0 * var_h)))
    else:
        return 0
    var_h = var_h * (1.0 - dig_H1 * var_h / 524288.0)
    if (var_h > 100.0):
        var_h = 100.0
    elif (var_h < 0.0):
        var_h = 0.0
    return var_h

def bme280_read_pressure(bus, address, t_fine):
    #read pressure compensation data
    dig_P1l = bus.read_byte_data(address, 0x8E)
    dig_P1h = bus.read_byte_data(address, 0x8F)
    dig_P2l = bus.read_byte_data(address, 0x90)
    dig_P2h = bus.read_byte_data(address, 0x91)
    dig_P3l = bus.read_byte_data(address, 0x92)
    dig_P3h = bus.read_byte_data(address, 0x93)
    dig_P4l = bus.read_byte_data(address, 0x94)
    dig_P4h = bus.read_byte_data(address, 0x95)
    dig_P5l = bus.read_byte_data(address, 0x96)
    dig_P5h = bus.read_byte_data(address, 0x97)
    dig_P6l = bus.read_byte_data(address, 0x98)
    dig_P6h = bus.read_byte_data(address, 0x99)
    dig_P7l = bus.read_byte_data(address, 0x9A)
    dig_P7h = bus.read_byte_data(address, 0x9B)
    dig_P8l = bus.read_byte_data(address, 0x9C)
    dig_P8h = bus.read_byte_data(address, 0x9D)
    dig_P9l = bus.read_byte_data(address, 0x9E)
    dig_P9h = bus.read_byte_data(address, 0x9F)
    #prepare pressure compensation values
    dig_P1 = float(int.from_bytes([dig_P1l, dig_P1h], byteorder='little', signed=False))
    dig_P2 = float(int.from_bytes([dig_P2l, dig_P2h], byteorder='little', signed=True))
    dig_P3 = float(int.from_bytes([dig_P3l, dig_P3h], byteorder='little', signed=True))
    dig_P4 = float(int.from_bytes([dig_P4l, dig_P4h], byteorder='little', signed=True))
    dig_P5 = float(int.from_bytes([dig_P5l, dig_P5h], byteorder='little', signed=True))
    dig_P6 = float(int.from_bytes([dig_P6l, dig_P6h], byteorder='little', signed=True))
    dig_P7 = float(int.from_bytes([dig_P7l, dig_P7h], byteorder='little', signed=True))
    dig_P8 = float(int.from_bytes([dig_P8l, dig_P8h], byteorder='little', signed=True))
    dig_P9 = float(int.from_bytes([dig_P9l, dig_P9h], byteorder='little', signed=True))
    #read pressure raw data
    press_msb = bus.read_byte_data(address, 0xF7)
    press_lsb = bus.read_byte_data(address, 0xF8)
    press_xlsb = bus.read_byte_data(address, 0xF9)
    #prepare raw temperature value
    adc_P = (press_msb << 12) | (press_lsb << 4) | ((press_xlsb >> 4) & 0xF)
    #compute pressure
    v_x1_u32 = (t_fine / 2.0) - 64000.0
    v_x2_u32 = v_x1_u32 * v_x1_u32 * dig_P6 / 32768.0
    v_x2_u32 = v_x2_u32 + v_x1_u32 * dig_P5 * 2.0
    v_x2_u32 = (v_x2_u32 / 4.0) + (dig_P4 * 65536.0)
    v_x1_u32 = (dig_P3 * v_x1_u32 * v_x1_u32 / 524288.0 + dig_P2 * v_x1_u32) / 524288.0
    v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) * dig_P1
    pressure = 1048576.0 - adc_P
    # Avoid exception caused by division by zero.
    if v_x1_u32 != 0:
        pressure = (pressure - (v_x2_u32 / 4096.0)) * 6250.0 / v_x1_u32
    else:
        return 0
    v_x1_u32 = dig_P9 * pressure * pressure / 2147483648.0
    v_x2_u32 = pressure * dig_P8 / 32768.0
    pressure = pressure + (v_x1_u32 + v_x2_u32 + dig_P7) / 16.0
    return pressure

#i2c setup
bme280_i2c = smbus.SMBus(1)
bme280_address = 0x76
#initiate bme280 conversion on all sensors
bme280_measure_forced(bme280_i2c, bme280_address)
#read and print temperature
temp_result = bme280_read_temperature(bme280_i2c,bme280_address)
print('Temperature: {:f} °C'.format(temp_result[0]))
#read and print humidity
hum_result = bme280_read_humidity(bme280_i2c,bme280_address, temp_result[1])
print('Humidity:    {:f} %'.format(hum_result))
#read and prin pressure
press_result = bme280_read_pressure(bme280_i2c,bme280_address, temp_result[1])
print('Pressure:    {:f} Pa'.format(press_result))


