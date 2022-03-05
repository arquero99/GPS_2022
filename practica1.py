# -*- coding: utf-8 -*-
"""
Created on Tue Mar  1 16:22:32 2022

@author: raulm
"""

import serial
import numpy as np

gnss = bytearray(200)
cabecera = bytearray(6)
aux = ''
sig_trama = False
uart = serial.Serial(
    port='COM3',
    baudrate=4800,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

# Datos para hallar coordenadas UTM con el elipsoide WGS84.
RADIO_ECUATORIAL = 6378137.0 # Radio ecuatorial (a).
ACHATAMIENTO = 1 / 298.25722  # Achatamiento (f).
EXCENTRICIDAD_2 = 0.00669437999013  # Cuadrado de la excentricidad (e^2).
RADIO_POLAR_CURV = 6399593.626  # Radio polar de curvatura (c).
HUSO = 30  # Huso horario en España.


def latitude(trama): #Norte o Sur
    trama_separada = trama.split(sep=',')        
    
    if trama_separada[3] == 'S':
        latitude = - float(trama_separada[2])
        latitude = gradosMin_to_grados(latitude)
    elif trama_separada[3] == 'N':
        latitude = float(trama_separada[2])
        latitude = gradosMin_to_grados(latitude)
    else:
        latitude = 99.99
    
    if latitude == 99.99:
        ok = False
    else:
        ok = True
    return latitude, ok


def longitude(trama): #Este u Oeste(West)
    trama_separada = trama.split(sep=',')        
    
    if trama_separada[5] == 'W':
        longitude = -float(trama_separada[4])
        longitude = gradosMin_to_grados(longitude)
    elif trama_separada[5] == 'E': 
        longitude = float(trama_separada[4])
        longitude = gradosMin_to_grados(longitude)
    else:
        longitude = 99.99    
    
    if longitude == 99.99:
        ok = False
    else:
        ok = True
        
    return longitude, ok

def gradosMin_to_grados(grad_min):
    grad = int(grad_min/100);
    minut = float(grad_min - (grad*100))
    grados = float(grad + (minut/60.0))
    return grados

def gradosARadianes(grados):  
    return np.radians(grados)


def calcularUTM(latitud, longitud):
    latitud = gradosARadianes(latitud)  # Latitud en radianes (φ).
    longitud = gradosARadianes(longitud)  # Longitud en radianes (λ).

    lambda_0 = np.radians((HUSO * 6) - 183)
    e = EXCENTRICIDAD_2 / (1 - EXCENTRICIDAD_2)  # Excentricidad??
    N = RADIO_ECUATORIAL / np.sqrt(1 - (EXCENTRICIDAD_2 * (np.sin(latitud) ** 2)))
    T = np.tan(latitud) ** 2
    C = e * (np.cos(latitud) ** 2)
    A = np.cos(latitud) * (longitud - lambda_0)
    M = RADIO_ECUATORIAL * ((1-(EXCENTRICIDAD_2/4)-((3/64)*(EXCENTRICIDAD_2**2))-((5/256)*(EXCENTRICIDAD_2**3)))*latitud - ((3*EXCENTRICIDAD_2/8)+((3/32)*(EXCENTRICIDAD_2**2))+((45/1024)*(EXCENTRICIDAD_2**3)))*np.sin(2*latitud) + (((15/256)*(EXCENTRICIDAD_2**2))+((45/1024)*(EXCENTRICIDAD_2**3)))*np.sin(4*latitud) - ((35/3072)*(EXCENTRICIDAD_2**3))*np.sin(6*latitud))
    UTM_easting = 0.9996 * N * (A + (((1-T+C)*(A**3))/6) + (((5-18*T+(T**2)+72*C-58*EXCENTRICIDAD_2)*(A**5))/120)) + 500000
    UTM_norting = 0.9996 * (M + N * np.tan(latitud) * (((A**2)/2) + (((5-T+9*C+4*(C**2))*(A**4))/24) + (((61-58*T+(T**2)+600*C-330*EXCENTRICIDAD_2)*(A**6))/720)))
    return UTM_norting, UTM_easting


uart.reset_input_buffer()
uart.reset_output_buffer()

while uart.isOpen():
    aux = uart.read()
    if aux == str.encode("$") or sig_trama:  # lee el primer byte para ver si es $
        sig_trama = False
        cabecera = uart.read(5)
        if cabecera == str.encode("GPGGA"):
            gnss = ''
            aux = uart.read()
            while aux != str.encode("$"):
                gnss = gnss + bytes.decode(aux)
                aux = uart.read()
            
            print(gnss)
            latitud,ok = latitude(gnss)
            longitud,ok = longitude(gnss)
            print("Longitude: ", longitud,"\n")
            print("Latitude: ", latitud, "\n")
            sig_trama = True
            if ok :
                y,x = calcularUTM(latitud,longitud)
                print("Coordenada X: ", x, "\n")
                print("Coordenada Y: ", y, "\n")
        
        
