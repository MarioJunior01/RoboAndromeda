#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Color,Stop,Button
from pybricks.tools import wait,StopWatch
from drives.ev3muxdevio import*
from drives.ev3muxdevices import *
motorDr = Motor(Port.A)
motorEs = Motor(Port.B)
motorGarra = Motor(Port.C)
sensor_corEs = ColorSensor(Port.S1)
sensor_corDr = ColorSensor(Port.S2)
sensor_distanciaFrente = InfraredSensor(Port.S3)

sensorDr_Multi=MuxColorSensor(4, 1)
sensor_corEs_Multi=MuxColorSensor(4, 2)

sensor_distanciaLateral = MuxInfraredSensor(4,3)






VELOCIDADE_BASE = 200
VELOCIDADE_CURVA = 100
velocidade = 200
velocidade_curva=100
DISTANCIA_OBJETO= 5

LIMIAR= 15

ALVO = 50
integral = 0
erro_anterior = 0      

KP = 2.5
KI = 0.02
KD = 0.3


ev3 = EV3Brick()

ev3.speaker.set_speech_options(language='pt-br', voice='m2', speed=140, pitch=50)

