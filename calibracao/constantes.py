#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Color,Stop,Button
from pybricks.tools import wait,StopWatch


motorDr = Motor(Port.A)
motorEs = Motor(Port.B)
sensor_corEs = ColorSensor(Port.S1)
sensor_corDr = ColorSensor(Port.S2)
sensor_distanciaFrente = InfraredSensor(Port.S3)
VELOCIDADE_BASE = 200
VELOCIDADE_CURVA = 100
velocidade = 100
velocidade_curva=100
DISTANCIA_OBJETO= 10
LIMIAR_PRETO = 10

ALVO = 50
integral = 0
erro_anterior = 0      

KP = 3.5  
KI = 0.0001 
KD = 0.1 


ev3 = EV3Brick()

ev3.speaker.set_speech_options(language='pt-br', voice='m2', speed=140, pitch=50)

