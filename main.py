#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Color,Stop
from pybricks.tools import wait

from verde import verificar_verde, 

ev3 = EV3Brick()
motorDr = Motor(Port.A)
motorEs = Motor(Port.B)
sensor_Ir = InfraredSensor(Port.S2)
sensor_corEs = ColorSensor(Port.S4)
sensor_corDr = ColorSensor(Port.S3)
sensorDistancia_Multi = InfraredSensor(Port.S1)
motorSensorFrente= Motor(Port.C)

integral = 0
erro_anterior = 0


velocidade = 100
velocidade_curva = 80
distancia_obstaculo = 16
desviando = False

integral = 0
erro_anterior = 0

ALVO = 50          



def desviar_obstaculo():
    parar()
    wait(000)
    virarEsquerda(vel=200)
    andar()
    wait(800)
    ev3.speaker.beep(800)

    # 3. alinhar lado
    virarDireita(vel=200)

    # 4. andar um tempo FIXO (evita bug)
    andar()
    wait(1200)

    # 5. usar sensor lateral SÓ PRA GARANTIR que passou
   

    
    andar(250)
    wait(10)

    parar()
    wait(300)

    # 6. voltar pra linha
    virarDireita(200)

    # 7. procurar linha
    parar()
    wait(300)

    # 8. alinhar
    virarEsquerda()
    ev3.speaker.beep(600)

                
d
while True:
    distanciaObj = sensor_Ir.distance() 
    if distanciaObj <= distancia_obstaculo:
        desviar_obstaculo()
        wait(30)
  
    else:
         seguirLinha()
         wait(30)   
