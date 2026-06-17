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

def andar(vel=velocidade):
    wait(1000)
    motorDr.run(vel)
    motorEs.run(vel)

def parar():
    wait(1000)
    motorDr.stop()
    motorEs.stop()
    wait(200)
def curvaSuaveDireita(vel=velocidade):
    wait(1000)
    motorDr.run(vel)
    motorEs.run(-vel*0.02)
def curvaSuaveEsquerda(vel=velocidade):
    wait(1000)
    motorDr.run(-vel*0.02)
    motorEs.run(vel)

def virarDireita(vel=velocidade_curva):
    wait(1000)
    motorEs.run(-vel)
    motorDr.run(vel)

def virarEsquerda(vel=velocidade_curva):
    wait(1000)
    motorEs.run(vel)
    motorDr.run(-vel)

def re():
    wait(1000)
    motorDr.run(-velocidade_curva)
    motorEs.run(-velocidade_curva)


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

                
def seguirLinha():
   
    global integral, erro_anterior

    corDr = sensor_corDr.reflection()
    corEs = sensor_corEs.reflection()
    
    erroEs = corEs-ALVO
    erroDr = corDr - ALVO
    
    
    erro = erroEs - erroDr
    erro_abs = abs(erro)

    proporcional = erro 
    integral += erro
    derivativo = erro - erro_anterior
    erro_anterior = erro

   
    vel_base = velocidade - erro_abs * 2
    vel_base = max(150, min(vel_base, velocidade)) 

 
    Kp = 2.5 + (erro_abs * 0.01)  
    Ki = 0     
    Kd = 1.5 + (erro_abs * 0.02)   

    correcao = Kp * proporcional + (Ki * 0.001) * integral + Kd * derivativo

    velA = vel_base + correcao
    velB = vel_base - correcao


    velA = max(-400, min(400, velA))
    velB = max(-400, min(400, velB))
    ev3.screen.print("Erro Dr: ", int(erroDr), " Es: ", int(erroEs))

    motorDr.run(velA)
    motorEs.run(velB)
    
motorSensorFrente.run_angle(speed=0, rotation_angle=0,then=Stop.HOLD)
while True:
    distanciaObj = sensor_Ir.distance() 
    if distanciaObj <= distancia_obstaculo:
        desviar_obstaculo()
        wait(30)
  
    else:
         seguirLinha()
         wait(30)