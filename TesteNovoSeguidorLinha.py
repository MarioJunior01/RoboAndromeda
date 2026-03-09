#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait


ev3 = EV3Brick()
motorDr = Motor(Port.A)
motorEs = Motor(Port.B)
sensor_Ir = InfraredSensor(Port.S3)
sensor_corEs = ColorSensor(Port.S1)
sensor_corDr = ColorSensor(Port.S2)


velocidade = 200
velocidade_curva = 100
distancia_obstaculo = 10
desviando = False

# PID variáveis globais
integral = 0
erro_anterior = 0

# Calibração simples
PRETO = 10
BRANCO = 90
ALVO = (PRETO + BRANCO) / 2
MARROM = 70


def andar(vel=velocidade):
    motorDr.run(vel)
    motorEs.run(vel)

def parar():
    motorDr.stop()
    motorEs.stop()
    wait(200)
def curvaSuaveDireita(vel=velocidade):
    motorDr.run(vel)
    motorEs.run(-vel*0.02)
def curvaSuaveEsquerda(vel=velocidade):
    motorDr.run(-vel*0.02)
    motorEs.run(vel)
def virarDireita(vel=velocidade_curva):
    motorEs.run(-vel)
    motorDr.run(vel)

def virarEsquerda(vel=velocidade_curva):
    motorEs.run(vel)
    motorDr.run(-vel)

def re():
    motorDr.run(-velocidade_curva)
    motorEs.run(-velocidade_curva)

# Desviar obstáculo
def desviarObj():
    re()
    wait(1000)
    virarDireita()
    wait(2000)

    corDr = sensor_corDr.reflection()
    corEs = sensor_corEs.reflection()
    while corDr != ALVO and corEs != ALVO:
        motorEs.run(velocidade)
        motorDr.run(velocidade * 0.8)
        wait(100)
        corDr = sensor_corDr.reflection()
        corEs = sensor_corEs.reflection()

        if corDr <= ALVO or corEs <= ALVO:
                    parar()
                    wait(500)
                    ev3.speaker.beep(400)
                    andar()
                    wait(500)
                    curvaSuaveDireita()
                    wait(1500)
                    desviando = False
                    break

# Seguir linha com PID
def seguirLinha():
    global integral, erro_anterior

    
    # Lê valores de reflexão dos sensores (0 = preto, 100 = branco)
    corDr = sensor_corDr.reflection()
    corEs = sensor_corEs.reflection()
    
    erroEs = corEs-ALVO
    erroDr = corDr - ALVO
    
    # Calcula erro (diferença entre os erros dos sensores em relacao ao alvo)
    erro = erroEs - erroDr
    erro_abs = abs(erro)
    if corEs == MARROM  or corDr == MARROM:
        # Só vai
        andar(500)

    # PID clássico
    proporcional = erro 
    integral += erro
    derivativo = erro - erro_anterior
    erro_anterior = erro

    # --- AJUSTE DINÂMICO ---
    # Reduz velocidade se estiver muito fora da linha
    # Ex: erro alto → velocidade menor
    vel_base = velocidade - erro_abs * 2
    vel_base = max(150, min(vel_base, velocidade))  # limita entre 150 e velocidade normal

    # Ajusta ganhos do PID dinamicamente
    # Quanto maior o erro, mais forte a correção (Kp e Kd aumentam)
    Kp = 2.3 + (erro_abs * 0.01)   # aumenta conforme curva basicamente é a força da curva
    Ki = 0.00001          # pequeno para evitar acumulação ou seja ele tenta estabilizar na linha
    Kd = 0.9 + (erro_abs * 0.02)  # deixa o movimento mais suave 

    # Calcula correção PID
    correcao = Kp * proporcional + (Ki * 0.001) * integral + Kd * derivativo

    # Calcula velocidades dos motores
    velA = vel_base + correcao
    velB = vel_base - correcao


    velA = max(-400, min(400, velA))
    velB = max(-400, min(400, velB))

    motorDr.run(velA)
    motorEs.run(velB)


while True:
    distanciaObj = sensor_Ir.distance() 
    if distanciaObj <= distancia_obstaculo:
        parar()
        desviarObj()
    else:
        seguirLinha()
    wait(10) 
