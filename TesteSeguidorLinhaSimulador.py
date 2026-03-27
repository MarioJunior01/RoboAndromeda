#!/usr/bin/env python3

# Import the necessary libraries
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

ev3 = EV3Brick()
motorDr = Motor(Port.A)
motorEs = Motor(Port.B)



sensor_corDr = ColorSensor(Port.S1)
sensor_corEs = ColorSensor(Port.S2)
sensor_Ir = UltrasonicSensor(Port.S3)



velocidade = 300
velocidade_curva = 200
distancia_obstaculo = 5
desviando = False

# PID variáveis globais
integral = 0
erro_anterior = 0

# Calibração simples
PRETO = 10
BRANCO = 90
ALVO = (PRETO + BRANCO) / 2

ultima_cor = "branco"

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
    wait(500)
    virarDireita()
    wait(1300)
    
    corDr = sensor_corDr.color()
    corEs = sensor_corEs.color()
    while corDr != Color.BLACK or corEs != Color.BLACK:
        motorEs.run(velocidade)        
        motorDr.run(velocidade * 0.6)
        wait(100)
        corDr = sensor_corDr.color()
        corEs = sensor_corEs.color()
        
        if corDr == Color.BLACK or corEs == Color.BLACK:
            parar()
            wait(500)
            ev3.speaker.beep(400)
            andar()
            wait(200)
            curvaSuaveDireita()
            wait(1500)
            desviando = False
            break

# Seguir linha com PID

def verVerde():
    """
    Identifica o verde com filtro de erro e executa a manobra completa.
    Retorna True se executou manobra, False se não viu nada.
    """
    # 1. Leitura inicial
    corEs = sensor_corEs.color()
    corDr = sensor_corDr.color()

    # Se ninguém viu verde, sai rápido para não travar o PID
    if  corEs != Color.GREEN and corDr != Color.GREEN:
        return False

    # --- FILTRO ANTI-FANTASMA (Confirmação) ---
    # Espera 30ms e lê de novo para ter certeza que não foi um brilho na borda preta
    wait(300)
    corEs = sensor_corEs.color()
    corDr = sensor_corDr.color()
    
    if  corEs != Color.GREEN and corDr != Color.GREEN:
        return False # Era um alarme falso, volta para o PID
    # ------------------------------------------

    # Se chegou aqui, o verde é real!
    ev3.speaker.beep(500, 100) # Bipe curto para avisar que detectou
    
    # AVANÇO ESTRATÉGICO
    # Alinha o eixo das rodas com a linha horizontal do cruzamento
    motorEs.run_time(250, 450, wait=False)
    motorDr.run_time(250, 450, wait=True)
    parar()

    # LÓGICA DE DECISÃO DAS MANOBRAS
    if  corEs == Color.GREEN and corDr == Color.GREEN:
        print("Manobra: Meia Volta")
        motorEs.run(-250)
        motorDr.run(250)
        wait(800) # Tempo de "cegueira" para sair da linha atual
        while sensor_corDr.reflection() > (PRETO + 15): # Busca a nova linha
            wait(5)

    elif    corEs == Color.GREEN:
        print("Manobra: Esquerda")
        motorEs.run(-250)
        motorDr.run(250)
        wait(450) # Tempo de "cegueira"
        while sensor_corDr.reflection() > (PRETO + 15):
            wait(5)

    elif corDr == Color.GREEN:
        print("Manobra: Direita")
        motorEs.run(250)
        motorDr.run(-250)
        wait(450) # Tempo de "cegueira"
        while sensor_corEs.reflection() > (PRETO + 15):
            wait(5)

    parar()
    return True # Indica que uma manobra foi feita (o PID deve ser pulado)
            
def seguirLinha():
    global integral, erro_anterior

    # Lê valores de reflexão dos sensores (0 = preto, 100 = branco)
    corDr = sensor_corDr.reflection()
    corEs = sensor_corEs.reflection()
     
    erroEs = corEs - ALVO
    erroDr = corDr - ALVO
    
    # Calcula erro (diferença entre os erros dos sensores em relacao ao alvo)
    erro = erroEs - erroDr
    erro_abs = abs(erro)

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
    Kp = 1.2 + (erro_abs * 0.09)   # aumenta conforme curva basicamente é a força da curva
    Ki = 0.001                     # pequeno para evitar acumulação ou seja ele tenta estabilizar na linha
    Kd = 0.08 + (erro_abs * 0.02)  # deixa o movimento mais suave 

    # Calcula correção PID
    correcao = Kp * proporcional + Ki * integral + Kd * derivativo

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
        # Só procura verde se o erro do PID não for gigantesco
        # Isso evita ver verde "fantasma" nas bordas das curvas
        if abs(erro_anterior) < 40: 
            if not verVerde():
                seguirLinha()
        else:
            seguirLinha()
