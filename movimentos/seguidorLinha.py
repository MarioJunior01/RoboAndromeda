#!/usr/bin/env pybricks-micropython

from calibracao.constantes import*
from movimentos.movimentosBasicos import *
from calibracao.hsv import*
def seguidor_linha():
    
   
   
    global erro_anterior
    global integral
        
   
    r1, g1, b1 = sensor_corEs.rgb()
    r2, g2, b2 = sensor_corDr.rgb()
        
      
    _, _, v1 = rgb_to_hsv(r1, g1, b1)
    _, _, v2 = rgb_to_hsv(r2, g2, b2)
        
    
    erro = v1 - v2
        

    proporcional = erro * KP
        
    integral = integral + erro

    if integral > 100: integral = 100
    elif integral < -100: integral = -100
    i_output = integral * KI
        
    derivada = (erro - erro_anterior) * KD
        
        # Sinal total de correção (Turn)
    turn = proporcional + i_output + derivada
        
       
    velocidade_esquerda = VELOCIDADE_BASE + turn
    velocidade_direita = VELOCIDADE_BASE - turn
        
    motorEs.run(velocidade_esquerda)
    motorDr.run(velocidade_direita)
        
    erro_anterior = erro
        
    wait(10)