#!/usr/bin/env pybricks-micropython

from calibracao.constantes import*
from movimentos.movimentosBasicos import *
from calibracao.hsv import*
from movimentos.desvia import*
def seguidor_linha():


    curva = detectar_curva_90()

    if curva == 'ambos':
        return

    if curva is not None:
        virar_90(curva)
        return
   
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



def detectar_curva_90():
    """
    Retorna 'direita', 'esquerda' ou None (sem curva),
    combinando os sensores centrais (perderam a linha)
    com os sensores laterais (estão vendo a linha).
    """

    # sensores centrais (Dr, Es
    liminarDr= sensor_corEs_Multi.reflection()
    liminarEs= sensorDr_Multi.reflection()

    if liminarDr < LIMIAR:
        return 'direita'

    if liminarEs < LIMIAR:
        return 'esquerda'
    
    if liminarDr < LIMIAR and liminarEs<LIMIAR:
        return 'ambos'

    return None
def esperar_linha_apos_curva(tentativas=150):
    # tentativas=150 * 20ms = ~3s de tolerância, ajuste conforme seu robô

    if tentativas <= 0:
        parar()
        return

    if not encontrou_linha():
        wait(10)
        esperar_linha_apos_curva(tentativas - 1)

def virar_90(direcao):

    parar()
    wait(100)

    if direcao == 'direita':
        parar()
        wait(100)
        virarEsquerda(200)   # pivota no eixo, para a direita
    if direcao=="esquerda":
       parar()
       wait(100)
       virarDireita(200)
        
    wait(400)

    esperar_linha_apos_curva()

    resetar_pid()

    parar()
    wait(100)


def resetar_pid():
    global integral, erro_anterior
    integral = 0
    erro_anterior = 0    