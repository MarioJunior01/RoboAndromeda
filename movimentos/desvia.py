#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from .movimentosBasicos import *
from .seguidorLinha import*


def ler_linha():

    s1 = sensor_corDr.reflection()
    s2 = sensor_corEs.reflection()
    s3 = sensor_corEs_Multi.reflection()
    s4 = sensorDr_Multi.reflection()

    return s1, s2, s3, s4


def encontrou_linha():

    s1, s2, s3, s4 = ler_linha()

    if s1 < LIMIAR:
        return True

    if s2 < LIMIAR:
        return True

    if s3 < LIMIAR:
        return True

    if s4 < LIMIAR:
        return True

    return False


def desviar():

    # -----------------------------------------------------
    # 1. PARA AO DETECTAR O OBSTÁCULO
    # -----------------------------------------------------

    parar()
    wait(1000)


    # -----------------------------------------------------
    # 2. SAI DA LINHA PARA A DIREITA
    # -----------------------------------------------------
    # OBS:
    # No seu robô, virarEsquerda() faz o movimento
    # físico para a DIREITA.
    # -----------------------------------------------------

    virarEsquerda()
    wait(4700)

    parar()
    wait(1000)


    # -----------------------------------------------------
    # 3. ANDA AO LADO DO OBSTÁCULO
    # -----------------------------------------------------

    andar(200)
    wait(2500)

    parar()
    wait(1000)
    
    virarDireita()
    wait(4000)
    parar()
    wait(1000)

    andar(200)
    wait(3000)

    # Continua andando enquanto o sensor lateral
    # estiver detectando o obstáculo

    while sensor_distanciaLateraEsquerda.distance() < 35:
       wait(50)


    # -----------------------------------------------------
    # 4. GARANTE QUE PASSOU COMPLETAMENTE DO OBSTÁCULO
    # -----------------------------------------------------

    andar(200)
    wait(1000)

    parar()
    wait(500)


    # -----------------------------------------------------
    # 5. VOLTA EM DIREÇÃO À LINHA
    # -----------------------------------------------------
    # OBS:
    # No seu robô, virarDireita() faz o movimento
    # físico para a ESQUERDA.
    # -----------------------------------------------------

    virarDireita()
    wait(4000)

    parar()
    wait(500)


    # -----------------------------------------------------
    # 6. PROCURA A LINHA
    # --------------gg---------------------------------------



    

    
    andar(200)
    wait(800)



    virarEsquerda()
    wait(200)

    lado = detectar_curva_90()

    if lado =="esquerda":
        virar_90("direita")


    
    parar()
    wait(500)

    ev3.screen.clear()
    ev3.screen.print("LINHA ENCONTRADA!")

    wait(1000)


def detectar_curva_90():

    """
    Lê TODOS os sensores de reflexão usados na detecção
    da curva.

    Sensores:
        sensor_corEs_Multi
        sensorDr_Multi

    Retorna:
        'direita'
        'esquerda'
        'ambos'
        None
    """

    # --------------------------------------------------------
    # LEITURA DOS SENSORES DE REFLEXÃO
    # --------------------------------------------------------

    reflexao_esquerda = sensor_corEs_Multi.reflection()
    reflexao_direita = sensorDr_Multi.reflection()


    # --------------------------------------------------------
    # Ambos detectaram a condição de curva
    # --------------------------------------------------------

    if (reflexao_esquerda < LIMIAR and
            reflexao_direita < LIMIAR):

        return 'ambos'


    # --------------------------------------------------------
    # Sensor esquerdo detectou
    # --------------------------------------------------------

    if reflexao_esquerda < LIMIAR:

        return 'esquerda'


    # --------------------------------------------------------
    # Sensor direito detectou
    # --------------------------------------------------------

    if reflexao_direita < LIMIAR:

        return 'direita'


    return None
    