#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from .movimentosBasicos import *


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
    wait(3500)

    parar()
    wait(1000)


    # -----------------------------------------------------
    # 3. ANDA AO LADO DO OBSTÁCULO
    # -----------------------------------------------------

    andar(200)

    wait(1500)


    # Continua andando enquanto o sensor lateral
    # estiver detectando o obstáculo

    while sensor_distanciaLateral.distance() < 35:
        wait(50)


    # -----------------------------------------------------
    # 4. GARANTE QUE PASSOU COMPLETAMENTE DO OBSTÁCULO
    # -----------------------------------------------------

    andar(200)
    wait(2000)

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
    wait(3800)

    parar()
    wait(500)


    # -----------------------------------------------------
    # 6. PROCURA A LINHA
    # --------------gg---------------------------------------

    andar(200)
    wait(4000)

    virarDireita()
    wait(3500)

    andar(200)
    wait(2000)

    while not encontrou_linha():
        wait(10)


    # -----------------------------------------------------
    # 7. ENCONTROU A LINHA
    # -----------------------------------------------------

    parar()
    wait(500)

    ev3.screen.clear()
    ev3.screen.print("LINHA ENCONTRADA!")

    wait(1000)