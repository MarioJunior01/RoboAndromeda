#!/usr/bin/env pybricks-micropython

from calibracao.hsv import*
from movimentos.movimentosBasicos import *

TEMPO_ESPERA = 30 # tempo em ticks para evitar detecção repetida do verde

verde_estado = None # estado atual do verde
verde_lado = None # lado do verde
verde_contagem = 0 # contagem de verdes
verde_tempoEspera = 0 # inteiro para o tempo de espera




def detectou_verde():


    cor_esq, cor_dir =  verificar_cor()

    print("ESQ:", cor_esq)
    print("DIR:", cor_dir)

    ev3.screen.clear()
    ev3.screen.print("ESQ: " + cor_esq)
    ev3.screen.print("DIR: " + cor_dir)

    esq = (cor_esq == "VERDE")
    dir_ = (cor_dir == "VERDE")

    if esq and dir_:
        return True, "AMBOS"

    elif esq:
        return True, "ESQUERDA"

    elif dir_:
        return True, "DIREITA"

    return False, None

def verificar_verde():

    global verde_estado
    global verde_lado
    global verde_contagem
    global verde_tempoEspera

    if verde_tempoEspera > 0:
        verde_tempoEspera -= 1
        return False

    detectou, lado = detectou_verde()

    if not detectou:
        return False

    verde_lado = lado
    verde_tempoEspera = TEMPO_ESPERA

 

    if lado == "AMBOS":

        verde_estado = "DOIS_VERDES"
        verde_contagem = 2

        ev3.speaker.beep(1000, 100)
        wait(100)
        ev3.speaker.beep(1000, 100)

        ev3.screen.clear()
        ev3.screen.print("BECO SEM SAIDA")
        virar_360()
        wait(1000)

        return True


    if lado == "ESQUERDA":

        verde_estado = "VERDE_ESQUERDA"

        ev3.speaker.beep(400, 100)

        ev3.screen.clear()
        ev3.screen.print("VERDE")
        ev3.screen.print("ESQUERDA")
        virarDireita()
        wait(1500)

        return True



    if lado == "DIREITA":

        verde_estado = "VERDE_DIREITA"

        ev3.speaker.beep(400, 100)

        ev3.screen.clear()
        ev3.screen.print("VERDE")
        ev3.screen.print("DIREITA")
        virarEsquerda()
        wait(1500)

        return True
    
    

    return False


def verificar_vermelho(ev3):

    cor_esq, cor_dir = verificar_cor()

    print("ESQ:", cor_esq)
    print("DIR:", cor_dir)

    esq = (cor_esq == "VERMELHO")
    dir_ = (cor_dir == "VERMELHO")

    if esq or dir_:

        ev3.screen.clear()
        ev3.screen.print("VERMELHO!")

        if esq and dir_:

            ev3.screen.print("AMBOS")

        elif esq:

            ev3.screen.print("ESQUERDA")

        else:

            ev3.screen.print("DIREITA")

        return True

    return False




