#!/usr/bin/env pybricks-micropython

from calibracao.hsv import*
from movimentos.movimentosBasicos import *

TEMPO_ESPERA = 30

verde_estado = None
verde_lado = None
verde_contagem = 0
verde_tempoEspera = 0


def detectou_verde():

    cor_esq, cor_dir = verificar_cor()

    print("ESQ:", cor_esq)
    print("DIR:", cor_dir)

    esq = (cor_esq == "VERDE")
    dir_ = (cor_dir == "VERDE")

    if esq and dir_:
        return True, "AMBOS"

    elif esq:
        return True, "ESQUERDA"

    elif dir_:
        return True, "DIREITA"

    return False, None


def verificar_linha_depois_do_verde():

    # ==========================================
    # ANDA PARA SAIR DO VERDE
    # ==========================================

    andar(100)

    wait(50)

    motorEs.stop()
    motorDr.stop()

    wait(300)

    # ==========================================
    # LÊ OS DOIS SENSORES
    # ==========================================

    cor_esq, cor_dir = verificar_cor()

    # ==========================================
    # DECIDE ONDE ESTÁ A LINHA
    # ==========================================

    if cor_esq == "PRETO" and cor_dir == "PRETO":

        return "AMBOS"

    elif cor_esq == "PRETO":

        return "PRETO_ESQUERDA"

    elif cor_dir == "PRETO":

        return "PRETO_DIREITA"

    else:

        return "DESCONHECIDO"


def verificar_verde(ev3):

    global verde_estado
    global verde_lado
    global verde_contagem
    global verde_tempoEspera

    if verde_tempoEspera > 0:
        verde_tempoEspera -= 1
        return False

    # Detecta verde
    detectou, lado = detectou_verde()

    if not detectou:
        return False

    verde_lado = lado

  


    # ==========================================
    # VERDE NOS DOIS SENSORES
    # ==========================================

    if lado == "AMBOS":

        verde_estado = "DOIS_VERDES"
        verde_contagem = 2

        ev3.speaker.beep(1000, 100)

        wait(100)

        ev3.speaker.beep(1000, 100)
        resultado = verificar_linha_depois_do_verde()
        if resultado == "AMBOS":
         ev3.screen.clear()
         ev3.screen.print("BECO SEM SAIDA")

         virar_360()

         wait(3000)



        return True


    # ==========================================
    # VERDE À ESQUERDA
    # ==========================================

    if lado == "ESQUERDA":

        verde_estado = "VERDE_ESQUERDA"

        ev3.speaker.beep(400, 100)

        ev3.screen.clear()
        ev3.screen.print("VERDE")
        ev3.screen.print("ESQUERDA")

        # Anda e verifica o que existe depois do verde
        resultado = verificar_linha_depois_do_verde()

        ev3.screen.clear()
        ev3.screen.print("ESQ: " + resultado)

    

        if resultado == "PRETO_ESQUERDA":
         andar(100)
         wait(500)
         virarEsquerda()
           


        else:

            ev3.screen.print("DESCONHECIDO")

        wait(500)

        return True


    # ==========================================
    # VERDE À DIREITA
    # ==========================================

    if lado == "DIREITA":

        verde_estado = "VERDE_DIREITA"

        ev3.speaker.beep(400, 100)

        ev3.screen.clear()
        ev3.screen.print("VERDE")
        ev3.screen.print("DIREITA")

        # Anda e verifica o que existe depois do verde
        resultado = verificar_linha_depois_do_verde()

        ev3.screen.clear()
        ev3.screen.print("DIR: " + resultado)

      


        if resultado == "PRETO_DIREITA":
            andar(100)
            wait(500)
            virarDireita()
            wait(3000)

        else:

            ev3.screen.print("DESCONHECIDO")

        wait(500)

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