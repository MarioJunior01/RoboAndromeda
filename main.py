#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from movimentos.verde import verificar_verde
from movimentos.seguidorLinha import seguidor_linha
from movimentos.desvia import desvia
from menus.principal import menu_principal
from menus.principal import *
from movimentos.movimentosBasicos import *

def programa_principal():

    ev3.screen.clear()
    ev3.screen.print("Executando:")
    ev3.screen.print("Principal")

    wait(1000)

    while True:

        # ==============================
        # BOTÃO CENTRAL
        # ==============================

        if Button.CENTER in ev3.buttons.pressed():

            while ev3.buttons.pressed():
                wait(10)

            parar()

            ev3.screen.clear()
            ev3.screen.print("Voltando ao menu...")

            wait(500)

            return


        # ==============================
        # 1 - OBSTÁCULO
        # ==============================

        distancia = sensor_distanciaFrente.distance()

        if distancia < DISTANCIA_OBJETO:

            desvia()

            continue



        if verificar_vermelho(ev3):

            parar()

            ev3.screen.clear()
            ev3.screen.print("VERMELHO!")
            ev3.screen.print("FIM")
            ev3.speaker.beep(400)

            wait(5000)

            return


        if verificar_verde(ev3):

            continue


        seguidor_linha()

        wait(20)

def main():

    menu_principal(programa_principal)


if __name__ == "__main__":

    main()