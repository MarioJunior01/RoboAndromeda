#!/usr/bin/env pybricks-micropython

from calibracao.constantes import *
from movimentos.verde import *
from movimentos.seguidorLinha import seguidor_linha
from movimentos.desvia import desviar
from menus.principal import menu_principal
from menus.principal import *
from movimentos.movimentosBasicos import *

def programa_principal():

    ev3.screen.clear()
    ev3.screen.print("Executando:")
    ev3.screen.print("Principal")

    wait(1000)

    while True:

        if Button.CENTER in ev3.buttons.pressed():

            while ev3.buttons.pressed():
                wait(10)

            parar()
            ev3.screen.clear()
            ev3.screen.print("Voltando ao menu...")
            wait(500)
            return


        distancia = sensor_distanciaFrente.distance()

        if distancia < DISTANCIA_OBJETO:

            desviar()
    
        if verificar_vermelho(ev3):

            parar()
            ev3.speaker.beep(400)

            wait(5000)

            return

        if verificar_verde():

            continue

        seguidor_linha()

        wait(20)

def main():

    menu_principal(programa_principal)


if __name__ == "__main__":

    main()