#!/usr/bin/env pybricks-micropython

from pybricks.tools import wait

TEMPO_ESPERA = 30 # tempo em ticks para evitar detecção repetida do verde

verde_estado = None # estado atual do verde
verde_lado = None # lado do verde
verde_contagem = 0 # contagem de verdes
verde_tempoEspera = 0 # inteiro para o tempo de espera


def verde_rgb(sensor, r_max=10, g_min=10, b_max=30):
    """CALIBRAR r_max, g_min, b_max de acordo com a luminosidade."""
    try:
        r, g, b = sensor.rgb()
        return (r < r_max) and (g > g_min) and (b < b_max)
    except Exception:
        return False


def detectou_verde(sensor_es, sensor_dr):
    esq = verde_rgb(sensor_es)
    dir_ = verde_rgb(sensor_dr)

    if esq and dir_:
        return True, "AMBOS"
    if esq:
        return True, "ESQUERDA"
    if dir_:
        return True, "DIREITA"
    return False, None


def verificar_verde(sensor_es, sensor_dr, ev3):
    """Chamar no loop principal, antes de seguirLinha()."""
    """Estas variavies são globais para serem usadas no futuro"""
    global verde_estado, verde_lado, verde_contagem, verde_tempoEspera

    if verde_tempoEspera > 0:
        verde_tempoEspera -= 1
        return False

    detectou, lado = detectou_verde(sensor_es, sensor_dr)
    if not detectou:
        return False

    verde_lado = lado
    verde_tempoEspera = TEMPO_ESPERA

    if lado == "AMBOS":
        verde_contagem = 2
        verde_estado = "DOIS_VERDES"
        ev3.speaker.beep(1000, 100)
        wait(100)
        ev3.speaker.beep(1000, 100)
        ev3.screen.clear()
        ev3.screen.print("BECO SEM SAIDA")
        return True

    verde_contagem += 1

    if verde_contagem == 1:
        verde_estado = "UM_VERDE"
        ev3.speaker.beep(1000, 100)
        ev3.screen.clear()
        ev3.screen.print("VERDE #1")
        ev3.screen.print("Lado: " + lado)

    elif verde_contagem == 2:
        verde_estado = "DOIS_VERDES"
        ev3.speaker.beep(1000, 100)
        wait(100)
        ev3.speaker.beep(1000, 100)
        ev3.screen.clear()
        ev3.screen.print("BECO SEM SAIDA")

    return True


