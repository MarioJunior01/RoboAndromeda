#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

ev3 = EV3Brick()
sensor_corEs = ColorSensor(Port.S3)
sensor_corDr = ColorSensor(Port.S4)

ev3.screen.print("Calibracao Verde")
ev3.screen.print("Posicione sobre")
ev3.screen.print("o verde e veja")
ev3.screen.print("os valores RGB")
wait(2000)

while True:
    r_es, g_es, b_es = sensor_corEs.rgb()
    r_dr, g_dr, b_dr = sensor_corDr.rgb()

    ev3.screen.clear()
    ev3.screen.print("=== ESQUERDA ===")
    ev3.screen.print("R:", int(r_es), "G:", int(g_es), "B:", int(b_es))
    ev3.screen.print("=== DIREITA ===")
    ev3.screen.print("R:", int(r_dr), "G:", int(g_dr), "B:", int(b_dr))

    wait(500)
