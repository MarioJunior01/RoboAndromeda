def andar(vel=velocidade):
    wait(1000)
    motorDr.run(vel)
    motorEs.run(vel)

def parar():
    wait(1000)
    motorDr.stop()
    motorEs.stop()
    wait(200)
def curvaSuaveDireita(vel=velocidade):
    wait(1000)
    motorDr.run(vel)
    motorEs.run(-vel*0.02)
def curvaSuaveEsquerda(vel=velocidade):
    wait(1000)
    motorDr.run(-vel*0.02)
    motorEs.run(vel)

def virarDireita(vel=velocidade_curva):
    wait(1000)
    motorEs.run(-vel)
    motorDr.run(vel)

def virarEsquerda(vel=velocidade_curva):
    wait(1000)
    motorEs.run(vel)
    motorDr.run(-vel)

def re():
    wait(1000)
    motorDr.run(-velocidade_curva)
    motorEs.run(-velocidade_curva)


