#!/usr/bin/env pybricks-micropython
from calibracao.constantes import*
from .movimentosBasicos import*

def desvia():
    parar()
    wait(000)
    virarEsquerda(vel=200)
    andar()
    wait(800)
    ev3.speaker.beep(800)

    # 3. alinhar lado
    virarDireita(vel=200)

    # 4. andar um tempo FIXO (evita bug)
    andar()
    wait(1200)

    # 5. usar sensor lateral SÓ PRA GARANTIR que passou
   

    
    andar(250)
    wait(10)

    parar()
    wait(300)

    # 6. voltar pra linha
    virarDireita(200)

    # 7. procurar linha
    parar()
    wait(300)

    # 8. alinhar
    virarEsquerda()
    ev3.speaker.beep(600)
