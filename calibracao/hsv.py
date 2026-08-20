#!/usr/bin/env pybricks-micropython

from .constantes import*

def rgb_to_hsv(r, g, b):
    r = r / 100
    g = g / 100
    b = b / 100

    maior = max(r, g, b)
    menor = min(r, g, b)

    delta = maior - menor

    v = maior * 100

    if maior == 0:
        s = 0
    else:
        s = (delta / maior) * 100

   
    if delta == 0:
        h = 0

    elif maior == r:
        h = 60 * (((g - b) / delta) % 6)

    elif maior == g:
        h = 60 * (((b - r) / delta) + 2)

    else:
        h = 60 * (((r - g) / delta) + 4)

    return h, s, v


def executar_hsv():
    # Removido o import de os para evitar conflitos no MicroPython

    print("--- MODO CALIBRACAO ---")
    print("Posicione os sensores e aperte o BOTAO CENTRAL para salvar.")
    print("Ou aperte o BOTAO PARA BAIXO para sair sem salvar.")
    print("-----------------------------------------------------")

    # Caminho absoluto direto em formato de string pura
    caminho_arquivo = "/home/robot/RoboAndromeda/calibracao/calibracao.txt"

    while True:
        # Mantém a sua lógica original para sair com o botão para baixo
        if Button.DOWN in ev3.buttons.pressed():
            print("Saindo sem salvar.")
            break
            
        r1, g1, b1 = sensor_corEs.rgb()
        r2, g2, b2 = sensor_corDr.rgb()
       
        h1, s1, v1 = rgb_to_hsv(r1, g1, b1)
        h2, s2, v2 = rgb_to_hsv(r2, g2, b2)
         
        print("Sensor esquerdo")
        print("RGB: r =", r1, "g =", g1, "b =", b1)
        print("HSV: h =", int(h1), "s =", int(s1), "v =", int(v1))

        print("--------------------")

        print("Sensor direito")
        print("RGB: r =", r2, "g =", g2, "b =", b2)
        print("HSV: h =", int(h2), "s =", int(s2), "v =", v2)
        
        # --- LÓGICA DE GRAVAÇÃO DIRETA E COMPATÍVEL ---
        if Button.CENTER in ev3.buttons.pressed():
            try:
                # O comando 'with' abre, escreve e fecha o arquivo de forma segura
                with open(caminho_arquivo, "w") as arquivo:
                    arquivo.write("h1={},s1={},v1={},h2={},s2={},v3={}".format(
                        int(h1), int(s1), int(v1), 
                        int(h2), int(s2), int(v2)
                    ))
                
                ev3.speaker.beep() # Sinal sonoro de sucesso
                print("\n[SUCESSO] Constantes salvas em: " + caminho_arquivo)
                
                wait(500) # Evita leituras duplicadas do botão ao sair
                break # Encerra o programa após salvar
            except Exception as e:
                print("Erro ao salvar arquivo:", e)

        wait(500)
  
    

