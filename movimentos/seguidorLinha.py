#!/usr/bin/env pybricks-micropython

from calibracao.constantes import*
from movimentos.movimentosBasicos import *
from calibracao.hsv import*
def seguidor_linha():
    print("--- INICIANDO SEGUIDOR PID (HSV) ---")
    
    # 1. Carrega dados salvos (Útil se você precisar normalizar os sensores)
    calib = carregar_calibracao()
    
    # 2. Configurações dos Ganhos PID (Ajuste esses números na pista)
       # Amortecimento para evitar oscilação nas retas
     # Velocidade dos motores em graus por segundo (deg/s)
    
    # 3. Variáveis de controle do PID
    erro_anterior = 0.0
    integral = 0.0
        
    # 4. Loop Principal de Controle
  
        # Se pressionar o botão para baixo, encerra o seguidor de linha
            
        # Leitura RGB dos dois sensores
    r1, g1, b1 = sensor_corEs.rgb()
    r2, g2, b2 = sensor_corDr.rgb()
        
        # Conversão para HSV (focando no componente V - Brilho)
    _, _, v1 = rgb_to_hsv(r1, g1, b1)
    _, _, v2 = rgb_to_hsv(r2, g2, b2)
        
        # --- CÁLCULO DO ERRO COM DOIS SENSORES ---
        # Se ambos lerem o mesmo brilho (linha centralizada), o erro é 0.
        # Se o robô pender para um lado, a diferença gera o sinal de correção.
    erro = v1 - v2
        
        # --- ALGORITMO PID ---
    proporcional = erro * KP
        
    integral = integral + erro
        # Anti-windup: Limita o acúmulo da integral para o robô não perder o controle
    if integral > 100: integral = 100
    elif integral < -100: integral = -100
    i_output = integral * KI
        
    derivada = (erro - erro_anterior) * KD
        
        # Sinal total de correção (Turn)
    turn = proporcional + i_output + derivada
        
        # --- APLICAÇÃO NOS MOTORES ---
        # Se v1 (esquerdo) ver branco (valor alto) e v2 (direito) ver preto (valor baixo):
        # erro será positivo -> turn positivo -> motor esquerdo acelera e direito desacelera -> robô vira para a direita.
    velocidade_esquerda = VELOCIDADE_BASE + turn
    velocidade_direita = VELOCIDADE_BASE - turn
        
        # Envia o comando físico para os motores (com controle de rotação embarcado do Pybricks)
    motorEs.run(velocidade_esquerda)
    motorDr.run(velocidade_direita)
        
        # Guarda o erro atual para o próximo ciclo
    erro_anterior = erro
        
        # Taxa de amostragem controlada de 10ms (100Hz) para estabilizar o cálculo derivativo
    wait(10)