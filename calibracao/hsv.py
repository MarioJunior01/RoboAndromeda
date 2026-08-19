def rgb_para_hsv(r, g, b):
    # Normaliza os valores de 0-255 para 0-1
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    
    cmax = max(r, g, b)
    cmin = min(r, g, b)
    delta = cmax - cmin
    
    # Calcula o Valor (V)
    v = cmax
    
    # Calcula a Saturação (S)
    if cmax == 0:
        s = 0
    else:
        s = delta / cmax
        
    # Calcula o Matiz (H)
    if delta == 0:
        h = 0
    elif cmax == r:
        h = ((g - b) / delta) % 6
    elif cmax == g:
        h = ((b - r) / delta) + 2
    else:
        h = ((r - g) / delta) + 4
        
    h /= 6.0
    
    return (h, s, v)

# Exemplo de uso para o tom vermelho puro (255, 0, 0)
print(rgb_para_hsv(255, 0, 0))

    

