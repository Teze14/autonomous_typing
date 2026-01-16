# config.py

# --- CONFIGURACIÓN DE TECLAS ---
# Coordenadas relativas (en metros) desde el CENTRO detectado del teclado (0,0)
# Eje X: Horizontal en el teclado (Izquierda - / Derecha +)
# Eje Y: Vertical en el teclado (Abajo - / Arriba +)
# Tú llenarás esto midiendo tu teclado real.
KEY_MAP = {
    'A': (0.05, 0.02),  # Ejemplo: 5cm a la derecha, 2cm arriba del centro
    'B': (-0.03, -0.01),
    'C': (0.00, 0.00),
    # ... Agrega todas las letras y números necesarios
}

# --- CONFIGURACIÓN DEL BRAZO ---
# Distancias para la maniobra (en metros)
# Asumiendo que el eje Z del brazo apunta hacia el teclado (profundidad)
APPROACH_DISTANCE = 0.05  # Distancia de seguridad antes de teclear (5cm)
PRESS_DEPTH = 0.015       # Cuánto avanza para presionar la tecla (1.5cm)

# --- ORIENTACIÓN DEL GRIPPER ---
# Valores fijos que tu nodo de C++ necesita (Roll y Pitch en grados)
# Ajusta esto para que el dedo quede perpendicular al teclado vertical.
GRIPPER_ROLL = 0.0
GRIPPER_PITCH = 0.0  # Si el teclado es vertical, tal vez necesites 0 o 90 dependiendo de tu TF.