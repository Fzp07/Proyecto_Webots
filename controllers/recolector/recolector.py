"""
Controlador Supervisor para TIAGo - Recolección de basura en Santa Marta
=======================================================================
Compatible con el TIAGo (PAL Robotics) de Webots.
"""

from controller import Supervisor
import math
import sys

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# ============================================================
# SENSORES DEL SUPERVISOR
# ============================================================
lidar = robot.getDevice("Hokuyo URG-04LX-UG01")
if lidar is not None:
    lidar.enable(timestep)
    print("Lidar encontrado. Activando evasión de obstáculos.")
else:
    print("Advertencia: No se encontró Lidar. El robot no evitará obstáculos.")

# ============================================================
# MOTORES DE RUEDAS (TIAGo: diferencial 2 ruedas)
# ============================================================
rueda_izq = robot.getDevice("wheel_left_joint")
rueda_der = robot.getDevice("wheel_right_joint")

if None in (rueda_izq, rueda_der):
    print("ERROR: No se encontraron los motores de las ruedas.")
    print("Nombres esperados: wheel_left_joint, wheel_right_joint")
    sys.exit(1)

rueda_izq.setPosition(float('inf'))
rueda_der.setPosition(float('inf'))
rueda_izq.setVelocity(0.0)
rueda_der.setVelocity(0.0)

# ============================================================
# BRAZO ROBÓTICO (7 articulaciones)
# ============================================================
motores_brazo = []
for i in range(1, 8):
    motor = robot.getDevice(f"arm_{i}_joint")
    if motor is None:
        print(f"ERROR: No se encontró el motor arm_{i}_joint")
        sys.exit(1)
    motor.setVelocity(1.76)
    motores_brazo.append(motor)

# ============================================================
# PINZA (TIAGo gripper)
# ============================================================
pinza_izq = robot.getDevice("gripper_left_finger_joint")
pinza_der = robot.getDevice("gripper_right_finger_joint")
if pinza_izq is None or pinza_der is None:
    print("ERROR: No se encontraron los motores de la pinza")
    sys.exit(1)
pinza_izq.setVelocity(0.05)
pinza_der.setVelocity(0.05)

# ============================================================
# PARÁMETROS
# ============================================================
MAX_VEL = 8.0
OBSTACLE_THRESHOLD = 0.6
TOLERANCIA_POS = 0.3
ANGULO_UMBRAL = 0.08

# Ángulos del brazo para TIAGo (7-DOF)
# Límites: arm1[0.07,2.68] arm2[-1.5,1.02] arm3[-3.46,1.5] arm4[-0.32,2.29] arm5[-2.07,2.07] arm6[-1.39,1.39] arm7[-2.07,2.07]
# BRAZO_RECOGER: brazo extendido hacia adelante y abajo para recoger del suelo
BRAZO_RECOGER = [0.07, -1.0, 0.0, 1.5, 0.0, -1.0, 0.0]
# BRAZO_LLEVAR: plegado para transportar
BRAZO_LLEVAR = [0.07, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

# ============================================================
# FUNCIONES AUXILIARES
# ============================================================
def get_basura_list():
    basuras = []
    i = 1
    while True:
        nodo = robot.getFromDef(f"basura_{i}")
        if nodo is None:
            break
        basuras.append(nodo)
        i += 1
    return basuras

def get_robot_position():
    node = robot.getSelf()
    pos = node.getPosition()
    return pos[0], pos[1], pos[2]

def get_robot_heading():
    node = robot.getSelf()
    orientation = node.getOrientation()
    yaw = math.atan2(orientation[3], orientation[0])
    return yaw

def set_velocidad(v_izq, v_der):
    """Aplica velocidades a las ruedas (control diferencial)."""
    rueda_izq.setVelocity(v_izq)
    rueda_der.setVelocity(v_der)

def normalizar_angulo(angulo):
    while angulo > math.pi:
        angulo -= 2 * math.pi
    while angulo < -math.pi:
        angulo += 2 * math.pi
    return angulo

def girar_a_angulo(angulo_deseado):
    """Gira el robot hasta alcanzar la orientación deseada."""
    set_velocidad(0, 0)
    robot.step(timestep)
    
    angulo_actual = get_robot_heading()
    error = normalizar_angulo(angulo_deseado - angulo_actual)
    if abs(error) < ANGULO_UMBRAL:
        return
    
    pasos_max = 800
    pasos = 0
    while robot.step(timestep) != -1:
        angulo_actual = get_robot_heading()
        error = normalizar_angulo(angulo_deseado - angulo_actual)
        
        pasos += 1
        if pasos > pasos_max:
            print("   ⚠ Tiempo de giro agotado")
            set_velocidad(0, 0)
            robot.step(timestep)
            break
        
        if abs(error) < ANGULO_UMBRAL:
            set_velocidad(0, 0)
            robot.step(timestep)
            break
        
        # Giro en el lugar: un lado avanza, el otro retrocede
        vel_giro = 3.0 * error
        vel_giro = max(min(vel_giro, MAX_VEL), -MAX_VEL)
        set_velocidad(-vel_giro, vel_giro)

def mover_recto(distancia, vel=MAX_VEL):
    """Avanza en línea recta la distancia indicada (metros)."""
    x0, y0, _ = get_robot_position()
    recorrido = 0
    while robot.step(timestep) != -1:
        x, y, _ = get_robot_position()
        recorrido = math.hypot(x - x0, y - y0)
        if recorrido >= distancia:
            set_velocidad(0, 0)
            robot.step(timestep)
            break
        set_velocidad(vel, vel)

def mover_a_punto(x_dest, y_dest):
    """Navega hacia el destino: orientarse y luego avanzar recto."""
    dx = x_dest - get_robot_position()[0]
    dy = y_dest - get_robot_position()[1]
    distancia = math.hypot(dx, dy)
    angulo_objetivo = math.atan2(dy, dx)
    girar_a_angulo(angulo_objetivo)
    mover_recto(distancia)

def mover_brazo(angulos, timeout=100):
    """Mueve el brazo a los ángulos dados y espera el tiempo necesario."""
    for motor, ang in zip(motores_brazo, angulos):
        motor.setPosition(ang)
    
    for _ in range(timeout):
        robot.step(timestep)

def abrir_pinza():
    pinza_izq.setPosition(0.03)
    pinza_der.setPosition(0.03)
    for _ in range(20):
        robot.step(timestep)

def cerrar_pinza():
    pinza_izq.setPosition(0.0)
    pinza_der.setPosition(0.0)
    for _ in range(20):
        robot.step(timestep)

def recoger_objeto():
    mover_brazo(BRAZO_RECOGER, 100)
    cerrar_pinza()
    mover_brazo(BRAZO_LLEVAR, 100)

def soltar_objeto():
    mover_brazo(BRAZO_RECOGER, 100)
    abrir_pinza()
    mover_brazo(BRAZO_LLEVAR, 100)

# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================
print("=" * 60)
print("MISIÓN DE RECOLECCIÓN DE BASURA - SANTA MARTA (CORREGIDO)")
print("=" * 60)

contenedor = robot.getFromDef("contenedor")
if contenedor is None:
    print("ERROR CRÍTICO: No se encontró el objeto 'contenedor'.")
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    sys.exit(1)

pos_contenedor = contenedor.getPosition()
bx, by = pos_contenedor[0], pos_contenedor[1]
print(f"Contenedor en: ({bx:.2f}, {by:.2f})")

basuras_recolectadas = 0
depositadas = set()

while robot.step(timestep) != -1:
    basuras = [b for b in get_basura_list() if b.getDef() not in depositadas]
    if not basuras:
        print("\n¡MISIÓN COMPLETADA!")
        print(f"Total basuras recolectadas: {basuras_recolectadas}")
        print("Santa Marta está limpia. 🎉")
        break
    
    print(f"\nBasuras restantes: {len(basuras)}")
    
    # Encontrar la basura más cercana
    rx, ry, _ = get_robot_position()
    mejor_distancia = float('inf')
    objetivo = None
    for b in basuras:
        pos_b = b.getPosition()
        dist = math.hypot(rx - pos_b[0], ry - pos_b[1])
        if dist < mejor_distancia:
            mejor_distancia = dist
            objetivo = b
    
    if objetivo is None:
        print("No se pudo seleccionar objetivo, reintentando...")
        continue
    
    ox, oy = objetivo.getPosition()[0], objetivo.getPosition()[1]
    print(f"➡️  Basura en ({ox:.2f}, {oy:.2f}) - Distancia: {mejor_distancia:.2f}m")
    
    # Navegar hacia la basura
    print("   Navegando...")
    mover_a_punto(ox, oy)
    
    # Recoger
    print("   Recogiendo...")
    recoger_objeto()
    
    # Ocultar la basura (subirla para que parezca recogida)
    if objetivo is not None:
        campo_pos = objetivo.getField("translation")
        if campo_pos is not None:
            campo_pos.setSFVec3f([rx, ry + 10, 10])
        for _ in range(10):
            robot.step(timestep)
    basuras_recolectadas += 1
    print(f"   ✓ Recogida. Total: {basuras_recolectadas}")
    
    # Llevar al contenedor
    print("   Llevando al contenedor...")
    mover_a_punto(bx, by)
    
    # Depositar: colocar la basura dentro del contenedor
    print("   Depositando...")
    campo_pos = objetivo.getField("translation")
    if campo_pos is not None:
        campo_pos.setSFVec3f([bx, by, 0.1])
    for _ in range(5):
        robot.step(timestep)
    soltar_objeto()
    depositadas.add(objetivo.getDef())
    print("   ✓ Depositada en el contenedor")

print("\nPrograma finalizado.")
robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)