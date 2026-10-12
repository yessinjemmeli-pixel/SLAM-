import math

# Convention des roues :
# On fait toujours FL, FR, BR, BL (dans le sens des aiguilles d'une montre)
#   FL = Front-Left  (avant-gauche)
#   FR = Front-Right (avant-droite)
#   BR = Back-Right  (arrière-droite)
#   BL = Back-Left   (arrière-gauche)

# -- Console --
DEFAULT_RPM = 150.0  # RPM par défaut envoyé aux roues
HELP_MESSAGE = """
Commands :
- automatic: Sends the robot to a target position. Can be used like so :
    >> automatic <x_target> <y_target>
- manual: Takes manual control of the robot, allowing you to send direct speed commands to the wheels
    >> manual
- setrpm : Sets the RPMS of the wheels to the given speeds. Can be used like so :
    >> setrpm <front_left_wheel_rpm> <front_right_wheel_rpm> <back_right_wheel_rpm> <back_left_wheel_rpm> 
    >> setrpm <all_wheels_rpm>
    >> setrpm
    This last one is setting all wheels' rpms to the default RPM value (150)
- setpwm : Sets the raw PWM sent to the wheels to a certain value between -255 and 255.
    >> setpwm <front_left_wheel_pwm> <front_right_wheel_pwm> <back_right_wheel_pwm> <back_left_wheel_pwm> 
    >> setpwm <all_wheels_pwm>
    >> setpwm
- stop: equivalent to setrpm 0
- setkp, setki, setkd: sets a coefficient for the servoing (asservissement) of a given wheel. Can be used like so :
    >> setkp frontleft 0.3
    >> setkd backright 2.3
    >> setkp 2.5 (sets kp to 2.5 for all wheels)  
- setki, setkd work the same way
- printpid: prints the current PID coefficients for all wheels
    >> printpid
- help: prints this message                  
"""

# Roues
FRONT_LEFT_FLIPPED = False # Est ce qu'on doit flip la direction des moteurs pour cette roue ?
FRONT_RIGHT_FLIPPED = True
BACK_RIGHT_FLIPPED = True
BACK_LEFT_FLIPPED = False

# Asservissement (valeurs du PID)
FL_KP = 2.33
FL_KI = 7.6
FL_KD = 0
FR_KP = 2.15
FR_KI = 6.9
FR_KD = 0
BR_KP = 2.54
BR_KI = 7.3
BR_KD = 0
BL_KP = 2.37
BL_KI = 7.74
BL_KD = 0
MAX_ACCUMULATED_ERROR = 1000.0

# Timers Periods
WHEEL_UPDATING_PERIOD = 0.02 # Période du timer qui envoie à l'arduino les nouvelles vitesses des roues
RPMS_UPDATING_PERIOD = 0.1 # Période de l'update des RPMs par les encodeurs

# Serial
BAUDRATE = 115200
ARDUINO_CONNECT_PERIOD = 1  #time between each attempt to connect to the arduino
#Just for your information, the lidar baudrate is 460800

# -- Automatic --
K_ATT                 = 8.0            # Gain du champ attractif #1.5
K_REP                 = 0.8            # Gain du champ répulsif
APF_D0                = 2.0            # Rayon d'influence des obstacles (m) — doit correspondre à LIDAR_OBSTACLE_RANGE
APF_MIN_DIST          = 0.05           # Distance plancher pour éviter la division par zéro (m)
GOAL_RADIUS           = 0.2            # Distance en dessous de laquelle la cible est considérée atteinte (m)
DEFAULT_GOAL_X        = 1.0            # Position cible par défaut en x (m)
DEFAULT_GOAL_Y        = 0.0            # Position cible par défaut en y (m)
APF_MAX_LINEAR_FORCE  = 3.0            # Force maximale appliquée par l'APF convertie en vitesse
APF_MAX_ANGULAR_FORCE = 2.0            # Force angulaire max
AUTO_LOOP_HZ          = 10.0           # Fréquence de la boucle de controle automatique (Hz)
APF_HEADING_GAIN      = 0.2            # Gain pour l'alignement angulaire APF
APF_HEADING_TOLERANCE = math.pi / 4.0  # Angle à partir duquel le robot réduit sa vitesse linéaire pour privilégier la rotation et éviter les zigzags

# -- Localization (ICP et Cartographie) --
LOC_DIST_THRESHOLD        = 0.3               # Distance mini pour maj de la carte (m)
LOC_ANGLE_THRESHOLD       = math.radians(15)  # Angle mini pour maj de la carte (rad)
ICP_MAX_CORRESPOND_DIST   = 1.5               # Distance max pour qu'une correspondance soit valide (m)
ICP_MAX_JUMP_DIST         = 0.3              # Tolérance de saut de distance brutale ICP (m)
ICP_MAX_JUMP_ANGLE        = 0.40              # Tolérance de saut d'angle brutale ICP (rad)
ICP_VOXEL_SIZE            = 0.05              # Taille du voxel pour le filtrage du nuage de points (m)
ICP_FITNESS_THRESHOLD     = 0.7               # Score min pour accepter la correction (0.0 à 1.0)
ICP_INLIER_RMSE_THRESHOLD = 0.15              # Erreur de distance max acceptée (m)
ICP_MAP_UPDATE_FITNESS    = 0.85              # Score min de fitness pour autoriser l'ajout à la carte
ICP_MIN_POINTS            = 30                # Nombre minimum de points requis pour lancer l'algo ICP
ENCODER_NOISE_THRESHOLD = 0.5


# -- Paramètres généraux du robot --
WHEEL_RADIUS = 0.0325  # Rayon des roues en mètres (3.25 cm)
ROBOT_RADIUS = 0.15  # Rayon approximatif du robot pour l'APF (m)
LX           = 0.10  # Distance centre -> axe avant/arrière (m)
LY           = 0.10  # Distance centre -> axe gauche/droite (m)
GEAR_RATIO   = 1.0   # Rapport de réduction (1.0 = RPM moteur == RPM roue)


# -- Paramètres des actionneurs --
MAX_RPM = 160.0  # RPM maximum envoyé aux roues (même échelle que la console)
MIN_RPM = 130.0  # RPM minimum envoyé aux roues


# -- Paramètres du Lidar --
LIDAR_MAX_RANGE_M    = 6.0   # Portée maximale du lidar (m)
LIDAR_N_RAYS         = 360   # Nombre de rayons / Simulation
LIDAR_MIN_DIST       = 0.05  # Distance plancher pour ignorer les points trop proches (m)
LIDAR_OBSTACLE_RANGE = 2.0   # Distance en dessous de laquelle un point est un "obstacle" (m)

# -- Noeud de Navigation Automatic (Navigation DWA) --

# Limites cinématiques du robot
DWA_MAX_VX = 0.5  # m/s   vitesse longitudinale max
DWA_MAX_VY = 0.5  # m/s   vitesse latérale max car Robot holonome
DWA_MAX_WZ = 1.0  # rad/s vitesse de rotation max
DWA_MAX_ACC_V = 1.0  # m/s²  accélération linéaire max
DWA_MAX_ACC_W = 2.0  # rad/s² accélération angulaire max

# Paramètres d'échantillonnage de l'espace des vitesses
DWA_V_SAMPLES = 5  # nombre d'échantillons sur vx (et vy)
DWA_W_SAMPLES = 7  # nombre d'échantillons sur wz

# Paramètres de prédiction
DWA_SIM_TIME = 1.5  # horizon de prédiction en s
DWA_SIM_STEPS = 15  # discrétisation de la trajectoire

# Gestion des collsions et marges de sécurité
DWA_ROBOT_RADIUS = 0.15  # m     rayon du robot (clearance)
DWA_OBSTACLE_MARGIN = 0.05  # m     marge de sécurité supplémentaire
DWA_LETHAL_DIST = 0.0  # m     distance en dessous de laquelle on bloque

# Poids de la fonction de coût
DWA_W_HEADING = 0.2  # poids cap vers le but
DWA_W_CLEARANCE = 0.1  # poids distance aux obstacles
DWA_W_VELOCITY = 0.1  # poids vitesse (favorise les trajectoires rapides)
DWA_W_GOAL_DIST = 0.2  # poids distance euclidienne au but

# Seuils d'arrivée
NAV_GOAL_DIST_TOL = 0.15  # m     on considère le but atteint
NAV_GOAL_ANGLE_TOL = 0.2  # rad   tolérance angulaire

# Paramètres de robustesse et mode de Recovery
OBSTACLE_MEMORY_SEC = 0.5  # durée de vie d'un obstacle mémorisé (s)
LIDAR_STALE_SEC = 0.3  # timeout de sécurité donnée lidar (s)
STUCK_COUNT_THRESH = 5  # seuil de déclenchement de la séquence de dégagement

# Récupération : recul puis rotation, durées et vitesses fixes
RECOVERY_BACKUP_S = 0.4  # s      durée du recul
RECOVERY_ROTATE_S = 0.3  # s      durée de la rotation
RECOVERY_VX = -0.15  # m/s    vitesse de recul (négatif = reculer)
RECOVERY_WZ = 0.6  # rad/s  vitesse de rotation (le signe change pour ne pas faire deux fois la meme chose)


def angle_wrap(angle: float) -> float:
    """Ramène un angle dans [-pi, pi]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle
