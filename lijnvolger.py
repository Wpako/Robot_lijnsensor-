# lijnvolger.py
# Sensoruitlezing + drempel

from pyfirmata import Arduino, util
from time import sleep

POORT = 'COM5'

# sensor-kanalen A0..A4 = [L1, L2, M, R2, R1]
SENSOR = [0, 1, 2, 3, 4]
DREMPEL = 0.46         # < drempel = zwart (1)

# motor
PWM_L, PWM_R = 11, 3
IN1_L, IN2_L = 12, 13
IN3_R, IN4_R = 9, 10
VOORUIT_L = (0, 1); VOORUIT_R = (1, 0)

board = Arduino(POORT)
it = util.Iterator(board); it.start(); sleep(0.3)

ain = [board.get_pin(f'a:{ch}:i') for ch in SENSOR]
pL = board.get_pin(f'd:{PWM_L}:p'); pR = board.get_pin(f'd:{PWM_R}:p')
L1 = board.get_pin(f'd:{IN1_L}:o'); L2 = board.get_pin(f'd:{IN2_L}:o')
R3 = board.get_pin(f'd:{IN3_R}:o'); R4 = board.get_pin(f'd:{IN4_R}:o')

def zet_richting(pair, a, b):
    x, y = pair
    a.write(1 if x else 0); b.write(1 if y else 0)

def vooruit():
    zet_richting(VOORUIT_L, L1, L2)
    zet_richting(VOORUIT_R, R3, R4)

def stop():
    pL.write(0); pR.write(0)

def lees_bits():
    out = []
    for p in ain:
        v = p.read()
        v = 0.0 if v is None else float(v)
        out.append(1 if v < DREMPEL else 0)
    return out  # [L1,L2,M,R2,R1] met 1=zwart

try:
    vooruit()
    for _ in range(40):
        print(lees_bits())
        sleep(0.1)
    stop()
except KeyboardInterrupt:
    stop()
finally:
    board.exit()
