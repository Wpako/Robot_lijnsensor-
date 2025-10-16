# lijnvolger.py
# Basis: verbinding + korte motortest

from pyfirmata import Arduino, util
from time import sleep

POORT = 'COM5'  # pas aan als nodig

# motorpinnen (Uno + L298N/L293D)
PWM_L, PWM_R = 11, 3
IN1_L, IN2_L = 12, 13
IN3_R, IN4_R = 9, 10

# richting (jouw setup: links omgekeerd, rechts goed)
VOORUIT_L = (0, 1)
VOORUIT_R = (1, 0)

board = Arduino(POORT)
it = util.Iterator(board); it.start(); sleep(0.3)

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

try:
    print("Korte motortest (1s)...")
    vooruit()
    pL.write(0.4); pR.write(0.4); sleep(1.0)
    stop()
    print("OK.")
except KeyboardInterrupt:
    stop()
finally:
    board.exit()
