# lijnvolger.py
# Simpel volgen met P-regeling

from pyfirmata import Arduino, util
from time import sleep

POORT = 'COM5'
SENSOR = [0,1,2,3,4]
DREMPEL = 0.46

# motor
PWM_L,PWM_R = 11,3
IN1_L,IN2_L = 12,13
IN3_R,IN4_R = 9,10
VOORUIT_L=(0,1); VOORUIT_R=(1,0)

# rijden
BASIS = 0.42   # basissnelheid
Kp = 0.18      # alleen P (eenvoudig)

board=Arduino(POORT)
it=util.Iterator(board); it.start(); sleep(0.3)

ain=[board.get_pin(f'a:{ch}:i') for ch in SENSOR]
pL=board.get_pin(f'd:{PWM_L}:p'); pR=board.get_pin(f'd:{PWM_R}:p')
L1=board.get_pin(f'd:{IN1_L}:o'); L2=board.get_pin(f'd:{IN2_L}:o')
R3=board.get_pin(f'd:{IN3_R}:o'); R4=board.get_pin(f'd:{IN4_R}:o')

def zet_richting(pair,a,b): x,y=pair; a.write(1 if x else 0); b.write(1 if y else 0)
def vooruit(): zet_richting(VOORUIT_L,L1,L2); zet_richting(VOORUIT_R,R3,R4)
def stop(): pL.write(0); pR.write(0)

def bits():
    res=[]
    for p in ain:
        v=p.read(); v=0.0 if v is None else float(v)
        res.append(1 if v<DREMPEL else 0)
    return res

def positie(b):
    w=[-2,-1,0,1,2]; s=sum(b)
    if s==0: return None
    return sum(wi*bi for wi,bi in zip(w,b))/s

try:
    vooruit()
    while True:
        b=bits(); e=positie(b)
        if e is None:
            pL.write(0.30); pR.write(0.30)  # rustig zoeken
        else:
            corr = -Kp*e
            l = max(0,min(1, BASIS + corr))
            r = max(0,min(1, BASIS - corr))
            pL.write(l); pR.write(r)
        sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    stop(); board.exit()
