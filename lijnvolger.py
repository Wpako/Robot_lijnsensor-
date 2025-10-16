# lijnvolger.py
# PD + filtering + micro-pauze bij kort lijnoverlies

from pyfirmata import Arduino, util
from time import sleep, time

POORT='COM5'
SENSOR=[0,1,2,3,4]
DREMPEL=0.46; HYST=0.03; ALPHA=0.5  # filtering
PWM_L,PWM_R=11,3; IN1_L,IN2_L=12,13; IN3_R,IN4_R=9,10
VOORUIT_L=(0,1); VOORUIT_R=(1,0)
BASIS=0.42; Kp=0.22; Kd=0.06
HOLD_MS=120; HOLD_SCALE=0.65

board=Arduino(POORT); it=util.Iterator(board); it.start(); sleep(0.3)
ain=[board.get_pin(f'a:{ch}:i') for ch in SENSOR]
pL=board.get_pin(f'd:{PWM_L}:p'); pR=board.get_pin(f'd:{PWM_R}:p')
L1=board.get_pin(f'd:{IN1_L}:o'); L2=board.get_pin(f'd:{IN2_L}:o')
R3=board.get_pin(f'd:{IN3_R}:o'); R4=board.get_pin(f'd:{IN4_R}:o')

flt=[0.0]*5; last=[0]*5; first=True
def bits():
    global first, flt, last
    raw=[0.0 if (v:=p.read()) is None else float(v) for p in ain]
    if first: flt=raw[:]; first=False
    else: flt=[ALPHA*r+(1-ALPHA)*f for r,f in zip(raw,flt)]
    out=[]; lo,hi=DREMPEL-HYST, DREMPEL+HYST
    for i,v in enumerate(flt):
        if v<=lo: out.append(1)
        elif v>=hi: out.append(0)
        else: out.append(last[i])
    last=out; return out

def positie(b):
    w=[-2,-1,0,1,2]; s=sum(b)
    return None if s==0 else sum(wi*bi for wi,bi in zip(w,b))/s

def zet_richting(pair,a,b): x,y=pair; a.write(1 if x else 0); b.write(1 if y else 0)
def vooruit(): zet_richting(VOORUIT_L,L1,L2); zet_richting(VOORUIT_R,R3,R4)
def stop(): pL.write(0); pR.write(0)

prev=0.0; hold_until=0.0; last_cmd=(0.0,0.0)

try:
    vooruit()
    while True:
        b=bits(); e=positie(b); now=time()
        if e is None:
            if hold_until==0.0: hold_until=now+HOLD_MS/1000.0
            if now<=hold_until:
                pL.write(last_cmd[0]*HOLD_SCALE); pR.write(last_cmd[1]*HOLD_SCALE)
            else:
                pL.write(0.30); pR.write(0.30)
        else:
            hold_until=0.0
            P=e; D=e-prev; prev=e
            steer=-(Kp*P+Kd*D)
            l=max(0,min(1,BASIS+steer)); r=max(0,min(1,BASIS-steer))
            pL.write(l); pR.write(r); last_cmd=(l,r)
        sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    stop(); board.exit()
