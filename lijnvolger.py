# - Sensor: A0..A4 = [L1, L2, M, R2, R1]
# - Volgt rustig, rijdt terug als hij de lijn kwijt is
# - Minder “heen-en-weer” sturen (dode zone + trim)

from pyfirmata import Arduino, util
from time import sleep, time

# instellingen
POORT = 'COM5'

# sensor
SENSOR_KANALEN = [0, 1, 2, 3, 4]   # A0..A4
DREMPEL_ZWART = 0.46               # lager = donker (zwart)
HYSTERESE = 0.03
DEMPING = 0.50                     # 0 veel demping, 1 weinig
SENSOR_INVERT = False

# motorpinnen (Uno + L298N/L293D)
PWM_LINKS, PWM_RECHTS = 11, 3
IN1_L, IN2_L = 12, 13
IN3_R, IN4_R = 9, 10

# richting (jouw setup)
VOORUIT_LINKS  = (0, 1)
VOORUIT_RECHTS = (1, 0)

# snelheid
BASIS = 0.40
MAX_P = 0.70
RECHTUIT_TRIM_START = 0.96        # rechterwiel ietsje knijpen

# regeling
Kp, Kd = 0.24, 0.07
STUUR_LPF = 0.55

# minder gewiebel
DODE_ZONE = 0.15                  # kleine fout negeren
STUUR_MIN_OMSLA_MS = 120          # niet te snel links<->rechts
STUUR_MIN_AMPL = 0.04             # hele kleine stuur stapjes weg

# auto-trim (corrigeert langzaam scheef rijden)
AUTO_TRIM_AAN = True
AUTO_TRIM_GAIN = 0.002
AUTO_TRIM_MAX_AFW = 0.10

# zachte PWM stappen
MAX_STAP = 0.05

# bochten
BOCHT_BOOST = 0.10
BINNEN_MIN = 0.28
SCHERP_BASIS = 0.34
SCHERP_BINNEN_MIN = 0.26

# lijn kwijt
HOU_VAST_MS = 120                 # kort vasthouden
HOU_VAST_REM = 0.65
TERUG_START_MS = 180              # daarna terugrijden
TERUG_SNELHEID = 0.30
TERUG_BIAS = 0.18
TERUG_MAX_MS = 900
BACKTRACK_COOLDOWN_MS = 300

# laatste redmiddel
ZOEK_LANGZAAM = 0.28
ZOEK_BIAS = 0.12

# helpers
def begrens(x, lo=0.0, hi=1.0):
    return hi if x > hi else lo if x < lo else x

def keer_om(pair):
    a, b = pair
    return (1 - a, 1 - b)

# verbinding
board = Arduino(POORT)
it = util.Iterator(board); it.start(); sleep(0.3)

# io
analoog_in = [board.get_pin(f'a:{ch}:i') for ch in SENSOR_KANALEN]
pwm_L = board.get_pin(f'd:{PWM_LINKS}:p'); pwm_R = board.get_pin(f'd:{PWM_RECHTS}:p')
L_IN1 = board.get_pin(f'd:{IN1_L}:o'); L_IN2 = board.get_pin(f'd:{IN2_L}:o')
R_IN3 = board.get_pin(f'd:{IN3_R}:o'); R_IN4 = board.get_pin(f'd:{IN4_R}:o')

def zet_richting_links(pair):
    a, b = pair
    L_IN1.write(1 if a else 0); L_IN2.write(1 if b else 0)

def zet_richting_rechts(pair):
    a, b = pair
    R_IN3.write(1 if a else 0); R_IN4.write(1 if b else 0)

def vooruit():
    zet_richting_links(VOORUIT_LINKS); zet_richting_rechts(VOORUIT_RECHTS)

def achteruit():
    zet_richting_links(keer_om(VOORUIT_LINKS)); zet_richting_rechts(keer_om(VOORUIT_RECHTS))

# sensorfilter
_filter, _bits, _eerste = [0.0]*5, [0]*5, True
def lees_sensor_bits():
    global _filter, _bits, _eerste
    ruwe = []
    for pin in analoog_in:
        v = pin.read()
        ruwe.append(0.0 if v is None else float(v))
    if _eerste:
        _filter = ruwe[:]; _eerste = False
    else:
        _filter = [DEMPING*r + (1-DEMPING)*f for r, f in zip(ruwe, _filter)]
    uit = []
    lo, hi = DREMPEL_ZWART - HYSTERESE, DREMPEL_ZWART + HYSTERESE
    for i, v in enumerate(_filter):
        if v <= lo:
            uit.append(1)   # zwart
        elif v >= hi:
            uit.append(0)   # wit
        else:
            uit.append(_bits[i])
    if SENSOR_INVERT:
        uit = [1 - b for b in uit]
    _bits = uit
    return uit  # [L1, L2, M, R2, R1]

def lijn_pos(bits5):
    w = [-2, -1, 0, 1, 2]
    s = sum(bits5)
    if s == 0: return None
    return sum(wi*bi for wi, bi in zip(w, bits5)) / s

# pwm met zachte stappen
_vorige_L = 0.0; _vorige_R = 0.0
def stel_snelheid_in(L, R, rechts_trim):
    global _vorige_L, _vorige_R
    L = begrens(L, 0.0, MAX_P)
    R = begrens(R * rechts_trim, 0.0, MAX_P)
    dL = begrens(L - _vorige_L, -MAX_STAP, MAX_STAP)
    dR = begrens(R - _vorige_R, -MAX_STAP, MAX_STAP)
    _vorige_L += dL; _vorige_R += dR
    pwm_L.write(_vorige_L); pwm_R.write(_vorige_R)

def stop():
    global _vorige_L, _vorige_R
    _vorige_L = 0.0; _vorige_R = 0.0
    pwm_L.write(0); pwm_R.write(0)

def weer_op_lijn(b):
    return b[2] == 1 or sum(b) >= 2

def backtrack(laatste_stuurteken, rechts_trim):
    achteruit()
    t0 = time()
    bias = TERUG_BIAS if laatste_stuurteken > 0 else -TERUG_BIAS
    while (time() - t0) * 1000 < TERUG_MAX_MS:
        b = lees_sensor_bits()
        if weer_op_lijn(b):
            stel_snelheid_in(0.0, 0.0, rechts_trim); sleep(0.06)
            vooruit()
            stel_snelheid_in(BASIS*0.9, BASIS*0.9, rechts_trim); sleep(0.05)
            return True
        L = TERUG_SNELHEID - bias
        R = TERUG_SNELHEID + bias
        stel_snelheid_in(L, R, rechts_trim)
        sleep(0.01)
    stop(); vooruit()
    return False

# hoofdloop
vorige_fout = 0.0
stuur_filt = 0.0
laatst_gezien = time()
vasthouden_tot = 0.0
laatste_cmd = (0.0, 0.0)
laatste_stuurteken = 0          # <0 links, >0 rechts
laatste_sign = 0                # teken van stuur_filt
laatste_omsla_tijd = 0.0
na_backtrack_tot = 0.0

rechts_trim = RECHTUIT_TRIM_START
auto_bias = 0.0

try:
    print("Lijnvolger (simpel NL) — Ctrl+C om te stoppen.")
    vooruit()

    while True:
        b = lees_sensor_bits()
        fout = lijn_pos(b)
        nu = time()

        # niks gezien
        if fout is None:
            if vasthouden_tot == 0.0:
                vasthouden_tot = nu + HOU_VAST_MS/1000.0
            if nu <= vasthouden_tot:
                stel_snelheid_in(laatste_cmd[0]*HOU_VAST_REM,
                                  laatste_cmd[1]*HOU_VAST_REM,
                                  rechts_trim)
                sleep(0.01); continue

            if nu >= na_backtrack_tot and (nu - laatst_gezien)*1000 >= TERUG_START_MS:
                if backtrack(laatste_stuurteken, rechts_trim):
                    laatst_gezien = time()
                    vasthouden_tot = 0.0
                    na_backtrack_tot = time() + BACKTRACK_COOLDOWN_MS/1000.0
                    continue

            bias = ZOEK_BIAS if (nu - laatst_gezien) > 0.3 else 0.0
            stel_snelheid_in(ZOEK_LANGZAAM*(1.0-bias),
                              ZOEK_LANGZAAM*(1.0+bias),
                              rechts_trim)
            sleep(0.01); continue

        # wel gezien
        vasthouden_tot = 0.0
        laatst_gezien = nu

        # kleine fout negeren
        fout_eff = 0.0 if abs(fout) < DODE_ZONE else fout

        # PD
        P = fout_eff
        D = fout_eff - vorige_fout
        vorige_fout = fout_eff

        ruwe_stuur = (Kp*P + Kd*D)      # >0 = rechts
        laatste_stuurteken = 1 if ruwe_stuur > 0 else -1

        # filter
        stuur = -ruwe_stuur
        nieuw = (1 - STUUR_LPF) * stuur_filt + STUUR_LPF * stuur

        # anti-triller: te klein -> 0
        if abs(nieuw) < STUUR_MIN_AMPL:
            nieuw = 0.0

        # anti-triller: niet te snel van teken wisselen
        sign = 0 if nieuw == 0 else (1 if nieuw > 0 else -1)
        if laatste_sign != 0 and sign != 0 and sign != laatste_sign:
            if (nu - laatste_omsla_tijd) * 1000 < STUUR_MIN_OMSLA_MS:
                nieuw = 0.5 * (stuur_filt if stuur_filt != 0 else (STUUR_MIN_AMPL * laatste_sign))
                sign = laatste_sign
            else:
                laatste_omsla_tijd = nu
        laatste_sign = sign
        stuur_filt = nieuw

        # auto-trim (langzaam)
        if AUTO_TRIM_AAN:
            auto_bias += AUTO_TRIM_GAIN * ruwe_stuur
            auto_bias = begrens(auto_bias, -AUTO_TRIM_MAX_AFW, AUTO_TRIM_MAX_AFW)
            rechts_trim = begrens(RECHTUIT_TRIM_START - auto_bias, 0.85, 1.15)

        # basis snelheden
        L = BASIS + stuur_filt
        R = BASIS - stuur_filt

        # bocht of scherpe hoek
        scherp_links  = (b[0] or b[1]) and not (b[2] or b[3] or b[4])
        scherp_rechts = (b[3] or b[4]) and not (b[0] or b[1] or b[2])
        if scherp_links:
            L = max(min(L, SCHERP_BASIS) - BOCHT_BOOST, SCHERP_BINNEN_MIN)
            R = min(max(R, SCHERP_BASIS) + BOCHT_BOOST, MAX_P)
        elif scherp_rechts:
            L = min(max(L, SCHERP_BASIS) + BOCHT_BOOST, MAX_P)
            R = max(min(R, SCHERP_BASIS) - BOCHT_BOOST, SCHERP_BINNEN_MIN)
        else:
            links  = (b[0] or b[1]) and not (b[3] or b[4])
            rechts = (b[3] or b[4]) and not (b[0] or b[1])
            if links:
                L = max(L - BOCHT_BOOST, BINNEN_MIN); R = min(R + BOCHT_BOOST, MAX_P)
            elif rechts:
                L = min(L + BOCHT_BOOST, MAX_P);      R = max(R - BOCHT_BOOST, BINNEN_MIN)

        stel_snelheid_in(L, R, rechts_trim)
        laatste_cmd = (L, R)
        sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    stop(); board.exit()
    print("Gestopt.")
