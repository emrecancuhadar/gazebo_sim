import math

def calculate_ffmc(prev_ffmc, temperature, relative_humidity, wind_speed, rain):
    mo = 147.2 * (101 - prev_ffmc) / (59.5 + prev_ffmc)
    if rain > 0.5:
        rf = rain - 0.5
        if mo > 150:
            mo += 42.5 * rf * math.exp(-100 / (251 - mo)) * (1 - math.exp(-6.93 / rf)) \
                  + 0.0015 * (mo - 150)**2 * math.sqrt(rf)
        else:
            mo += 42.5 * rf * math.exp(-100 / (251 - mo)) * (1 - math.exp(-6.93 / rf))
        mo = min(mo, 250)
    ed = 0.942 * relative_humidity**0.679 + 11 * math.exp((relative_humidity - 100) / 10) \
         + 0.18 * (21.1 - temperature) * (1 - math.exp(-0.115 * relative_humidity))
    if mo > ed:
        ko = 0.424 * (1 - (relative_humidity / 100)**1.7) \
             + 0.0694 * math.sqrt(wind_speed) * (1 - (relative_humidity / 100)**8)
        kd = ko * 0.581 * math.exp(0.0365 * temperature)
        mo = ed + (mo - ed) * 10**(-kd)
    else:
        ew = 0.618 * relative_humidity**0.753 + 10 * math.exp((relative_humidity - 100) / 10) \
             + 0.18 * (21.1 - temperature) * (1 - math.exp(-0.115 * relative_humidity))
        if mo < ew:
            ko = 0.424 * (1 - ((100 - relative_humidity) / 100)**1.7) \
                 + 0.0694 * math.sqrt(wind_speed) * (1 - ((100 - relative_humidity) / 100)**8)
            kw = ko * 0.581 * math.exp(0.0365 * temperature)
            mo = ew - (ew - mo) * 10**(-kw)
    ffmc = (59.5 * (250 - mo)) / (147.2 + mo)
    return max(0, min(ffmc, 101))


def calculate_dmc(prev_dmc, temperature, relative_humidity, rain, month):
    if rain > 1.5:
        re = 0.92 * rain - 1.27
        mo = 20 + math.exp(5.6348 - prev_dmc / 43.43)
        if prev_dmc <= 33:
            b = 100 / (0.5 + 0.3 * prev_dmc)
        elif prev_dmc <= 65:
            b = 14 - 1.3 * math.log(prev_dmc)
        else:
            b = 6.2 * math.log(prev_dmc) - 17.2
        mr = mo + (1000 * re) / (48.77 + b * re)
        dmc = 244.72 - 43.43 * math.log(mr - 20)
        dmc = max(dmc, 0)
    else:
        dmc = prev_dmc
    le_table = [6.5, 7.5, 9.0, 12.8, 13.9, 13.9, 12.4, 10.9, 9.4, 8.0, 7.0, 6.0]
    le = le_table[month - 1]
    if temperature < -1.1:
        temperature = -1.1
    k = 1.894 * (temperature + 1.1) * (100 - relative_humidity) * le * 1e-6
    return dmc + 100 * k


def calculate_dc(prev_dc, temperature, rain, month):
    if rain > 2.8:
        rd = 0.83 * rain - 1.27
        qo = 800 * math.exp(-prev_dc / 400)
        qr = qo + 3.937 * rd
        dc = 400 * math.log(800 / qr)
        dc = max(dc, 0)
    else:
        dc = prev_dc
    fl_table = [-1.6, -1.6, -1.6, 0.9, 3.8, 5.8, 6.4, 5.0, 2.4, 0.4, -1.6, -1.6]
    fl = fl_table[month - 1]
    if temperature < -2.8:
        temperature = -2.8
    v = 0.36 * (temperature + 2.8) + fl
    return dc + 0.5 * max(v, 0)


def calculate_isi(ffmc, wind_speed):
    mo = 147.2 * (101 - ffmc) / (59.5 + ffmc)
    fF = 91.9 * math.exp(-0.1386 * mo) * (1 + (mo**5.31) / (4.93e7))
    fW = math.exp(0.05039 * wind_speed)
    return 0.208 * fW * fF


def calculate_bui(dmc, dc):
    if dmc <= 0.4 * dc:
        return (0.8 * dmc * dc) / (dmc + 0.4 * dc)
    else:
        return dmc - (1 - (0.8 * dc) / (dmc + 0.4 * dc)) * (0.92 + (0.0114 * dmc)**1.7)


def calculate_fwi(isi, bui):
    if bui <= 80:
        fD = 0.626 * bui**0.809 + 2
    else:
        fD = 1000 / (25 + 108.64 * math.exp(-0.023 * bui))
    b = 0.1 * isi * fD
    return b if b <= 1 else math.exp(2.72 * (math.log(b) * 0.434) ** 0.647)
