#!/usr/bin/env python3
"""
diseno_pid.py
=============
Parte 1 (Control): modelo físico de cada articulación y diseño de su PID.

Modelo por articulación (referido al eje de salida / al carro):

    Armadura :  V = Ra·i + La·di/dt + Kb·w
    Par      :  tau = Kt·i
    Mecánica :  J·dw/dt + B·w = tau − tau_carga

El lazo interno de par compensa la f.c.e.m. (V = Ra·i* + Kb·w), de modo que
tau/tau* = 1/(te·s + 1) con te = La/Ra, y la planta que ve el PID es

    q(s)/tau*(s) = 1 / [ s·(J·s + B)·(te·s + 1) ]

PID de 2 grados de libertad:

    tau* = Kp·(b·r − q) + Ki·∫(r − q)dt + Kd·(c·dr − dq)  [+ G(q)]

Las ganancias se obtienen por asignación de polos. Con tp y ts se fija el par
dominante (sigma = 4/ts, wd = pi/tp) y un tercer polo real p = alfa·sigma.
Con b = wn²/(wn² + 2·zeta·wn·p) el cero cancela al polo p y la respuesta al
escalón es exactamente la de segundo orden especificada.

Ejecutar como script (sin ROS) imprime las tablas y guarda las gráficas de
respuesta a escalón y rampa:

    python3 diseno_pid.py [--salida DIR] [--rotor]
"""

import argparse
import os

import numpy as np

from modelo_brazo import JOINTS, ETIQUETAS, UNIDADES, ModeloBrazo

# ── Especificaciones de sintonización ─────────────────────────────
TP = 1.0          # tiempo pico [s]
TS = 2.0          # tiempo de asentamiento, criterio 2 % (4/sigma) [s]
ALFA = 5.0        # tercer polo: p = ALFA·sigma

# Configuración nominal para evaluar la inercia efectiva (postura "home")
Q_NOMINAL = np.array([0.15, 0.37, 0.02, -0.55])

RPM = 2.0 * np.pi / 60.0
R_POLEA = 20 * 0.002 / (2.0 * np.pi)       # GT2 de 20 dientes, paso 2 mm → 6.366 mm

# ── Actuadores (propuesta 1 de la entrega 2) ──────────────────────
# Datos de catálogo: Vn, par de paro, corriente de paro, velocidad en vacío,
# reducción.  De ellos se derivan Ra, Kt y Kb referidos a la salida.
# 'La', 'Jm' y 'B' NO vienen en las hojas de datos de los servos: son
# estimaciones (ver README) y deben identificarse en el motor real.
MOTORES = [
    dict(nombre='NEMA 17 17HS4401 + polea GT2 20T', lineal=True,
         Vn=12.0, I_nom=1.7, T_sost=0.40, Ra=1.5, La=2.8e-3, Jm=5.4e-6,
         N=1.0 / R_POLEA, B=2.0),
    dict(nombre='Dynamixel XL330-M288-T', lineal=False,
         Vn=5.0, T_paro=0.52, I_paro=1.47, w0=103 * RPM, N=288.35,
         La=0.3e-3, Jm=3.0e-8),
    dict(nombre='Dynamixel XM430-W350-R', lineal=False,
         Vn=12.0, T_paro=4.10, I_paro=2.30, w0=46 * RPM, N=353.5,
         La=0.2e-3, Jm=2.0e-7),
    dict(nombre='Feetech STS3215 (12 V)', lineal=False,
         Vn=12.0, T_paro=2.94, I_paro=2.70, w0=62 * RPM, N=345.0,
         La=1.0e-3, Jm=2.0e-7),
]
FRACCION_FRICCION = 0.05      # B = 5 % del par de paro a la velocidad en vacío


def constantes_motor(i):
    """Constantes físicas de la articulación i, referidas a la salida."""
    m = MOTORES[i]
    if m['lineal']:
        # Motor a pasos tratado como motor síncrono equivalente (por fase):
        # Kt = T_sostenimiento / I_nominal, Kb = Kt (SI); la polea convierte a N y m.
        kt_m = m['T_sost'] / m['I_nom']
        Kt, Kb, Ra = kt_m * m['N'], kt_m * m['N'], m['Ra']
        u_max = m['T_sost'] * m['N']
        B = m['B']
    else:
        Ra = m['Vn'] / m['I_paro']
        Kt = m['T_paro'] / m['I_paro']
        Kb = m['Vn'] / m['w0']
        u_max = m['T_paro']
        B = FRACCION_FRICCION * m['T_paro'] / m['w0']
    return dict(nombre=m['nombre'], lineal=m['lineal'], Vn=m['Vn'], Ra=Ra, La=m['La'],
                Kt=Kt, Kb=Kb, N=m['N'], Jm=m['Jm'], J_rotor=m['Jm'] * m['N'] ** 2,
                B=B, te=m['La'] / Ra, u_max=u_max, eta=Kt / Kb)


def sintonizar(J, B, tp=TP, ts=TS, alfa=ALFA):
    """Ganancias del PID de 2 GDL por asignación de polos."""
    sigma = 4.0 / ts
    wd = np.pi / tp
    wn = np.hypot(sigma, wd)
    zeta = sigma / wn
    p = alfa * sigma
    Kp = J * (wn ** 2 + 2 * zeta * wn * p)
    Ki = J * wn ** 2 * p
    Kd = J * (2 * zeta * wn + p) - B
    b = wn ** 2 / (wn ** 2 + 2 * zeta * wn * p)
    return dict(Kp=Kp, Ki=Ki, Kd=Kd, b=b, wn=wn, zeta=zeta, p=p,
                Mp=float(np.exp(-sigma * np.pi / wd)), ev=2 * zeta / wn)


def disenar(modelo=None, q_nom=Q_NOMINAL, incluir_rotor=False):
    """
    Devuelve una lista (una entrada por articulación) con las constantes
    físicas, la inercia efectiva y las ganancias.

    incluir_rotor=False → planta de Gazebo (solo eslabones; es lo que se simula).
    incluir_rotor=True  → suma la inercia del rotor reflejada N²·Jm (hardware real).
    """
    modelo = modelo or ModeloBrazo()
    M = modelo.masa(np.asarray(q_nom, dtype=float))
    out = []
    for i in range(4):
        c = constantes_motor(i)
        J_carga = float(M[i, i])
        J = J_carga + (c['J_rotor'] if incluir_rotor else 0.0)
        # El amortiguamiento que simula Gazebo es el <dynamics damping> del xacro
        B = float(modelo.amort[i])
        g = sintonizar(J, B)
        u_max = min(c['u_max'], float(modelo.tau_max[i]))
        out.append(dict(c, joint=JOINTS[i], etiqueta=ETIQUETAS[i], unidad=UNIDADES[i],
                        J_carga=J_carga, J=J, B_xacro=B, u_max=u_max, **g))
    return out


# ──────────────────────────────────────────────────────────────────
# Bloques reutilizados por el nodo de control y por las simulaciones
# ──────────────────────────────────────────────────────────────────
class PID2GDL:
    """PID de 2 GDL con saturación y anti-windup (integración condicional)."""

    def __init__(self, Kp, Ki, Kd, b, u_max):
        self.Kp, self.Ki, self.Kd, self.b, self.u_max = Kp, Ki, Kd, b, u_max
        self.integral = 0.0
        self.seguimiento = False

    def cambiar_modo(self, seguimiento, r):
        """
        seguimiento=True  → b = c = 1 (PID sobre el error; referencia suave).
        seguimiento=False → b de diseño, c = 0 (respuesta al escalón sin ceros).
        Transferencia sin saltos: el integrador absorbe el cambio Kp·(1 − b)·r.
        """
        if seguimiento != self.seguimiento:
            salto = self.Kp * (1.0 - self.b) * r / self.Ki
            self.integral += -salto if seguimiento else salto
            self.seguimiento = seguimiento

    def paso(self, r, dr, q, dq, dt, u_ff=0.0):
        b, c = (1.0, 1.0) if self.seguimiento else (self.b, 0.0)
        e = r - q
        u = self.Kp * (b * r - q) + self.Ki * self.integral + self.Kd * (c * dr - dq) + u_ff
        u_sat = float(np.clip(u, -self.u_max, self.u_max))
        if u == u_sat or (u - u_sat) * e < 0.0:
            self.integral += e * dt
        return u_sat


class Actuador:
    """
    Motor de CD con lazo de par por compensación de f.c.e.m.
    Limita el par por la tensión disponible (curva par-velocidad) y aplica la
    constante de tiempo eléctrica te = La/Ra (discretización exacta).
    """

    def __init__(self, c):
        self.Ra, self.Kt, self.Kb, self.Vn, self.te = c['Ra'], c['Kt'], c['Kb'], c['Vn'], c['te']
        self.i = 0.0
        self.V = 0.0

    def paso(self, u_cmd, w, dt):
        i_ref = u_cmd / self.Kt
        self.V = float(np.clip(self.Ra * i_ref + self.Kb * w, -self.Vn, self.Vn))
        i_ss = (self.V - self.Kb * w) / self.Ra
        self.i = i_ss + (self.i - i_ss) * np.exp(-dt / self.te)
        return self.Kt * self.i


def simular_lineal(d, referencia, T=4.0, dt=1e-3, seguimiento=False, dt_ctrl=0.004):
    """
    Lazo cerrado de UNA articulación con su modelo lineal (J, B, motor) y el
    PID discreto (dt_ctrl).  referencia(t) → (r, dr).  Devuelve t, q, u, r.
    """
    pid = PID2GDL(d['Kp'], d['Ki'], d['Kd'], d['b'], d['u_max'])
    pid.cambiar_modo(seguimiento, 0.0)
    act = Actuador(d)
    n = int(round(T / dt))
    cada = max(1, int(round(dt_ctrl / dt)))
    t = np.arange(n) * dt
    q = w = u = 0.0
    Q, U, R = np.zeros(n), np.zeros(n), np.zeros(n)
    for k in range(n):
        r, dr = referencia(t[k])
        if k % cada == 0:
            u = pid.paso(r, dr, q, w, cada * dt)
        tau = act.paso(u, w, dt)
        w += dt * (tau - d['B_xacro'] * w) / d['J']
        q += dt * w
        Q[k], U[k], R[k] = q, tau, r
    return t, Q, U, R


def metricas_escalon(t, y, amplitud):
    yn = y / amplitud
    k = int(np.argmax(yn))
    fuera = np.where(np.abs(yn - 1.0) > 0.02)[0]
    return dict(tp=t[k], Mp=yn[k] - 1.0, ts=t[fuera[-1]] if len(fuera) else 0.0)


# Amplitudes de prueba (pequeñas, dentro del rango lineal de cada actuador)
ESCALON = [0.05, 0.20, 0.20, 0.20]      # m, rad
RAMPA = [0.02, 0.10, 0.10, 0.10]        # m/s, rad/s


def _tablas(D):
    print('\n=== CONSTANTES FÍSICAS (referidas a la salida) ===')
    print(f"{'articulación':<11}{'actuador':<34}{'Vn':>5}{'Ra':>7}{'La':>9}{'Kt':>8}{'Kb':>8}"
          f"{'te[ms]':>8}{'J_carga':>10}{'N²Jm':>9}{'B':>8}{'u_max':>7}")
    for d in D:
        print(f"{d['etiqueta']:<11}{d['nombre']:<34}{d['Vn']:>5.1f}{d['Ra']:>7.2f}{d['La']:>9.1e}"
              f"{d['Kt']:>8.3f}{d['Kb']:>8.3f}{d['te'] * 1e3:>8.2f}{d['J_carga']:>10.4f}"
              f"{d['J_rotor']:>9.4f}{d['B_xacro']:>8.4f}{d['u_max']:>7.2f}")
    d = D[0]
    print(f"\n=== POLOS: wn = {d['wn']:.3f} rad/s, zeta = {d['zeta']:.3f}, p = {d['p']:.1f} rad/s, "
          f"Mp = {100 * d['Mp']:.1f} %, b = {d['b']:.4f} ===")
    print(f"{'articulación':<11}{'J':>9}{'Kp':>10}{'Ki':>10}{'Kd':>10}")
    for d in D:
        print(f"{d['etiqueta']:<11}{d['J']:>9.4f}{d['Kp']:>10.4f}{d['Ki']:>10.4f}{d['Kd']:>10.4f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[2])
    ap.add_argument('--salida', default=os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'docs', 'img'))
    ap.add_argument('--rotor', action='store_true',
                    help='incluir la inercia del rotor reflejada (hardware real)')
    a = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    D = disenar(incluir_rotor=a.rotor)
    _tablas(D)
    os.makedirs(a.salida, exist_ok=True)

    fig, ax = plt.subplots(4, 3, figsize=(15, 12))
    print('\n=== RESPUESTA AL ESCALÓN Y A LA RAMPA (modelo lineal con motor) ===')
    for i, d in enumerate(D):
        A, v = ESCALON[i], RAMPA[i]
        un = d['unidad']
        t, q, u, r = simular_lineal(d, lambda t: (A, 0.0))
        m = metricas_escalon(t, q, A)
        ax[i, 0].plot(t, r, 'k--', lw=1, label='referencia')
        ax[i, 0].plot(t, q, 'C0', lw=1.8, label='respuesta')
        ax[i, 0].axhline(1.02 * A, color='gray', lw=0.6, ls=':')
        ax[i, 0].axhline(0.98 * A, color='gray', lw=0.6, ls=':')
        ax[i, 0].plot(m['tp'], q[int(np.argmax(q))], 'ro', ms=4)
        ax[i, 0].set_ylabel(f"{d['etiqueta']} [{un}]")
        ax[i, 0].set_title(f"Escalón {A:g} {un}:  tp = {m['tp']:.2f} s,  ts(2 %) = {m['ts']:.2f} s,  "
                           f"Mp = {100 * m['Mp']:.1f} %", fontsize=9)

        t2, q2, _, r2 = simular_lineal(d, lambda t: (v * t, v), seguimiento=False)
        t3, q3, _, _ = simular_lineal(d, lambda t: (v * t, v), seguimiento=True)
        ax[i, 1].plot(t2, r2, 'k--', lw=1, label='referencia')
        ax[i, 1].plot(t2, q2, 'C0', lw=1.8, label='PID 2 GDL (b de diseño, c = 0)')
        ax[i, 1].plot(t3, q3, 'C2', lw=1.4, label='modo seguimiento (b = c = 1)')
        ax[i, 1].set_title(f"Rampa {v:g} {un}/s:  error final = {r2[-1] - q2[-1]:.4f} {un} "
                           f"(2 GDL),  {r2[-1] - q3[-1]:.1e} (seguimiento)", fontsize=9)

        ax[i, 2].plot(t, u, 'C3', lw=1.4)
        ax[i, 2].set_title(f"Esfuerzo en el escalón [{'N' if d['lineal'] else 'N·m'}]  "
                           f"(límite ±{d['u_max']:.2f})", fontsize=9)
        for j in range(3):
            ax[i, j].grid(alpha=0.3)
        print(f"{d['etiqueta']:<11} escalón: tp = {m['tp']:.3f} s  ts = {m['ts']:.3f} s  "
              f"Mp = {100 * m['Mp']:.1f} %  |u|max = {np.abs(u).max():.3f}   "
              f"rampa: e_ss = {r2[-1] - q2[-1]:.4f} (2 GDL)  {r2[-1] - q3[-1]:.2e} (seguimiento)")
    ax[0, 0].legend(fontsize=8)
    ax[0, 1].legend(fontsize=8)
    for j in range(3):
        ax[3, j].set_xlabel('t [s]')
    fig.suptitle(f'PID por articulación — tp = {TP:g} s, ts = {TS:g} s'
                 + ('  (con inercia de rotor)' if a.rotor else ''), fontsize=12)
    fig.tight_layout()
    ruta = os.path.join(a.salida, 'pid_escalon_rampa.png')
    fig.savefig(ruta, dpi=130)
    print(f'\nFigura guardada en {os.path.normpath(ruta)}')


if __name__ == '__main__':
    main()
