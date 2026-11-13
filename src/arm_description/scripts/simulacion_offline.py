#!/usr/bin/env python3
"""
simulacion_offline.py
=====================
Valida el control de torque SIN ROS ni Gazebo.

Planta: dinámica completa de cuerpo rígido del xacro,
    M(q)·ddq + C(q,dq)·dq + G(q) + B·dq = tau
integrada a 1 kHz, con el mismo núcleo de control (nucleo_control.py) que usa
el nodo de Gazebo, ejecutado a 250 Hz.

Genera en docs/img/:
    sim_recta.png, sim_semiarco.png      corridas completas
    sim_escalon_rampa_no_lineal.png      escalón y rampa por articulación
                                         (brazo completo, con acoplamiento)

    python3 simulacion_offline.py [--salida DIR]
"""

import argparse
import os

import numpy as np

from diseno_pid import ESCALON, RAMPA, metricas_escalon
from modelo_brazo import ETIQUETAS, UNIDADES, ModeloBrazo, PlanoTrabajo
from nucleo_control import ControlTorque, T_ACERCAMIENTO, T_PAUSA, T_TRAZO, T_RAMPA

DT = 1e-3            # paso de la planta
CADA = 4             # control cada 4 pasos → 250 Hz


class Planta:
    def __init__(self, modelo, q0=None, cada_din=CADA):
        self.m = modelo
        self.cada_din = cada_din          # pasos entre refrescos de M, C y G
        self.q = np.zeros(4) if q0 is None else np.array(q0, dtype=float)
        self.dq = np.zeros(4)
        self._k = 0

    def paso(self, tau):
        m = self.m
        if self._k % self.cada_din == 0:  # por defecto M, C y G se refrescan a 250 Hz
            self._M = m.masa(self.q)
            self._h = m.coriolis(self.q, self.dq) + m.gravedad(self.q)
        self._k += 1
        tau = np.clip(tau, -m.tau_max, m.tau_max)
        ddq = np.linalg.solve(self._M, tau - self._h - m.amort * self.dq)
        self.dq = self.dq + DT * ddq
        self.q = self.q + DT * self.dq
        for i in range(4):                # topes mecánicos
            if self.q[i] < m.q_min[i]:
                self.q[i], self.dq[i] = m.q_min[i], max(self.dq[i], 0.0)
            elif self.q[i] > m.q_max[i]:
                self.q[i], self.dq[i] = m.q_max[i], min(self.dq[i], 0.0)


def correr(ctrl, planta, guion, T, caida=0.0):
    """guion: lista (t, comando). Devuelve el registro de estados a 250 Hz."""
    reg, tau, guion = [], np.zeros(4), sorted(guion)
    for k in range(int(round(T / DT))):
        t = k * DT
        if k % CADA == 0 and t >= caida:
            while guion and guion[0][0] <= t:
                ctrl.comando(guion.pop(0)[1])
            tau = ctrl.paso(t, planta.q, planta.dq)
            reg.append(ctrl.estado())
        planta.paso(tau if t >= caida else np.zeros(4))
    return reg


def col(reg, clave):
    return np.array([r[clave] for r in reg])


def figura_corrida(plt, reg, tipo, plano, ruta):
    t = col(reg, 't')
    traz = np.array([r['fase'] == 'trazando' for r in reg])
    P, V, uv, tau = col(reg, 'p'), col(reg, 'v'), col(reg, 'uv'), col(reg, 'tau')
    q, qr = col(reg, 'q'), col(reg, 'q_ref')
    err = np.array([r['error'] if r['error'] is not None else np.nan for r in reg])

    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    a = ax[0, 0]
    e = plano.esquinas_uv()
    a.fill(e[:, 0] * 1e3, e[:, 1] * 1e3, color='#f4efc0', ec='k', lw=0.8)
    ref = plano.lugar(tipo, np.linspace(0, 1, 200))
    a.plot(ref[:, 0] * 1e3, ref[:, 1] * 1e3, 'k--', lw=1, label='referencia')
    a.plot(uv[traz, 0] * 1e3, uv[traz, 1] * 1e3, 'C3', lw=2, label='efector')
    a.set_aspect('equal'); a.set_xlabel('u [mm]'); a.set_ylabel('v [mm]')
    a.set_title(f'Trazo sobre el plano ({tipo})'); a.legend(fontsize=8)

    a = ax[0, 1]
    for j, n in enumerate('xyz'):
        a.plot(t, P[:, j], label=n)
    a.set_title('Posición del efector en {0} [m]'); a.legend(fontsize=8)

    a = ax[0, 2]
    for j, n in enumerate(['vx', 'vy', 'vz']):
        a.plot(t, V[:, j], lw=1, label=n)
    a.plot(t, np.linalg.norm(V, axis=1), 'k', lw=1.5, label='|v|')
    a.set_title('Velocidad del efector [m/s]'); a.legend(fontsize=8)

    a = ax[1, 0]
    a.plot(t, err * 1e3, 'C3')
    a.plot(t[traz], np.abs(col(reg, 'dist_plano'))[traz] * 1e3, 'C0', lw=1, label='|distancia al plano|')
    a.set_title(f'Error cartesiano durante el trazo [mm]  (máx {np.nanmax(err) * 1e3:.2f})')
    a.legend(fontsize=8)

    a = ax[1, 1]
    for j in range(4):
        a.plot(t, (qr[:, j] - q[:, j]) * (1e3 if j == 0 else 1.0), label=ETIQUETAS[j] + (' [mm]' if j == 0 else ' [rad]'))
    a.set_title('Error articular'); a.legend(fontsize=8)

    a = ax[1, 2]
    a.plot(t, tau[:, 0], label='q0 riel [N]')
    for j in (1, 2, 3):
        a.plot(t, tau[:, j], label=ETIQUETAS[j] + ' [N·m]')
    a.set_title('Par / fuerza comandados'); a.legend(fontsize=8)
    for a in ax.flat:
        a.grid(alpha=0.3)
    for a in ax.flat[1:]:
        a.set_xlabel('t [s]')
        a.axvspan(t[traz][0], t[traz][-1], color='gray', alpha=0.08)
    fig.tight_layout()
    fig.savefig(ruta, dpi=120)
    return float(np.nanmax(err)), float(np.abs(col(reg, 'dist_plano'))[traz].max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--salida', default=os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'docs', 'img'))
    a = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(a.salida, exist_ok=True)

    modelo, plano = ModeloBrazo(), PlanoTrabajo()
    T_home = T_ACERCAMIENTO + 1.5

    # ── Corridas completas: caída inicial 0.3 s sin control → home → trazo ──
    print('=== CORRIDAS (planta no lineal) ===')
    for tipo in ('recta', 'semiarco'):
        ctrl = ControlTorque(modelo, plano)
        T = 0.3 + T_home + T_ACERCAMIENTO + T_PAUSA + T_TRAZO + 2.0
        reg = correr(ctrl, Planta(modelo), [(0.3 + T_home, tipo)], T, caida=0.3)
        emax, dmax = figura_corrida(plt, reg, tipo, plano, os.path.join(a.salida, f'sim_{tipo}.png'))
        tau = np.abs(col(reg, 'tau')).max(0)
        vmax = col(reg, 'rapidez')[np.array([r['fase'] == 'trazando' for r in reg])].max()
        print(f'{tipo:<9} error máx = {emax * 1e3:.2f} mm   fuera del plano máx = {dmax * 1e3:.2f} mm   '
              f'|v|máx = {vmax * 1e3:.1f} mm/s   |tau|máx = {np.round(tau, 3)}   '
              f'error final = {reg[-1]["error"] * 1e3:.3f} mm')

    # ── Escalón y rampa por articulación con el brazo completo ──
    print('\n=== ESCALÓN / RAMPA POR ARTICULACIÓN (planta no lineal, resto sosteniendo) ===')
    fig, ax = plt.subplots(4, 2, figsize=(12, 12))
    for i in range(4):
        for k, (prueba, amp) in enumerate((('escalon', ESCALON[i]), ('rampa', RAMPA[i]))):
            ctrl = ControlTorque(modelo, plano)
            planta = Planta(modelo, ctrl.q_home)
            reg = correr(ctrl, planta, [(T_home, f'{prueba} {i}')], T_home + 4.0)
            t = col(reg, 't') - T_home
            m = t >= -0.2
            q, qr = col(reg, 'q')[m, i], col(reg, 'q_ref')[m, i]
            q0 = ctrl.q_home[i]
            ax[i, k].plot(t[m], qr - q0, 'k--', lw=1, label='referencia')
            ax[i, k].plot(t[m], q - q0, 'C0', lw=1.8, label='respuesta')
            ax[i, k].grid(alpha=0.3)
            ax[i, k].set_ylabel(f'Δ{ETIQUETAS[i]} [{UNIDADES[i]}]')
            if prueba == 'escalon':
                mm = metricas_escalon(t[t >= 0], col(reg, 'q')[t >= 0, i] - q0, amp)
                otros = np.abs(col(reg, 'q')[m] - col(reg, 'q_ref')[m])
                otros[:, i] = 0
                txt = f"tp = {mm['tp']:.2f} s, ts(2 %) = {mm['ts']:.2f} s, Mp = {100 * mm['Mp']:.1f} %"
                ax[i, k].set_title(f'Escalón {amp:g} {UNIDADES[i]}:  {txt}', fontsize=9)
                print(f'{ETIQUETAS[i]:<10} escalón: {txt}   perturbación máx. en las demás = {otros.max():.4f}')
            else:
                e_ss = (qr - q)[np.searchsorted(t[m], T_RAMPA) - 1]
                ax[i, k].set_title(f'Rampa {amp:g} {UNIDADES[i]}/s durante {T_RAMPA:g} s:  '
                                   f'error al final de la rampa = {e_ss:.4f} {UNIDADES[i]}', fontsize=9)
                print(f'{ETIQUETAS[i]:<10} rampa:   error al final de la rampa = {e_ss:.4f} {UNIDADES[i]}')
    ax[0, 0].legend(fontsize=8)
    ax[3, 0].set_xlabel('t [s]'); ax[3, 1].set_xlabel('t [s]')
    fig.suptitle('Respuesta por articulación en la planta no lineal (PID 2 GDL + G(q))')
    fig.tight_layout()
    fig.savefig(os.path.join(a.salida, 'sim_escalon_rampa_no_lineal.png'), dpi=120)
    print(f'\nFiguras guardadas en {os.path.normpath(a.salida)}')


if __name__ == '__main__':
    main()
