#!/usr/bin/env python3
"""
nucleo_control.py
=================
Núcleo del control de torque, independiente de ROS (solo numpy).

    tau = PID(q_ref − q) + G(q)        (una ley por articulación)

Lo ejecutan, sin cambios, el nodo de Gazebo (control_torque.py) y la
simulación sin ROS (simulacion_offline.py): lo que se valida fuera de línea
es exactamente lo que corre en Gazebo.

Comandos (texto):
    home                 ir a la postura de reposo
    recta                acercarse al punto A y trazar la diagonal A→C del plano
    semiarco             acercarse y trazar el semiarco inscrito
    paro                 congelar la referencia en la posición actual
    escalon <i> [amp]    escalón en la articulación i (0..3)
    rampa <i> [pend]     rampa de 2 s en la articulación i
"""

import numpy as np

from diseno_pid import Actuador, PID2GDL, disenar, ESCALON, RAMPA, Q_NOMINAL
from modelo_brazo import CinematicaInversa, ModeloBrazo, PlanoTrabajo, perfil_quintico

T_ACERCAMIENTO = 3.0     # s, movimiento articular hasta el inicio del lugar geométrico
T_PAUSA = 1.0            # s, espera antes de trazar
T_TRAZO = 6.0            # s, duración del trazo (igual que en la entrega 2)
T_RAMPA = 2.0            # s, duración de la rampa de prueba
N_TABLA = 301            # puntos de la tabla q(s)


class ControlTorque:
    def __init__(self, modelo=None, plano=None, compensar_gravedad=True,
                 modelar_actuador=True, incluir_rotor=False):
        self.m = modelo or ModeloBrazo()
        self.plano = plano or PlanoTrabajo()
        self.diseno = disenar(self.m, incluir_rotor=incluir_rotor)
        self.pid = [PID2GDL(d['Kp'], d['Ki'], d['Kd'], d['b'], d['u_max']) for d in self.diseno]
        self.act = [Actuador(d) for d in self.diseno] if modelar_actuador else None
        self.compensar_gravedad = compensar_gravedad
        self.q_home = Q_NOMINAL.copy()

        # Tablas q(s) de los dos lugares geométricos (cinemática inversa)
        ik = CinematicaInversa(self.m, self.plano, semilla=self.q_home)
        self.tablas = {}
        self.s_tabla = np.linspace(0.0, 1.0, N_TABLA)
        for tipo in ('recta', 'semiarco'):
            P = self.plano.uv_a_xyz(self.plano.lugar(tipo, self.s_tabla))
            Q, ok = ik.trayectoria(P)
            if not ok:
                raise RuntimeError(f'Cinemática inversa sin solución para "{tipo}"')
            self.tablas[tipo] = dict(P=P, Q=Q, dQ=np.gradient(Q, self.s_tabla, axis=0))

        self.t = None
        self.modo = 'inicio'          # inicio | reposo | articular | trazo | prueba
        self.tipo = 'ninguna'         # recta | semiarco | home | escalon | rampa | paro
        self.fase = 'esperando'
        self.s = 0.0
        self.q_ref = np.zeros(4)
        self.dq_ref = np.zeros(4)
        self.tau = np.zeros(4)
        self._modo_pid(True)
        self._pendientes = []
        self._plan = None
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._q = np.zeros(4)
        self._p_ref = None

    # ── Comandos ──────────────────────────────────────────────────
    def comando(self, texto):
        """Encola un comando; se atiende en el siguiente paso de control."""
        self._pendientes.append(texto.strip().lower())

    def _atender(self, cmd, t, q):
        partes = cmd.split()
        if not partes:
            return
        c = partes[0]
        if c in ('recta', 'semiarco'):
            self.tipo = c
            self._ir_a(t, self.tablas[c]['Q'][0], 'acercamiento', despues='pausa')
        elif c == 'home':
            self.tipo = 'home'
            self._ir_a(t, self.q_home, 'acercamiento')
        elif c == 'paro':
            self.tipo, self.modo, self.fase = 'paro', 'reposo', 'detenido'
            self._modo_pid(True)
            self.q_ref, self.dq_ref = q.copy(), np.zeros(4)
            self._p_ref = None
        elif c in ('escalon', 'rampa') and len(partes) >= 2:
            i = int(partes[1])
            if not 0 <= i <= 3:
                return
            v = float(partes[2]) if len(partes) > 2 else (ESCALON[i] if c == 'escalon' else RAMPA[i])
            self.tipo, self.modo, self.fase = c, 'prueba', f'{c} q{i}'
            self._modo_pid(False)
            self._p_ref = None
            self._plan = dict(i=i, v=v, t0=t, q0=self.q_ref.copy())

    def _ir_a(self, t, q_fin, fase, despues=None):
        self.modo, self.fase = 'articular', fase
        self._modo_pid(True)
        self._p_ref = None
        self._plan = dict(t0=t, q0=self.q_ref.copy(), q1=np.asarray(q_fin, dtype=float),
                          despues=despues)

    def _modo_pid(self, seguimiento):
        for i, pid in enumerate(self.pid):
            pid.cambiar_modo(seguimiento, self.q_ref[i])

    # ── Generación de referencia ──────────────────────────────────
    def _referencia(self, t):
        if self.modo == 'articular':
            pl = self._plan
            s, ds = perfil_quintico(t - pl['t0'], T_ACERCAMIENTO)
            self.q_ref = pl['q0'] + (pl['q1'] - pl['q0']) * s
            self.dq_ref = (pl['q1'] - pl['q0']) * ds
            if t - pl['t0'] >= T_ACERCAMIENTO:
                self.dq_ref = np.zeros(4)
                if pl['despues'] == 'pausa':
                    self.modo, self.fase = 'pausa', 'pausa'
                    self._plan = dict(t0=t)
                else:
                    self.modo, self.fase = 'reposo', 'en posición'
        elif self.modo == 'pausa':
            if t - self._plan['t0'] >= T_PAUSA:
                self.modo, self.fase, self.s = 'trazo', 'trazando', 0.0
                self._plan = dict(t0=t)
        elif self.modo == 'trazo':
            tab = self.tablas[self.tipo]
            s, ds = perfil_quintico(t - self._plan['t0'], T_TRAZO)
            self.s = float(s)
            self.q_ref = np.array([np.interp(s, self.s_tabla, tab['Q'][:, j]) for j in range(4)])
            self.dq_ref = np.array([np.interp(s, self.s_tabla, tab['dQ'][:, j]) for j in range(4)]) * ds
            self._p_ref = self.plano.uv_a_xyz(self.plano.lugar(self.tipo, s))[0]
            if t - self._plan['t0'] >= T_TRAZO:
                self.modo, self.fase, self.dq_ref = 'reposo', 'terminada', np.zeros(4)
        elif self.modo == 'prueba':
            pl = self._plan
            i, dt = pl['i'], t - pl['t0']
            self.q_ref, self.dq_ref = pl['q0'].copy(), np.zeros(4)
            if self.tipo == 'escalon':
                self.q_ref[i] += pl['v']
            else:
                self.q_ref[i] += pl['v'] * min(dt, T_RAMPA)
                self.dq_ref[i] = pl['v'] if dt < T_RAMPA else 0.0
            self.q_ref = np.clip(self.q_ref, self.m.q_min, self.m.q_max)

    # ── Paso de control ───────────────────────────────────────────
    def paso(self, t, q, dq):
        """t [s], q y dq medidos (orden q0..q3). Devuelve tau (N, N·m, N·m, N·m)."""
        q, dq = np.asarray(q, dtype=float), np.asarray(dq, dtype=float)
        if self.t is None:
            # Primer ciclo: arranque sin saltos y de ahí a la postura de reposo
            self.t = t
            self.q_ref = q.copy()
            self.tipo = 'home'
            self._ir_a(t, self.q_home, 'acercamiento')
        dt = float(np.clip(t - self.t, 0.0, 0.05))
        self.t = t

        while self._pendientes:
            self._atender(self._pendientes.pop(0), t, q)
        self._referencia(t)

        self._p, J, G = self.m.evaluar(q)
        self._v = J @ dq
        self._q = q
        if not self.compensar_gravedad:
            G = np.zeros(4)
        for i in range(4):
            u = self.pid[i].paso(self.q_ref[i], self.dq_ref[i], q[i], dq[i], dt,
                                 u_ff=G[i])
            self.tau[i] = self.act[i].paso(u, dq[i], dt) if (self.act and dt > 0) else u
        return self.tau.copy()

    # ── Estado para la interfaz ───────────────────────────────────
    def estado(self):
        uv = self.plano.xyz_a_uv(self._p)[0]
        d = dict(
            t=self.t, tipo=self.tipo, fase=self.fase, s=self.s,
            p=self._p.tolist(), v=self._v.tolist(), rapidez=float(np.linalg.norm(self._v)),
            uv=uv.tolist(), dist_plano=float(self.plano.distancia(self._p)[0]),
            q=self._q.tolist(), q_ref=self.q_ref.tolist(), tau=self.tau.tolist(),
            p_ref=None, error=None,
        )
        if self._p_ref is not None:
            d['p_ref'] = self._p_ref.tolist()
            d['error'] = float(np.linalg.norm(self._p_ref - self._p))
        return d
