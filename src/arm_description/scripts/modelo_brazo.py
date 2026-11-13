#!/usr/bin/env python3
"""
modelo_brazo.py
===============
Modelo cinemático y dinámico del manipulador leído DIRECTAMENTE del xacro
(arm_description.xacro). No depende de ROS: solo numpy.

Lo usan:
  - control_torque.py        (nodo ROS 2: compensación de gravedad, FK, jacobiano)
  - diseno_pid.py            (inercias efectivas para sintonizar los PID)
  - simulacion_offline.py    (validación del lazo sin Gazebo)

Al tomar masas, inercias y ejes del mismo archivo que carga Gazebo, el modelo
del controlador y la planta simulada no pueden desalinearse.

Coordenadas generalizadas (orden fijo en todo el proyecto):
  q0  moviento_en_x            riel lineal            [m]
  q1  disco_soportes_rotacion  giro de la base        [rad]
  q2  brazo_motores            hombro                 [rad]
  q3  brazo_gripper_azul       codo                   [rad]

Marco de tarea {0} (el de la entrega 2): origen en el carro con q0 = 0,
X a lo largo del riel, Z hacia arriba, Y hacia el plano de trabajo.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = [
    'moviento_en_x',
    'disco_soportes_rotacion',
    'brazo_motores',
    'brazo_gripper_azul',
]
ETIQUETAS = ['q0 riel', 'q1 base', 'q2 hombro', 'q3 codo']
UNIDADES = ['m', 'rad', 'rad', 'rad']
LINK_EFECTOR = 'payload_400g'      # punto de agarre (centro de la carga)
G0 = 9.81


def ruta_xacro_por_defecto():
    """Busca el xacro junto al script (árbol fuente) o en share/ (instalado)."""
    aqui = os.path.dirname(os.path.realpath(__file__))
    candidatos = [
        os.path.join(aqui, '..', 'urdf', 'arm_description.xacro'),
        os.path.join(aqui, '..', '..', 'share', 'arm_description', 'urdf',
                     'arm_description.xacro'),
    ]
    for c in candidatos:
        if os.path.isfile(c):
            return os.path.normpath(c)
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('arm_description'),
                            'urdf', 'arm_description.xacro')
    except Exception as exc:  # pragma: no cover
        raise FileNotFoundError('No se encontró arm_description.xacro') from exc


# ──────────────────────────────────────────────────────────────────
# Utilidades de transformaciones
# ──────────────────────────────────────────────────────────────────
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _rot_eje(eje, ang):
    """Rodrigues."""
    x, y, z = eje
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def _vec(txt, n=3):
    return np.array([float(v) for v in txt.split()]) if txt else np.zeros(n)


# ──────────────────────────────────────────────────────────────────
# Modelo
# ──────────────────────────────────────────────────────────────────
class ModeloBrazo:
    def __init__(self, ruta_xacro=None):
        self.ruta = ruta_xacro or ruta_xacro_por_defecto()
        raiz = ET.parse(self.ruta).getroot()

        # ── Links: masa, centro de masa e inercia (en el marco del link) ──
        self.links = {}
        for ln in raiz.findall('link'):
            ine = ln.find('inertial')
            m, c, I = 0.0, np.zeros(3), np.zeros((3, 3))
            if ine is not None:
                m = float(ine.find('mass').get('value'))
                o = ine.find('origin')
                c = _vec(o.get('xyz')) if o is not None else np.zeros(3)
                i = ine.find('inertia')
                ixx, ixy, ixz = float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))
                iyy, iyz, izz = float(i.get('iyy')), float(i.get('iyz')), float(i.get('izz'))
                I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
            self.links[ln.get('name')] = dict(m=m, c=c, I=I)

        # ── Joints (solo los hijos directos de <robot>, no los de <ros2_control>) ──
        self.joints = {}
        self.hijos = {}
        for j in raiz.findall('joint'):
            o = j.find('origin')
            ax = j.find('axis')
            lim = j.find('limit')
            din = j.find('dynamics')
            d = dict(
                nombre=j.get('name'), tipo=j.get('type'),
                padre=j.find('parent').get('link'),
                hijo=j.find('child').get('link'),
                T0=_T(_vec(o.get('xyz')), _vec(o.get('rpy'))) if o is not None else np.eye(4),
                eje=_vec(ax.get('xyz')) if ax is not None else np.array([0.0, 0.0, 1.0]),
                lo=float(lim.get('lower', 0)) if lim is not None else 0.0,
                hi=float(lim.get('upper', 0)) if lim is not None else 0.0,
                esfuerzo=float(lim.get('effort', 0)) if lim is not None else 0.0,
                vel=float(lim.get('velocity', 0)) if lim is not None else 0.0,
                amort=float(din.get('damping', 0)) if din is not None else 0.0,
            )
            self.joints[d['nombre']] = d
            self.hijos.setdefault(d['padre'], []).append(d)

        self.q_min = np.array([self.joints[j]['lo'] for j in JOINTS])
        self.q_max = np.array([self.joints[j]['hi'] for j in JOINTS])
        self.tau_max = np.array([self.joints[j]['esfuerzo'] for j in JOINTS])
        self.dq_max = np.array([self.joints[j]['vel'] for j in JOINTS])
        self.amort = np.array([self.joints[j]['amort'] for j in JOINTS])

        # Links que arrastra cada articulación controlada
        self._desc = []
        for jn in JOINTS:
            self._desc.append(set(self._subarbol(self.joints[jn]['hijo'])))

        # ── Marco de tarea {0} ──
        # Origen: origen del carro (eje_x) con q = 0.  X: eje del riel.  Z: vertical.
        T = self.transformadas(np.zeros(4))
        ex = self._eje_mundo(T, 0)
        ez = np.array([0.0, 0.0, 1.0])
        ey = np.cross(ez, ex)
        self.R0 = np.column_stack([ex, ey, ez])       # {0} → mundo
        self.o0 = T['eje_x'][:3, 3].copy()

    # ── Árbol ─────────────────────────────────────────────────────
    def _subarbol(self, link):
        out = [link]
        for j in self.hijos.get(link, []):
            out += self._subarbol(j['hijo'])
        return out

    def transformadas(self, q):
        """Devuelve {link: T_mundo_link (4x4)} para q = [q0, q1, q2, q3]."""
        qd = dict(zip(JOINTS, q))
        T = {'world': np.eye(4)}
        pila = ['world']
        while pila:
            padre = pila.pop()
            for j in self.hijos.get(padre, []):
                Tj = T[padre] @ j['T0']
                v = qd.get(j['nombre'], 0.0)
                if j['tipo'] == 'prismatic':
                    M = np.eye(4)
                    M[:3, 3] = j['eje'] * v
                    Tj = Tj @ M
                elif j['tipo'] in ('revolute', 'continuous'):
                    M = np.eye(4)
                    M[:3, :3] = _rot_eje(j['eje'], v)
                    Tj = Tj @ M
                T[j['hijo']] = Tj
                pila.append(j['hijo'])
        return T

    def _eje_mundo(self, T, i):
        j = self.joints[JOINTS[i]]
        return T[j['hijo']][:3, :3] @ j['eje']

    def _origen_mundo(self, T, i):
        return T[self.joints[JOINTS[i]]['hijo']][:3, 3]

    def _columna(self, T, i, p):
        """∂p/∂q_i (lineal) y eje angular de la articulación i."""
        a = self._eje_mundo(T, i)
        if self.joints[JOINTS[i]]['tipo'] == 'prismatic':
            return a, np.zeros(3)
        return np.cross(a, p - self._origen_mundo(T, i)), a

    # ── Cinemática del efector (en el marco de tarea {0}) ─────────
    def efector(self, q):
        T = self.transformadas(q)
        return self.R0.T @ (T[LINK_EFECTOR][:3, 3] - self.o0)

    def jacobiano(self, q):
        """Jacobiano lineal 3x4 del efector, expresado en {0}."""
        T = self.transformadas(q)
        p = T[LINK_EFECTOR][:3, 3]
        J = np.column_stack([self._columna(T, i, p)[0] for i in range(4)])
        return self.R0.T @ J

    def evaluar(self, q):
        """Una sola pasada de cinemática: (p efector, jacobiano 3x4, G(q))."""
        T = self.transformadas(q)
        pm = T[LINK_EFECTOR][:3, 3]
        J = self.R0.T @ np.column_stack([self._columna(T, i, pm)[0] for i in range(4)])
        return self.R0.T @ (pm - self.o0), J, self.gravedad(q, T)

    # ── Dinámica ──────────────────────────────────────────────────
    def gravedad(self, q, T=None):
        """G(q): par/fuerza que debe aplicar cada actuador para sostener el brazo."""
        T = self.transformadas(q) if T is None else T
        G = np.zeros(4)
        for nombre, L in self.links.items():
            if L['m'] == 0.0 or nombre not in T:
                continue
            pc = T[nombre][:3, 3] + T[nombre][:3, :3] @ L['c']
            for i in range(4):
                if nombre in self._desc[i]:
                    G[i] += L['m'] * G0 * self._columna(T, i, pc)[0][2]
        return G

    def masa(self, q):
        """Matriz de inercia M(q) 4x4."""
        T = self.transformadas(q)
        M = np.zeros((4, 4))
        for nombre, L in self.links.items():
            if L['m'] == 0.0 or nombre not in T:
                continue
            R = T[nombre][:3, :3]
            pc = T[nombre][:3, 3] + R @ L['c']
            Jv = np.zeros((3, 4))
            Jw = np.zeros((3, 4))
            for i in range(4):
                if nombre in self._desc[i]:
                    Jv[:, i], Jw[:, i] = self._columna(T, i, pc)
            M += L['m'] * Jv.T @ Jv + Jw.T @ (R @ L['I'] @ R.T) @ Jw
        return M

    def coriolis(self, q, dq, h=1e-6):
        """Vector C(q, dq)·dq mediante diferencias centrales de M(q)."""
        dM = []
        for k in range(4):
            e = np.zeros(4)
            e[k] = h
            dM.append((self.masa(q + e) - self.masa(q - e)) / (2 * h))
        Mdot = sum(dM[k] * dq[k] for k in range(4))
        grad = np.array([dq @ dM[k] @ dq for k in range(4)])
        return Mdot @ dq - 0.5 * grad

    def masa_movil(self):
        return sum(self.links[n]['m'] for n in self._desc[0])


# ──────────────────────────────────────────────────────────────────
# Plano de trabajo y lugares geométricos (entrega 2)
# ──────────────────────────────────────────────────────────────────
class PlanoTrabajo:
    """
    Plano tamaño carta inclinado 45° frente al riel, en el marco {0}.
    Coordenadas del plano: u a lo largo de X (ancho), v sobre la pendiente.
    Esquinas: A=(0,0)  B=(w,0)  C=(w,h)  D=(0,h).
    """

    def __init__(self, ancho=0.2794, alto=0.2159, inclinacion=np.radians(45.0),
                 y0=0.24, z0=0.0, x_centro=0.08):
        self.w, self.h = ancho, alto
        self.c, self.s = np.cos(inclinacion), np.sin(inclinacion)
        self.y0, self.z0 = y0, z0
        self.x0 = x_centro - ancho / 2.0          # X de la arista izquierda
        self.x_centro = x_centro
        self._semiarco_maximo()

    def uv_a_xyz(self, uv):
        uv = np.atleast_2d(uv)
        return np.column_stack([self.x0 + uv[:, 0],
                                self.y0 + uv[:, 1] * self.c,
                                self.z0 + uv[:, 1] * self.s])

    def xyz_a_uv(self, p):
        p = np.atleast_2d(p)
        return np.column_stack([p[:, 0] - self.x0,
                                (p[:, 1] - self.y0) * self.c + (p[:, 2] - self.z0) * self.s])

    def distancia(self, p):
        """Distancia con signo del punto p al plano (>0 del lado de la normal)."""
        p = np.atleast_2d(p)
        return -(p[:, 1] - self.y0) * self.s + (p[:, 2] - self.z0) * self.c

    def esquinas_uv(self):
        return np.array([[0, 0], [self.w, 0], [self.w, self.h], [0, self.h]])

    def _semiarco_maximo(self):
        """Semicírculo más grande inscrito con diámetro sobre la diagonal A-C."""
        Ld = np.hypot(self.w, self.h)
        dh = np.array([self.w, self.h]) / Ld
        nh = np.array([-dh[1], dh[0]])
        th = np.linspace(0, np.pi, 181)
        mejor = (0.0, 0.0, 1)
        for lado in (1, -1):
            for t in np.linspace(0, Ld, 1201):
                lo, hi = 0.0, Ld
                for _ in range(40):
                    r = 0.5 * (lo + hi)
                    pts = (dh * t)[None, :] + r * (np.outer(np.cos(th), dh)
                                                   + lado * np.outer(np.sin(th), nh))
                    dentro = (np.all(pts[:, 0] >= -1e-12) and np.all(pts[:, 0] <= self.w + 1e-12)
                              and np.all(pts[:, 1] >= -1e-12) and np.all(pts[:, 1] <= self.h + 1e-12))
                    lo, hi = (r, hi) if dentro else (lo, r)
                if lo > mejor[0]:
                    mejor = (lo, t, lado)
        self.arco_r, t, self.arco_lado = mejor
        self._dh, self._nh = dh, nh
        self.arco_c = dh * t
        self.diag = Ld

    # Lugares geométricos parametrizados con s ∈ [0, 1]
    def recta(self, s):
        s = np.atleast_1d(s)
        return np.column_stack([self.w * s, self.h * s])

    def semiarco(self, s):
        s = np.atleast_1d(s)
        return self.arco_c[None, :] + self.arco_r * (
            -np.outer(np.cos(np.pi * s), self._dh)
            + self.arco_lado * np.outer(np.sin(np.pi * s), self._nh))

    def lugar(self, tipo, s):
        return self.recta(s) if tipo == 'recta' else self.semiarco(s)


def perfil_quintico(t, tf):
    """s(t) ∈ [0,1] con velocidad y aceleración nulas en los extremos."""
    x = np.clip(t / tf, 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / tf
    return s, ds


# ──────────────────────────────────────────────────────────────────
# Cinemática inversa
# ──────────────────────────────────────────────────────────────────
class CinematicaInversa:
    """
    Posición 3D con 4 GDL: la redundancia se resuelve haciendo que el riel
    siga una fracción kappa del movimiento en X (igual que en la entrega 2):
        q0 = q0_c + kappa·(x − x_c)
    Las tres articulaciones restantes se obtienen con Newton amortiguado sobre
    la cinemática directa del xacro, así no hay que suponer ceros ni signos.
    """

    def __init__(self, modelo, plano, kappa=0.5, q0_centro=0.15,
                 semilla=(0.15, 0.0, 0.6, -1.4)):
        self.m, self.plano, self.kappa, self.q0_c = modelo, plano, kappa, q0_centro
        self.semilla = np.array(semilla, dtype=float)

    def q0_de_x(self, x):
        q0 = self.q0_c + self.kappa * (x - self.plano.x_centro)
        return float(np.clip(q0, self.m.q_min[0], self.m.q_max[0]))

    def resolver(self, p, q_ini=None, tol=1e-7, iters=60):
        q = np.array(self.semilla if q_ini is None else q_ini, dtype=float)
        q[0] = self.q0_de_x(p[0])
        for _ in range(iters):
            e = p - self.m.efector(q)
            if np.linalg.norm(e) < tol:
                return q, True
            J = self.m.jacobiano(q)[:, 1:]
            dq = np.linalg.solve(J.T @ J + 1e-8 * np.eye(3), J.T @ e)
            paso = np.max(np.abs(dq))
            if paso > 0.3:
                dq *= 0.3 / paso
            q[1:] = np.clip(q[1:] + dq, self.m.q_min[1:], self.m.q_max[1:])
        return q, bool(np.linalg.norm(p - self.m.efector(q)) < 1e-4)

    def trayectoria(self, puntos):
        """Resuelve una lista de puntos encadenando la solución anterior como semilla."""
        Q, ok, q = [], True, None
        for p in puntos:
            q, bien = self.resolver(p, q)
            ok = ok and bien
            Q.append(q.copy())
        return np.array(Q), ok
