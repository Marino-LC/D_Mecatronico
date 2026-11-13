#!/usr/bin/env python3
"""
interfaz_trayectoria.py
=======================
Interfaz del prototipo (Parte 2). Muestra en tiempo real:
  - Tipo de trayectoria solicitada (RECTA o SEMIARCO) y su fase
  - Posición del órgano terminal (x, y, z en el marco {0}; u, v sobre el plano)
  - Velocidad del órgano terminal (vx, vy, vz y rapidez)
  - Trazo del efector sobre el plano tamaño carta contra la referencia

Uso con Gazebo (requiere control_torque.py corriendo):
    ros2 run arm_description interfaz_trayectoria.py

Uso sin ROS ni Gazebo (planta simulada en este mismo proceso):
    python3 interfaz_trayectoria.py --demo
"""

import json
import sys
import threading
import time

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QProgressBar, QPushButton, QVBoxLayout, QWidget)

from modelo_brazo import PlanoTrabajo

NOMBRES = {'recta': 'RECTA (diagonal A → C)', 'semiarco': 'SEMIARCO', 'home': 'HOME',
           'paro': 'PARO', 'escalon': 'ESCALÓN (prueba)', 'rampa': 'RAMPA (prueba)',
           'ninguna': '—'}
VENTANA_S = 20.0      # segundos visibles en la gráfica de rapidez


# ──────────────────────────────────────────────────────────────────
# Fuentes de datos
# ──────────────────────────────────────────────────────────────────
class FuenteROS:
    """Lee /trayectoria/estado y publica en /trayectoria/comando."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        rclpy.init(args=sys.argv)
        self._String = String
        self._estado = None
        self.nodo = Node('interfaz_trayectoria')
        self.pub = self.nodo.create_publisher(String, '/trayectoria/comando', 10)
        self.nodo.create_subscription(String, '/trayectoria/estado', self._cb, 10)
        threading.Thread(target=rclpy.spin, args=(self.nodo,), daemon=True).start()

    def _cb(self, msg):
        try:
            self._estado = json.loads(msg.data)
        except ValueError:
            pass

    def estado(self):
        return self._estado

    def comando(self, texto):
        self.pub.publish(self._String(data=texto))


class FuenteDemo:
    """Planta no lineal + el mismo controlador, en un hilo a tiempo real."""

    def __init__(self):
        from nucleo_control import ControlTorque
        from simulacion_offline import CADA, DT, Planta
        self.ctrl = ControlTorque()
        self.planta = Planta(self.ctrl.m, cada_din=4 * CADA)   # más ligero: tiempo real
        self._dt_ctrl = CADA * DT
        self._cada = CADA
        self._estado = None
        threading.Thread(target=self._bucle, daemon=True).start()

    def _bucle(self):
        t, t0 = 0.0, time.time()
        while True:
            tau = self.ctrl.paso(t, self.planta.q, self.planta.dq)
            self._estado = self.ctrl.estado()
            for _ in range(self._cada):
                self.planta.paso(tau)
            t += self._dt_ctrl
            falta = t - (time.time() - t0)
            if falta > 0:
                time.sleep(falta)

    def estado(self):
        return self._estado

    def comando(self, texto):
        self.ctrl.comando(texto)


# ──────────────────────────────────────────────────────────────────
# Vista del plano de trabajo
# ──────────────────────────────────────────────────────────────────
class VistaPlano(QWidget):
    def __init__(self, plano):
        super().__init__()
        self.plano = plano
        self.setMinimumSize(520, 420)
        s = np.linspace(0, 1, 120)
        self.refs = {t: plano.lugar(t, s) for t in ('recta', 'semiarco')}
        self.tipo = None
        self.traza = []
        self.punto = None
        self.sobre_plano = False

    def _a_pixel(self, uv, esc, ox, oy):
        return QPointF(ox + uv[0] * esc, oy - uv[1] * esc)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#1e272e'))
        m = 45
        esc = min((self.width() - 2 * m) / self.plano.w, (self.height() - 2 * m) / self.plano.h)
        ox = (self.width() - self.plano.w * esc) / 2
        oy = (self.height() + self.plano.h * esc) / 2
        px = lambda uv: self._a_pixel(uv, esc, ox, oy)

        p.setPen(QPen(QColor('#57606f'), 1))
        p.setBrush(QColor('#f5f0c4'))
        p.drawRect(QRectF(px((0, self.plano.h)), px((self.plano.w, 0))))

        p.setFont(QFont('Arial', 9))
        p.setPen(QColor('#d2dae2'))
        for nombre, uv, dx, dy in (('A', (0, 0), -16, 16), ('B', (self.plano.w, 0), 6, 16),
                                   ('C', (self.plano.w, self.plano.h), 6, -6),
                                   ('D', (0, self.plano.h), -16, -6)):
            q = px(uv)
            p.drawText(QPointF(q.x() + dx, q.y() + dy), nombre)
        p.drawText(QPointF(ox, oy + 34),
                   f'Plano carta {self.plano.w * 1e3:.1f} × {self.plano.h * 1e3:.1f} mm, inclinado 45°'
                   '   (u →,  v ↑ sobre la pendiente)')

        p.setBrush(Qt.NoBrush)
        for tipo, pts in self.refs.items():
            activa = tipo == self.tipo
            p.setPen(QPen(QColor('#2f3542' if activa else '#b2bec3'), 2 if activa else 1, Qt.DashLine))
            p.drawPolyline(QPolygonF([px(v) for v in pts]))

        if len(self.traza) > 1:
            p.setPen(QPen(QColor('#e84118'), 3))
            p.drawPolyline(QPolygonF([px(v) for v in self.traza]))

        if self.punto is not None:
            p.setPen(QPen(QColor('#192a56'), 1))
            p.setBrush(QColor('#e84118') if self.sobre_plano else QColor('#718093'))
            p.drawEllipse(px(self.punto), 6, 6)
        p.end()


class GraficaRapidez(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(130)
        self.datos = []          # (t, rapidez m/s)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#1e272e'))
        w, h, m = self.width(), self.height(), 28
        p.setPen(QPen(QColor('#57606f'), 1))
        p.drawRect(m, 8, w - m - 8, h - m - 4)
        p.setFont(QFont('Arial', 8))
        p.setPen(QColor('#d2dae2'))
        if not self.datos:
            p.end()
            return
        t1 = self.datos[-1][0]
        vmax = max(0.05, max(v for _, v in self.datos)) * 1.15
        p.drawText(2, 16, f'{vmax * 1e3:.0f}')
        p.drawText(2, h - m + 6, '0')
        p.drawText(m + 4, h - 6, f'|v| [mm/s] — últimos {VENTANA_S:.0f} s')
        pts = [QPointF(m + (1 - (t1 - t) / VENTANA_S) * (w - m - 8),
                       8 + (1 - v / vmax) * (h - m - 4)) for t, v in self.datos]
        p.setPen(QPen(QColor('#0be881'), 2))
        p.drawPolyline(QPolygonF(pts))
        p.end()


# ──────────────────────────────────────────────────────────────────
# Ventana principal
# ──────────────────────────────────────────────────────────────────
class Interfaz(QWidget):
    def __init__(self, fuente, plano=None, titulo_extra=''):
        super().__init__()
        self.fuente = fuente
        self.plano = plano or PlanoTrabajo()
        self._tipo_prev = None
        self._fase_prev = None
        self.setWindowTitle('Manipulador con riel — control de torque' + titulo_extra)
        self.setStyleSheet('QWidget{background:#2f3640; color:#f5f6fa; font-size:13px;}'
                           'QGroupBox{border:1px solid #57606f; border-radius:6px; margin-top:10px;'
                           ' padding:8px;} QGroupBox::title{subcontrol-origin:margin; left:10px;}')

        raiz = QHBoxLayout(self)
        izq = QVBoxLayout()
        raiz.addLayout(izq, 3)
        der = QVBoxLayout()
        raiz.addLayout(der, 2)

        # ── Izquierda: plano + rapidez ──
        self.vista = VistaPlano(self.plano)
        izq.addWidget(self.vista, 4)
        self.grafica = GraficaRapidez()
        izq.addWidget(self.grafica, 1)

        # ── Derecha: trayectoria ──
        g = QGroupBox('Trayectoria solicitada')
        l = QVBoxLayout(g)
        self.lbl_tipo = QLabel('—')
        self.lbl_tipo.setStyleSheet('color:#fbc531; font-size:24px; font-weight:bold;')
        self.lbl_fase = QLabel('Sin datos del controlador')
        self.barra = QProgressBar()
        self.barra.setRange(0, 1000)
        self.barra.setFormat('avance del trazo %p%')
        self.barra.setStyleSheet('QProgressBar{border:1px solid #57606f; border-radius:4px;'
                                 ' text-align:center;} QProgressBar::chunk{background:#44bd32;}')
        l.addWidget(self.lbl_tipo)
        l.addWidget(self.lbl_fase)
        l.addWidget(self.barra)
        der.addWidget(g)

        mono = QFont('DejaVu Sans Mono', 13)
        g = QGroupBox('Posición del órgano terminal  [mm]')
        l = QGridLayout(g)
        self.pos = {}
        for k, n in enumerate(('x', 'y', 'z', 'u', 'v')):
            a = QLabel(n + ' =')
            b = QLabel('—')
            b.setFont(mono)
            b.setAlignment(Qt.AlignRight)
            l.addWidget(a, k, 0)
            l.addWidget(b, k, 1)
            self.pos[n] = b
        self.lbl_plano = QLabel('')
        l.addWidget(self.lbl_plano, 5, 0, 1, 2)
        der.addWidget(g)

        g = QGroupBox('Velocidad del órgano terminal  [mm/s]')
        l = QGridLayout(g)
        self.vel = {}
        for k, n in enumerate(('vx', 'vy', 'vz', '|v|')):
            a = QLabel(n + ' =')
            b = QLabel('—')
            b.setFont(mono)
            b.setAlignment(Qt.AlignRight)
            l.addWidget(a, k, 0)
            l.addWidget(b, k, 1)
            self.vel[n] = b
        der.addWidget(g)

        g = QGroupBox('Seguimiento y pares')
        l = QVBoxLayout(g)
        self.lbl_error = QLabel('Error cartesiano: —')
        self.lbl_tau = QLabel('τ: —')
        self.lbl_tau.setFont(QFont('DejaVu Sans Mono', 10))
        l.addWidget(self.lbl_error)
        l.addWidget(self.lbl_tau)
        der.addWidget(g)

        fila = QGridLayout()
        for k, (txt, cmd, color) in enumerate((('RECTA', 'recta', '#0097e6'),
                                               ('SEMIARCO', 'semiarco', '#8c7ae6'),
                                               ('HOME', 'home', '#e1b12c'),
                                               ('PARO', 'paro', '#c23616'))):
            b = QPushButton(txt)
            b.setMinimumHeight(44)
            b.setStyleSheet(f'background:{color}; color:white; font-weight:bold; border-radius:5px;')
            b.clicked.connect(lambda _, c=cmd: self.fuente.comando(c))
            fila.addWidget(b, k // 2, k % 2)
        der.addLayout(fila)
        der.addStretch()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(50)

    def actualizar(self):
        e = self.fuente.estado()
        if e is None:
            return
        tipo, fase = e['tipo'], e['fase']
        if tipo != self._tipo_prev or (fase == 'acercamiento' and self._fase_prev != 'acercamiento'):
            self.vista.traza = []
        self._tipo_prev, self._fase_prev = tipo, fase

        self.lbl_tipo.setText(NOMBRES.get(tipo, tipo))
        self.lbl_fase.setText(f'Fase: {fase}      t = {e["t"]:.2f} s')
        en_trazo = tipo in ('recta', 'semiarco')
        self.barra.setValue(int(1000 * e['s']) if en_trazo and fase in ('trazando', 'terminada') else 0)

        x, y, z = e['p']
        u, v = e['uv']
        for n, val in zip(('x', 'y', 'z', 'u', 'v'), (x, y, z, u, v)):
            self.pos[n].setText(f'{val * 1e3:+9.2f}')
        self.lbl_plano.setText(f'Distancia al plano: {e["dist_plano"] * 1e3:+.2f} mm')
        for n, val in zip(('vx', 'vy', 'vz', '|v|'), (*e['v'], e['rapidez'])):
            self.vel[n].setText(f'{val * 1e3:+9.2f}')
        self.lbl_error.setText('Error cartesiano: ' + (f'{e["error"] * 1e3:.2f} mm'
                                                       if e['error'] is not None else '—'))
        tau = e['tau']
        self.lbl_tau.setText(f'τ riel {tau[0]:+6.2f} N   base {tau[1]:+5.2f}\n'
                             f'τ hombro {tau[2]:+5.2f}   codo {tau[3]:+5.2f} N·m')

        self.vista.tipo = tipo if en_trazo else None
        self.vista.punto = (u, v)
        self.vista.sobre_plano = abs(e['dist_plano']) < 0.005
        if fase == 'trazando':
            self.vista.traza.append((u, v))
        self.vista.update()

        self.grafica.datos.append((e['t'], e['rapidez']))
        while self.grafica.datos and self.grafica.datos[0][0] < e['t'] - VENTANA_S:
            self.grafica.datos.pop(0)
        self.grafica.update()


def main():
    demo = '--demo' in sys.argv
    app = QApplication([a for a in sys.argv if a != '--demo'])
    fuente = FuenteDemo() if demo else FuenteROS()
    gui = Interfaz(fuente, titulo_extra='  [DEMO sin Gazebo]' if demo else '')
    gui.resize(1050, 640)
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
