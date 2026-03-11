"""
network_comm.py
---------------
Comunicación UDP entre PC Maestro y PC Esclavo.

Protocolo de paquetes (little-endian, IEEE 754 float64):

  Master → Slave  (96 bytes):
    [0]     timestamp  (float64, segundos UNIX)
    [1..6]  q[0..5]    (float64 × 6, ángulos articulares rad)

  Slave → Master  (144 bytes):
    [0]     timestamp  (float64)
    [1..6]  F_ext[0..5](float64 × 6, fuerza/torque estimado N, Nm)
    [7..12] tau[0..5]  (float64 × 6, torques medidos Nm)

Uso maestro:
    tx = MasterSender('192.168.1.20', 5005)
    rx = SlaveReceiver(port=5006, timeout=0.01)
    tx.send(q_rad)
    data = rx.recv()

Uso esclavo:
    rx = MasterReceiver(port=5005, timeout=0.01)
    tx = SlaveSender('192.168.1.10', 5006)
    data = rx.recv()
    tx.send(F_ext, tau_meas)
"""

import socket
import struct
import time
import numpy as np
from typing import Optional, Tuple

# Formato de estructura:  '<' little-endian,  'd' = float64
_MASTER_FMT = '<' + 'd' * 7    # timestamp + 6 joints
_SLAVE_FMT  = '<' + 'd' * 13   # timestamp + 6 F_ext + 6 tau

_MASTER_SIZE = struct.calcsize(_MASTER_FMT)   # 56 bytes
_SLAVE_SIZE  = struct.calcsize(_SLAVE_FMT)    # 104 bytes

_SOCK_BUF = 4096


# ── Sender genérico ───────────────────────────────────────────────────────

class _UDPSender:
    def __init__(self, remote_ip: str, remote_port: int):
        self.addr = (remote_ip, remote_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self):
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class _UDPReceiver:
    def __init__(self, local_port: int, timeout: float = 0.05):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', local_port))
        self._sock.settimeout(timeout)

    def close(self):
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


# ── Master sender / Slave receiver ────────────────────────────────────────

class MasterSender(_UDPSender):
    """Envía posición articular del maestro al esclavo."""

    def send(self, q_rad: np.ndarray) -> None:
        payload = struct.pack(
            _MASTER_FMT, time.time(), *np.asarray(q_rad, dtype=float)
        )
        self._sock.sendto(payload, self.addr)


class MasterReceiver(_UDPReceiver):
    """Recibe posición articular del maestro (lado esclavo)."""

    def recv(self) -> Optional[Tuple[float, np.ndarray]]:
        """
        Returns:
            (timestamp, q_rad)  o  None si timeout
        """
        try:
            data, _ = self._sock.recvfrom(_SOCK_BUF)
        except socket.timeout:
            return None
        if len(data) != _MASTER_SIZE:
            return None
        fields = struct.unpack(_MASTER_FMT, data)
        return fields[0], np.array(fields[1:], dtype=float)


# ── Slave sender / Master receiver ────────────────────────────────────────

class SlaveSender(_UDPSender):
    """Envía fuerza estimada + torques medidos al maestro."""

    def send(self, F_ext: np.ndarray, tau_meas: np.ndarray) -> None:
        payload = struct.pack(
            _SLAVE_FMT,
            time.time(),
            *np.asarray(F_ext, dtype=float),
            *np.asarray(tau_meas, dtype=float),
        )
        self._sock.sendto(payload, self.addr)


class SlaveReceiver(_UDPReceiver):
    """Recibe fuerza estimada + torques del esclavo (lado maestro)."""

    def recv(self) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
        """
        Returns:
            (timestamp, F_ext, tau_meas)  o  None si timeout
        """
        try:
            data, _ = self._sock.recvfrom(_SOCK_BUF)
        except socket.timeout:
            return None
        if len(data) != _SLAVE_SIZE:
            return None
        fields = struct.unpack(_SLAVE_FMT, data)
        return fields[0], np.array(fields[1:7]), np.array(fields[7:])


# ── Utilidad: medición de RTT ─────────────────────────────────────────────

def measure_rtt(remote_ip: str, n_samples: int = 100) -> dict:
    """
    Mide RTT entre este PC y remote_ip usando socket UDP eco (puerto 7).
    Nota: el puerto 7 (echo) debe estar habilitado en el host remoto,
    o usa la función con un servidor echo personalizado.

    Alternativa más confiable: usar `ping` del OS.

    Returns:
        dict con 'mean_ms', 'std_ms', 'min_ms', 'max_ms', 'p99_ms'
    """
    import subprocess
    result = subprocess.run(
        ['ping', '-c', str(n_samples), '-i', '0.01', remote_ip],
        capture_output=True, text=True, timeout=30
    )
    rtts = []
    for line in result.stdout.splitlines():
        if 'time=' in line:
            rtt_str = line.split('time=')[1].split(' ')[0]
            rtts.append(float(rtt_str))
    if not rtts:
        return {}
    rtts = np.array(rtts)
    return {
        'mean_ms': float(rtts.mean()),
        'std_ms':  float(rtts.std()),
        'min_ms':  float(rtts.min()),
        'max_ms':  float(rtts.max()),
        'p99_ms':  float(np.percentile(rtts, 99)),
        'n_samples': len(rtts),
    }


def print_rtt_stats(stats: dict) -> None:
    print("── RTT Statistics ──────────────────────────────")
    print(f"  Mean   : {stats.get('mean_ms', 0):.2f} ms")
    print(f"  Std    : {stats.get('std_ms', 0):.2f} ms  (jitter)")
    print(f"  Min    : {stats.get('min_ms', 0):.2f} ms")
    print(f"  Max    : {stats.get('max_ms', 0):.2f} ms")
    print(f"  P99    : {stats.get('p99_ms', 0):.2f} ms")
    print(f"  n      : {stats.get('n_samples', 0)}")
    Td = stats.get('mean_ms', 0) / 2.0
    # Criterio de pasividad de Colgate: K_d_max < pi/(2*Td*wc)
    # Asumiendo wc = 2*pi*50 rad/s (50 Hz ancho de banda controlador)
    wc = 2.0 * np.pi * 50.0
    if Td > 0:
        Kd_max = np.pi / (2.0 * (Td / 1000.0) * wc)
        print(f"  Retardo T_d = {Td:.2f} ms → K_d_max ≈ {Kd_max:.1f} N/m (pasividad Colgate)")
    print("─────────────────────────────────────────────────")
