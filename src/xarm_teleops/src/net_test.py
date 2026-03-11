"""
net_test.py
-----------
Herramienta de diagnóstico de red para el sistema de teleoperación.

Mide RTT, jitter y throughput entre PC Maestro y PC Esclavo.
Genera estadísticas y recomendaciones para el reporte IEEE.

Uso:
    # En la PC que actúa como servidor (esclavo):
    python3 net_test.py server

    # En la PC que actúa como cliente (maestro):
    python3 net_test.py client <IP_SERVIDOR>

Salida:
    - Histograma de RTT (ASCII)
    - Tabla de estadísticas para la Pregunta A2 del reporte
    - Guardar resultados en data/net_test_results.csv
"""

import sys
import socket
import struct
import time
import os
import numpy as np

SERVER_PORT   = 9999
N_PACKETS     = 200
PACKET_SIZE   = 56    # bytes (igual que el paquete maestro→esclavo)
TIMEOUT_S     = 2.0


def run_server():
    """Servidor de eco UDP — refleja cada paquete recibido."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', SERVER_PORT))
    print(f"[net_test] Servidor UDP escuchando en puerto {SERVER_PORT}...")
    print("[net_test] Esperando cliente. Ctrl+C para salir.\n")
    try:
        while True:
            data, addr = sock.recvfrom(4096)
            sock.sendto(data, addr)   # eco inmediato
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print("[net_test] Servidor detenido.")


def run_client(server_ip: str):
    """Cliente UDP — mide RTT enviando paquetes de eco al servidor."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT_S)

    payload = bytes(PACKET_SIZE)   # paquete de ceros
    rtts_ms = []
    lost    = 0

    print(f"[net_test] Midiendo RTT con {server_ip}:{SERVER_PORT}")
    print(f"           {N_PACKETS} paquetes de {PACKET_SIZE} bytes...\n")

    for i in range(N_PACKETS):
        # Insertar timestamp en los primeros 8 bytes
        t_send = time.perf_counter()
        pkt = struct.pack('<d', t_send) + payload[8:]

        try:
            sock.sendto(pkt, (server_ip, SERVER_PORT))
            resp, _ = sock.recvfrom(4096)
            t_recv = time.perf_counter()
            rtt_ms = (t_recv - t_send) * 1000.0
            rtts_ms.append(rtt_ms)
        except socket.timeout:
            lost += 1

        time.sleep(0.01)   # 100 paquetes/s

    sock.close()

    if not rtts_ms:
        print("[net_test] ERROR: No se recibió ningún paquete.")
        return

    rtts = np.array(rtts_ms)
    Td   = rtts.mean() / 2.0   # retardo efectivo

    # ── Estadísticas ─────────────────────────────────────────────────────
    print("─" * 55)
    print("  Resultados de Test de Red UDP")
    print("─" * 55)
    print(f"  Paquetes enviados : {N_PACKETS}")
    print(f"  Paquetes recibidos: {len(rtts_ms)}")
    print(f"  Paquetes perdidos : {lost} ({100.0*lost/N_PACKETS:.1f}%)")
    print()
    print(f"  RTT promedio      : {rtts.mean():.3f} ms")
    print(f"  RTT mínimo        : {rtts.min():.3f} ms")
    print(f"  RTT máximo        : {rtts.max():.3f} ms")
    print(f"  Jitter (std)      : {rtts.std():.3f} ms")
    print(f"  P95               : {np.percentile(rtts, 95):.3f} ms")
    print(f"  P99               : {np.percentile(rtts, 99):.3f} ms")
    print()
    print(f"  Retardo efectivo T_d = RTT/2 = {Td:.3f} ms")

    # Criterio de pasividad de Colgate
    wc = 2.0 * np.pi * 50.0   # 50 Hz ancho de banda controlador interno
    Kd_max = np.pi / (2.0 * (Td / 1000.0) * wc)
    print(f"  K_d_max (pasividad Colgate, wc=50Hz) = {Kd_max:.1f} N/m")
    print("─" * 55)

    # ── Histograma ASCII ──────────────────────────────────────────────────
    print("\n  Histograma de RTT:")
    bins = np.linspace(rtts.min(), rtts.max(), 11)
    hist, _ = np.histogram(rtts, bins=bins)
    max_bar = 30
    for i in range(len(hist)):
        bar_len = int(hist[i] / hist.max() * max_bar) if hist.max() > 0 else 0
        label = f"{bins[i]:5.2f}-{bins[i+1]:5.2f} ms"
        print(f"  {label} | {'█' * bar_len} {hist[i]}")

    # ── Guardar CSV ───────────────────────────────────────────────────────
    os.makedirs('data', exist_ok=True)
    out_path = 'data/net_test_results.csv'
    header = 'rtt_ms'
    np.savetxt(out_path, rtts.reshape(-1, 1), delimiter=',',
               header=header, comments='')
    print(f"\n  Datos guardados en {out_path}")


def main():
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python3 net_test.py server")
        print("  python3 net_test.py client <IP_SERVIDOR>")
        sys.exit(1)

    mode = sys.argv[1].lower()
    if mode == 'server':
        run_server()
    elif mode == 'client':
        if len(sys.argv) < 3:
            print("ERROR: especifica la IP del servidor")
            sys.exit(1)
        run_client(sys.argv[2])
    else:
        print(f"ERROR: modo desconocido '{mode}'")
        sys.exit(1)


if __name__ == '__main__':
    main()
