"""
net_test.py — Prueba de conectividad UDP entre maestro y esclavo.
Ejecutar en AMBAS computadoras para verificar la red antes del examen.

    En PC-A (receptor): python3 net_test.py --mode server
    En PC-B (emisor):   python3 net_test.py --mode client --ip <IP-PC-A>
"""

import socket
import time
import argparse
import statistics

PORT      = 9999
N_PACKETS = 100   # número de paquetes de prueba


def run_server():
    """
    Servidor UDP: recibe paquetes y mide latencia de ida y vuelta (RTT).
    Responde inmediatamente con un eco para que el cliente mida RTT.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))
    print(f"[SERVIDOR] Esperando paquetes en puerto {PORT}...")
    count = 0
    while count < N_PACKETS:
        try:
            data, addr = sock.recvfrom(256)
            sock.sendto(data, addr)   # eco inmediato
            count += 1
            if count % 10 == 0:
                print(f"[SERVIDOR] {count}/{N_PACKETS} paquetes procesados")
        except KeyboardInterrupt:
            break
    print(f"[SERVIDOR] {count} paquetes procesados. ¡Conectividad OK!")
    sock.close()


def run_client(server_ip):
    """
    Cliente UDP: envía paquetes de prueba y mide RTT.
    Calcula estadísticas: mínimo, máximo, promedio, desviación estándar.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)   # timeout de 1 segundo por paquete
    rtts = []
    lost = 0

    print(f"[CLIENTE] Enviando {N_PACKETS} paquetes a {server_ip}:{PORT}...")
    for i in range(N_PACKETS):
        payload = f"PING_{i:04d}_{time.time():.6f}".encode()
        t0      = time.perf_counter()
        sock.sendto(payload, (server_ip, PORT))
        try:
            resp, _ = sock.recvfrom(256)
            rtt_ms  = (time.perf_counter() - t0) * 1000.0
            rtts.append(rtt_ms)
        except socket.timeout:
            lost += 1
        time.sleep(0.01)   # 100 Hz (igual que el loop de control)

    sock.close()

    if rtts:
        print(f"\n{'='*55}")
        print(f"  Paquetes enviados   : {N_PACKETS}")
        print(f"  Paquetes perdidos   : {lost} ({100*lost/N_PACKETS:.1f}%)")
        print(f"  RTT mínimo          : {min(rtts):.2f} ms")
        print(f"  RTT máximo          : {max(rtts):.2f} ms")
        print(f"  RTT promedio        : {statistics.mean(rtts):.2f} ms")
        print(f"  Desv. estándar RTT  : {statistics.stdev(rtts):.2f} ms")
        print(f"{'='*55}")

        mean_rtt  = statistics.mean(rtts)
        loss_rate = lost / N_PACKETS

        # Evaluación para control de tiempo real (criterio del documento)
        if mean_rtt < 10.0 and loss_rate < 0.01:
            print("✅ RED APTA para control de impedancia (< 10 ms, < 1% pérdida)")
            print(f"   Frecuencia de Nyquist efectiva: {500/mean_rtt:.0f} Hz")
        elif mean_rtt < 20.0 and loss_rate < 0.05:
            print("⚠️  RED MARGINAL — reducir Kd a la mitad (Kd ≤ 200 N/m)")
            print("   Recomendación: usar cable en lugar de WiFi")
        else:
            print("❌ RED INADECUADA — revisar conexión WiFi o usar cable")
            print("   RTT > 20 ms o > 5% pérdida → inestabilidad posible")

        # Análisis de jitter
        if len(rtts) > 1:
            jitter = statistics.stdev(rtts)
            print(f"\n  Jitter (σ_RTT): {jitter:.2f} ms", end='')
            if jitter < 2.0:
                print("  ✅ Jitter bajo — aceptable")
            else:
                print("  ⚠️  Jitter alto — puede causar oscilaciones en fuerza")
    else:
        print("ERROR: Sin respuesta del servidor. Verificar IP y firewall.")
        print(f"  → Asegúrate que el servidor esté corriendo en {server_ip}:{PORT}")
        print("  → Verificar: sudo ufw allow 9999/udp")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Prueba de red UDP para TE3001B",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  PC-A (receptor):  python3 net_test.py --mode server
  PC-B (emisor):    python3 net_test.py --mode client --ip 192.168.1.105
  Local (una PC):   python3 net_test.py --mode server &
                    python3 net_test.py --mode client --ip 127.0.0.1
        """
    )
    parser.add_argument("--mode", choices=["server", "client"],
                        required=True, help="Modo de ejecución")
    parser.add_argument("--ip", default="127.0.0.1",
                        help="IP del servidor (solo en modo cliente)")
    args = parser.parse_args()

    if args.mode == "server":
        run_server()
    else:
        run_client(args.ip)
