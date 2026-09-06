#!/usr/bin/env python3
"""
registrar_posicion.py

TFG: Analisis de la conectividad y latencia en redes de UAV mediante
el protocolo MAVLink en entornos simulados.

Hito H4 - parte B: registro pasivo de la posicion del vehiculo.

Se ejecutan DOS instancias simultaneas:

  Observador A  ->  en el espacio raiz, colgado de MAVProxy.
                    Los mensajes NO cruzan el canal simulado.
                    Es la posicion REAL (verdad de referencia).

  Observador B  ->  dentro de ns-dronB, tras el canal simulado.
                    Es la posicion PERCIBIDA por la estacion de tierra.

La diferencia entre ambas series, emparejada por marca de tiempo,
cuantifica el impacto del canal en metros y en segundos.

Este script solo escucha: no envia nada, para no alterar la medida.

Uso:
  python3 registrar_posicion.py --etiqueta C1_A --conexion udpin:0.0.0.0:14570
  python3 registrar_posicion.py --etiqueta C1_B --conexion udpin:0.0.0.0:14550
"""

import argparse
import csv
import json
import os
import signal
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pymavlink no disponible. Revisa el usuario con el que ejecutas.")

detener = False


def manejar_senal(signum, frame):
    global detener
    detener = True
    print("\n[!] Interrupcion recibida. Cerrando fichero...")


def construir_argumentos():
    ap = argparse.ArgumentParser(description="Registra la posicion del vehiculo.")
    ap.add_argument("--etiqueta", required=True,
                    help="Nombre del registro, p.ej. C1_A o C1_B.")
    ap.add_argument("--conexion", required=True,
                    help="Cadena de conexion pymavlink.")
    ap.add_argument("--duracion", type=float, default=600.0,
                    help="Duracion maxima del registro en segundos.")
    ap.add_argument("--salida", default=os.path.expanduser("~/Escritorio/TFG/resultados"),
                    help="Directorio de salida.")
    return ap.parse_args()


def main():
    args = construir_argumentos()
    signal.signal(signal.SIGINT, manejar_senal)

    print(f"[*] Abriendo {args.conexion}")
    conn = mavutil.mavlink_connection(args.conexion,
                                      source_system=202,
                                      source_component=190)

    print("[*] Esperando HEARTBEAT...")
    if conn.wait_heartbeat(timeout=60) is None:
        sys.exit("ERROR: sin HEARTBEAT en 60 s.")
    print(f"[+] Vehiculo {conn.target_system}:{conn.target_component}")

    os.makedirs(args.salida, exist_ok=True)
    ruta_csv = os.path.join(args.salida, f"{args.etiqueta}_pos.csv")
    ruta_meta = os.path.join(args.salida, f"{args.etiqueta}_pos.meta.json")

    # T0: ancla temporal comun a los dos observadores.
    t0_wall = time.time()
    t0_mono = time.monotonic()

    n = 0
    hb = 0
    print(f"[*] T0 = {t0_wall:.6f}")
    print(f"[*] Registrando hasta {args.duracion:.0f} s. Ctrl+C para parar.\n")

    with open(ruta_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t_unix", "t_rel_s", "time_boot_ms",
                    "lat", "lon", "alt_msl_m", "alt_rel_m",
                    "vx_ms", "vy_ms", "vz_ms", "rumbo_deg"])

        while not detener and (time.monotonic() - t0_mono) < args.duracion:
            msg = conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue

            tipo = msg.get_type()
            if tipo == "HEARTBEAT":
                hb += 1
                continue
            if tipo != "GLOBAL_POSITION_INT":
                continue

            ahora_mono = time.monotonic()
            n += 1
            w.writerow([
                n,
                f"{time.time():.6f}",
                f"{ahora_mono - t0_mono:.6f}",
                msg.time_boot_ms,
                f"{msg.lat / 1e7:.7f}",
                f"{msg.lon / 1e7:.7f}",
                f"{msg.alt / 1000.0:.3f}",
                f"{msg.relative_alt / 1000.0:.3f}",
                f"{msg.vx / 100.0:.3f}",
                f"{msg.vy / 100.0:.3f}",
                f"{msg.vz / 100.0:.3f}",
                f"{msg.hdg / 100.0:.2f}" if msg.hdg != 65535 else "",
            ])
            f.flush()

            if n % 20 == 0:
                print(f"  [{n:5d}] t={ahora_mono - t0_mono:7.1f}s  "
                      f"alt={msg.relative_alt / 1000.0:6.1f} m  hb={hb}")

    duracion = time.monotonic() - t0_mono
    meta = {
        "etiqueta": args.etiqueta,
        "conexion": args.conexion,
        "t0_unix": t0_wall,
        "t0_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0_wall)),
        "duracion_s": round(duracion, 3),
        "muestras_posicion": n,
        "tasa_posicion_hz": round(n / duracion, 3) if duracion else None,
        "heartbeats": hb,
        "tasa_heartbeat_hz": round(hb / duracion, 3) if duracion else None,
    }
    with open(ruta_meta, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("\n" + "=" * 46)
    print(f"  Etiqueta:   {args.etiqueta}")
    print(f"  Muestras:   {n}   ({n / duracion:.2f} Hz)")
    print(f"  HEARTBEAT:  {hb}   ({hb / duracion:.2f} Hz)")
    print(f"  Duracion:   {duracion:.1f} s")
    print("=" * 46)
    print(f"\n  Datos:     {ruta_csv}")
    print(f"  Metadatos: {ruta_meta}\n")


if __name__ == "__main__":
    main()
