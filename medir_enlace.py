#!/usr/bin/env python3
"""
medir_enlace.py

TFG: Analisis de la conectividad y latencia en redes de UAV mediante
el protocolo MAVLink en entornos simulados.

Sonda de calidad de enlace MAVLink a traves del canal simulado en ns-3.

Metricas que registra:
  - RTT de aplicacion: PARAM_REQUEST_READ -> PARAM_VALUE
  - Perdida efectiva: peticiones sin respuesta dentro del timeout
  - Salud del flujo de telemetria: recuento y tasa de HEARTBEAT

Salidas (en el directorio indicado por --salida):
  <escenario>.csv        una fila por sondeo
  <escenario>.meta.json  metadatos de la corrida y resumen estadistico

Nota sobre relojes:
  Las duraciones se miden con time.monotonic(), inmune a los saltos de
  reloj de la maquina virtual. El reloj de pared (time.time()) se usa
  unicamente para registrar T0, que es el ancla comun para correlacionar
  despues con los registros de tiempo de simulacion de ns-3.
"""

import argparse
import csv
import json
import os
import signal
import statistics
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pymavlink no disponible. Revisa que no estas ejecutando "
             "como root sin el entorno del usuario (ver nota sobre sudo anidado).")


detener = False


def manejar_senal(signum, frame):
    global detener
    detener = True
    print("\n[!] Interrupcion recibida. Cerrando ficheros y resumiendo...")


def percentil(datos, p):
    """Percentil por interpolacion lineal. Evita depender de numpy."""
    if not datos:
        return None
    d = sorted(datos)
    if len(d) == 1:
        return d[0]
    k = (len(d) - 1) * (p / 100.0)
    inf = int(k)
    sup = min(inf + 1, len(d) - 1)
    if inf == sup:
        return d[inf]
    return d[inf] + (d[sup] - d[inf]) * (k - inf)


def construir_argumentos():
    ap = argparse.ArgumentParser(
        description="Sonda de RTT y perdida sobre enlace MAVLink degradado.")
    ap.add_argument("--conexion", default="udpin:0.0.0.0:14570",
                    help="Cadena de conexion pymavlink. Ej: udpin:0.0.0.0:14570")
    ap.add_argument("--escenario", default="sin_nombre",
                    help="Etiqueta de la corrida. Da nombre a los ficheros de salida.")
    ap.add_argument("--param", default="RC1_MIN",
                    help="Parametro usado como sonda de RTT.")
    ap.add_argument("--intervalo", type=float, default=1.0,
                    help="Segundos entre sondeos consecutivos.")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="Segundos de espera maxima por cada PARAM_VALUE.")
    ap.add_argument("--duracion", type=float, default=120.0,
                    help="Duracion total de la corrida en segundos.")
    ap.add_argument("--espera-inicial", type=float, default=10.0,
                    dest="espera_inicial",
                    help="Segundos de reposo tras el primer HEARTBEAT, para "
                         "dejar que MAVProxy termine su descarga de parametros.")
    ap.add_argument("--salida", default=os.path.expanduser("~/Escritorio/TFG/resultados"),
                    help="Directorio donde escribir los ficheros de salida.")
    ap.add_argument("--latencia-cfg", type=float, default=None, dest="latencia_cfg",
                    help="Latencia configurada en ns-3, solo para los metadatos.")
    ap.add_argument("--perdida-cfg", type=float, default=None, dest="perdida_cfg",
                    help="Perdida configurada en ns-3, solo para los metadatos.")
    ap.add_argument("--comprobar", action="store_true",
                    help="Solo verifica la conexion y sale. No mide ni escribe nada.")
    return ap.parse_args()


def conectar(args):
    print(f"[*] Abriendo conexion: {args.conexion}")
    conn = mavutil.mavlink_connection(args.conexion,
                                      source_system=200,
                                      source_component=190)

    print("[*] Esperando HEARTBEAT del vehiculo...")
    hb = conn.wait_heartbeat(timeout=30)
    if hb is None:
        sys.exit("ERROR: no se recibio HEARTBEAT en 30 s.\n"
                 "  Revisa que el canal ns-3 sigue vivo, que preparar_namespace.sh\n"
                 "  se ejecuto tras arrancar ns-3, y que el puente MAVProxy del\n"
                 "  namespace esta reenviando trafico.")

    print(f"[+] Enlace activo. Sistema {conn.target_system}, "
          f"componente {conn.target_component}, tipo {hb.type}, "
          f"autopiloto {hb.autopilot}")
    return conn


def sondear(conn, args, hb_contador):
    """Lanza un PARAM_REQUEST_READ y espera su PARAM_VALUE.

    Devuelve (rtt_ms | None, estado, hb_contador_actualizado).
    Los HEARTBEAT que llegan mientras esperamos se contabilizan, no se tiran.
    """
    t_envio = time.monotonic()
    conn.mav.param_request_read_send(
        conn.target_system,
        conn.target_component,
        args.param.encode("utf-8"),
        -1)

    while (time.monotonic() - t_envio) < args.timeout:
        if detener:
            break
        msg = conn.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.0005)
            continue

        tipo = msg.get_type()
        if tipo == "HEARTBEAT":
            hb_contador += 1
        elif tipo == "PARAM_VALUE":
            nombre = msg.param_id
            if isinstance(nombre, bytes):
                nombre = nombre.decode("utf-8", "ignore")
            if nombre.replace("\x00", "").strip() == args.param:
                rtt_ms = (time.monotonic() - t_envio) * 1000.0
                return rtt_ms, "ok", hb_contador

    return None, "timeout", hb_contador


def reposar(conn, args, t_envio_mono, hb_contador):
    """Consume el resto del ciclo sin dejar de contar HEARTBEAT."""
    fin_ciclo = t_envio_mono + args.intervalo
    while not detener and time.monotonic() < fin_ciclo:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.0005)
            continue
        if msg.get_type() == "HEARTBEAT":
            hb_contador += 1
    return hb_contador


def main():
    args = construir_argumentos()
    signal.signal(signal.SIGINT, manejar_senal)

    conn = conectar(args)

    if args.comprobar:
        print("[+] Comprobacion superada. El enlace responde.")
        return

    print(f"[*] Reposando {args.espera_inicial:.0f} s antes de medir...")
    time.sleep(args.espera_inicial)

    os.makedirs(args.salida, exist_ok=True)
    ruta_csv = os.path.join(args.salida, f"{args.escenario}.csv")
    ruta_meta = os.path.join(args.salida, f"{args.escenario}.meta.json")

    # T0: ancla temporal comun con los registros de ns-3.
    t0_wall = time.time()
    t0_mono = time.monotonic()

    rtts = []
    timeouts = 0
    hb_total = 0
    seq = 0

    print(f"[*] T0 = {t0_wall:.6f} (epoch UNIX)")
    print(f"[*] Midiendo {args.duracion:.0f} s. Ctrl+C para detener antes.\n")

    with open(ruta_csv, "w", newline="") as f:
        escritor = csv.writer(f)
        escritor.writerow(["seq", "t_unix", "t_rel_s", "rtt_ms",
                           "estado", "hb_total"])

        while not detener and (time.monotonic() - t0_mono) < args.duracion:
            seq += 1
            t_envio_mono = time.monotonic()
            t_envio_wall = time.time()

            rtt_ms, estado, hb_total = sondear(conn, args, hb_total)

            if estado == "ok":
                rtts.append(rtt_ms)
            else:
                timeouts += 1

            escritor.writerow([
                seq,
                f"{t_envio_wall:.6f}",
                f"{t_envio_mono - t0_mono:.6f}",
                f"{rtt_ms:.3f}" if rtt_ms is not None else "",
                estado,
                hb_total,
            ])
            f.flush()

            marca = f"{rtt_ms:7.2f} ms" if rtt_ms is not None else "  TIMEOUT"
            print(f"  [{seq:4d}] t={t_envio_mono - t0_mono:7.2f}s  "
                  f"rtt={marca}   hb={hb_total}")

            hb_total = reposar(conn, args, t_envio_mono, hb_total)

    duracion_real = time.monotonic() - t0_mono
    total = seq
    exitos = len(rtts)

    resumen = {
        "sondeos_totales": total,
        "respuestas_ok": exitos,
        "timeouts": timeouts,
        "tasa_timeout_pct": round(100.0 * timeouts / total, 3) if total else None,
        "heartbeats_recibidos": hb_total,
        "tasa_heartbeat_hz": round(hb_total / duracion_real, 3) if duracion_real else None,
        "rtt_ms": {
            "min": round(min(rtts), 3) if rtts else None,
            "media": round(statistics.mean(rtts), 3) if rtts else None,
            "mediana": round(statistics.median(rtts), 3) if rtts else None,
            "p95": round(percentil(rtts, 95), 3) if rtts else None,
            "max": round(max(rtts), 3) if rtts else None,
            "desv_tipica": round(statistics.pstdev(rtts), 3) if len(rtts) > 1 else None,
        },
    }

    meta = {
        "escenario": args.escenario,
        "t0_unix": t0_wall,
        "t0_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0_wall)),
        "duracion_solicitada_s": args.duracion,
        "duracion_real_s": round(duracion_real, 3),
        "conexion": args.conexion,
        "parametro_sonda": args.param,
        "intervalo_s": args.intervalo,
        "timeout_s": args.timeout,
        "canal_ns3": {
            "latencia_ms_configurada": args.latencia_cfg,
            "perdida_configurada": args.perdida_cfg,
        },
        "resumen": resumen,
    }

    with open(ruta_meta, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("\n" + "=" * 52)
    print(f"  Escenario:      {args.escenario}")
    print(f"  Sondeos:        {total}   OK: {exitos}   Timeouts: {timeouts}")
    if total:
        print(f"  Tasa timeout:   {100.0 * timeouts / total:.2f} %")
    if rtts:
        print(f"  RTT medio:      {statistics.mean(rtts):.2f} ms")
        print(f"  RTT mediana:    {statistics.median(rtts):.2f} ms")
        print(f"  RTT p95:        {percentil(rtts, 95):.2f} ms")
        print(f"  RTT min/max:    {min(rtts):.2f} / {max(rtts):.2f} ms")
    print(f"  HEARTBEAT:      {hb_total}  "
          f"({hb_total / duracion_real:.2f} Hz)" if duracion_real else "")
    print("=" * 52)
    print(f"\n  Datos:      {ruta_csv}")
    print(f"  Metadatos:  {ruta_meta}\n")


if __name__ == "__main__":
    main()
