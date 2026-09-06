#!/usr/bin/env python3
"""
mision_h4.py

TFG: Analisis de la conectividad y latencia en redes de UAV mediante
el protocolo MAVLink en entornos simulados.

Hito H4 - parte A: carga y arranque de una mision de vuelo.

Sustituye a DroneKit (proyecto sin mantenimiento desde 2020) usando
unicamente pymavlink, ya presente en el entorno.

Que hace:
  1. Espera al vehiculo y lee su posicion actual (origen del cuadrado)
  2. Construye una mision: despegue, cuadrado de N metros, vuelta a casa
  3. Sube la mision siguiendo el protocolo MISSION correctamente
     y CRONOMETRA la subida  <-- esta es la metrica H4a
  4. Pasa a GUIDED, arma, despega y espera a alcanzar altura
  5. Pasa a AUTO para que se ejecute la mision

Salidas:
  <escenario>_mision.meta.json  tiempo de subida, reintentos, waypoints

Notas de protocolo (errores tipicos que este script evita):
  - Tras MISSION_COUNT el autopiloto responde MISSION_REQUEST_INT,
    NO MISSION_ACK. El ACK llega al final de la transferencia.
  - Cada elemento debe enviarse con su numero de secuencia correcto,
    y el autopiloto puede volver a pedir uno ya enviado.
  - El elemento 0 es, por convenio, la posicion HOME.
  - Un COMMAND_ACK significa "orden aceptada", no "tarea terminada".
    Por eso el despegue se confirma leyendo la altura, no el ACK.
"""

import argparse
import json
import math
import os
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pymavlink no disponible. Revisa el usuario con el que ejecutas.")

# Modos personalizados de ArduCopter
MODO_AUTO = 3
MODO_GUIDED = 4

FRAME_GLOBAL = mavutil.mavlink.MAV_FRAME_GLOBAL
FRAME_REL = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT


def construir_argumentos():
    ap = argparse.ArgumentParser(description="Carga y arranca una mision de vuelo.")
    ap.add_argument("--conexion", default="udpin:0.0.0.0:14551",
                    help="Cadena de conexion pymavlink.")
    ap.add_argument("--escenario", default="C0",
                    help="Etiqueta del escenario de canal (C0, C1, C2, C3).")
    ap.add_argument("--altura", type=float, default=30.0,
                    help="Altura de crucero en metros.")
    ap.add_argument("--lado", type=float, default=200.0,
                    help="Lado del cuadrado en metros.")
    ap.add_argument("--salida", default=os.path.expanduser("~/Escritorio/TFG/resultados"),
                    help="Directorio de salida para los metadatos.")
    ap.add_argument("--timeout-item", type=float, default=25.0, dest="timeout_item",
                    help="Espera maxima por cada peticion de elemento, en s.")
    ap.add_argument("--solo-carga", action="store_true", dest="solo_carga",
                    help="Sube la mision y sale, sin armar ni despegar.")
    return ap.parse_args()


def desplazar(lat, lon, norte_m, este_m):
    """Devuelve (lat, lon) desplazados N metros al norte y E metros al este."""
    radio = 6378137.0
    dlat = norte_m / radio
    dlon = este_m / (radio * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def conectar(cadena):
    print(f"[*] Conectando a {cadena}")
    conn = mavutil.mavlink_connection(cadena, source_system=201, source_component=190)
    print("[*] Esperando HEARTBEAT...")
    if conn.wait_heartbeat(timeout=40) is None:
        sys.exit("ERROR: sin HEARTBEAT en 40 s. Revisa el canal y el rele.")
    print(f"[+] Vehiculo {conn.target_system}:{conn.target_component}")
    return conn


def leer_posicion(conn, timeout=30):
    """Lee la primera posicion global valida."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg and msg.lat != 0:
            return msg.lat / 1e7, msg.lon / 1e7
    sys.exit("ERROR: no se recibio GLOBAL_POSITION_INT valido.")


def construir_mision(lat0, lon0, altura, lado):
    """Devuelve la lista de elementos de la mision.

    Cada elemento es un diccionario. El indice en la lista es su seq.
      seq 0 : HOME (convenio del protocolo)
      seq 1 : despegue
      seq 2-5: esquinas del cuadrado
      seq 6 : vuelta a casa
    """
    m = lado / 2.0
    esquinas = [
        desplazar(lat0, lon0, +m, +m),
        desplazar(lat0, lon0, +m, -m),
        desplazar(lat0, lon0, -m, -m),
        desplazar(lat0, lon0, -m, +m),
    ]

    items = []
    items.append(dict(frame=FRAME_GLOBAL, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      lat=lat0, lon=lon0, alt=0.0, current=1))
    items.append(dict(frame=FRAME_REL, cmd=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                      lat=lat0, lon=lon0, alt=altura, current=0))
    for la, lo in esquinas:
        items.append(dict(frame=FRAME_REL, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          lat=la, lon=lo, alt=altura, current=0))
    items.append(dict(frame=FRAME_REL, cmd=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                      lat=0.0, lon=0.0, alt=0.0, current=0))
    return items


def enviar_item(conn, item, seq):
    conn.mav.mission_item_int_send(
        conn.target_system, conn.target_component,
        seq,
        item["frame"],
        item["cmd"],
        item["current"],
        1,                      # autocontinue
        0, 0, 0, 0,             # param1..param4
        int(round(item["lat"] * 1e7)),
        int(round(item["lon"] * 1e7)),
        float(item["alt"]),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)


def subir_mision(conn, items, timeout_item):
    """Sube la mision y devuelve (segundos, reenvios, peticiones)."""
    total = len(items)
    print(f"[*] Subiendo mision de {total} elementos...")

    t0 = time.monotonic()
    conn.mav.mission_count_send(conn.target_system, conn.target_component,
                                total, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    enviados = set()
    peticiones = 0
    reenvios = 0

    while True:
        msg = conn.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True, timeout=timeout_item)

        if msg is None:
            raise TimeoutError(
                f"Sin respuesta en {timeout_item:.0f} s. "
                f"Enviados {len(enviados)}/{total}.")

        tipo = msg.get_type()

        if tipo == "MISSION_ACK":
            transcurrido = time.monotonic() - t0
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mision rechazada, codigo {msg.type}")
            print(f"[+] Mision aceptada en {transcurrido:.3f} s "
                  f"({peticiones} peticiones, {reenvios} reenvios)")
            return transcurrido, reenvios, peticiones

        seq = msg.seq
        peticiones += 1
        if seq in enviados:
            reenvios += 1
            print(f"    [!] Reenvio del elemento {seq}")
        if seq >= total:
            continue
        enviar_item(conn, items[seq], seq)
        enviados.add(seq)


def fijar_modo(conn, modo, nombre, timeout=15):
    print(f"[*] Cambiando a modo {nombre}...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            modo, 0, 0, 0, 0, 0)
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and hb.custom_mode == modo:
            print(f"[+] Modo {nombre} confirmado")
            return True
    raise RuntimeError(f"No se pudo fijar el modo {nombre}")


def armar(conn, timeout=30):
    print("[*] Armando...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[+] Armado")
            return True
    raise RuntimeError("No se pudo armar el vehiculo")


def despegar(conn, altura, timeout=90):
    """Ordena despegue y espera a alcanzar el 95 % de la altura.

    El COMMAND_ACK solo confirma que la orden se acepto. La unica forma
    fiable de saber que el despegue termino es leer la altura relativa.
    """
    print(f"[*] Despegando a {altura:.0f} m...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altura)

    objetivo = altura * 0.95
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        print(f"    altura {rel:6.1f} m", end="\r")
        if rel >= objetivo:
            print(f"\n[+] Altura alcanzada: {rel:.1f} m")
            return True
    raise RuntimeError("Tiempo agotado durante el despegue")


def main():
    args = construir_argumentos()
    conn = conectar(args.conexion)

    lat0, lon0 = leer_posicion(conn)
    print(f"[+] Origen: {lat0:.7f}, {lon0:.7f}")

    items = construir_mision(lat0, lon0, args.altura, args.lado)
    t_subida, reenvios, peticiones = subir_mision(conn, items, args.timeout_item)

    os.makedirs(args.salida, exist_ok=True)
    meta = {
        "escenario": args.escenario,
        "t0_unix": time.time(),
        "origen": {"lat": lat0, "lon": lon0},
        "altura_m": args.altura,
        "lado_m": args.lado,
        "elementos": len(items),
        "carga_mision": {
            "segundos": round(t_subida, 4),
            "peticiones_recibidas": peticiones,
            "reenvios": reenvios,
        },
    }

    if not args.solo_carga:
        fijar_modo(conn, MODO_GUIDED, "GUIDED")
        armar(conn)
        despegar(conn, args.altura)
        fijar_modo(conn, MODO_AUTO, "AUTO")
        meta["mision_iniciada_unix"] = time.time()
        print("\n[+] Mision en curso. El vehiculo navega de forma autonoma.")

    ruta = os.path.join(args.salida, f"{args.escenario}_mision.meta.json")
    with open(ruta, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"\n  Carga de mision: {t_subida:.3f} s")
    print(f"  Reenvios:        {reenvios}")
    print(f"  Metadatos:       {ruta}\n")


if __name__ == "__main__":
    main()
