#!/bin/bash
#
# preparar_namespace_v2.sh
#
# Aisla cada extremo del canal simulado en su propio network namespace.
#
# La topologia mete dos routers de ns-3
# entre los dos extremos, asi NO estan en la misma subred:
#
#   ns-dronA  192.168.30.10/24  --> pasarela 192.168.30.1  (router izq)
#   ns-dronB  192.168.31.10/24  --> pasarela 192.168.31.1  (router der)
#
# Se usa ruta especifica y no ruta por defecto para no
# interferir con el trafico de las veth.
#
# IMPORTANTE: ejecutar SOLO despues de ver en la terminal de ns-3 el
# mensaje "Interfaces TAP creadas". Antes de eso las TAP no existen.

set -e

# --- Namespace + tap para dron A ---
sudo ip netns add ns-dronA
sudo ip link set tap-left netns ns-dronA
sudo ip netns exec ns-dronA sysctl -w net.ipv6.conf.tap-left.disable_ipv6=1
sudo ip netns exec ns-dronA ip addr add 192.168.30.10/24 dev tap-left
sudo ip netns exec ns-dronA ip link set tap-left up
sudo ip netns exec ns-dronA ip link set lo up

# --- Namespace + tap para dron B ---
sudo ip netns add ns-dronB
sudo ip link set tap-right netns ns-dronB
sudo ip netns exec ns-dronB sysctl -w net.ipv6.conf.tap-right.disable_ipv6=1
sudo ip netns exec ns-dronB ip addr add 192.168.31.10/24 dev tap-right
sudo ip netns exec ns-dronB ip link set tap-right up
sudo ip netns exec ns-dronB ip link set lo up

# --- Rutas a traves del canal simulado ---
sudo ip netns exec ns-dronA ip route add 192.168.31.0/24 via 192.168.30.1 dev tap-left
sudo ip netns exec ns-dronB ip route add 192.168.30.0/24 via 192.168.31.1 dev tap-right

# --- Veth: raiz <-> ns-dronA ---
sudo ip link add veth-hostA type veth peer name veth-nsA
sudo ip link set veth-nsA netns ns-dronA
sudo ip addr add 10.0.0.1/30 dev veth-hostA
sudo ip link set veth-hostA up
sudo ip netns exec ns-dronA ip addr add 10.0.0.2/30 dev veth-nsA
sudo ip netns exec ns-dronA ip link set veth-nsA up

# --- Veth: raiz <-> ns-dronB ---
sudo ip link add veth-hostB type veth peer name veth-nsB
sudo ip link set veth-nsB netns ns-dronB
sudo ip addr add 10.0.1.1/30 dev veth-hostB
sudo ip link set veth-hostB up
sudo ip netns exec ns-dronB ip addr add 10.0.1.2/30 dev veth-nsB
sudo ip netns exec ns-dronB ip link set veth-nsB up

echo ""
echo "Namespaces listos."
echo "  ns-dronA: tap-left  192.168.30.10  ->  192.168.31.10 via 192.168.30.1"
echo "  ns-dronB: tap-right 192.168.31.10  ->  192.168.30.10 via 192.168.31.1"
echo ""
echo "Comprueba con:"
echo "  sudo ip netns exec ns-dronA ping -c 10 192.168.31.10"
echo ""
