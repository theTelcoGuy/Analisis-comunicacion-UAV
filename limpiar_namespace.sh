#!/bin/bash
echo "Limpiando namespaces, puentes e interfaces..."
sudo ip netns delete ns-dronA 2>/dev/null || true
sudo ip netns delete ns-dronB 2>/dev/null || true
sudo ip link delete br-left 2>/dev/null || true
sudo ip link delete br-right 2>/dev/null || true
sudo ip link delete veth-hostA 2>/dev/null || true
sudo ip link delete veth-hostB 2>/dev/null || true
sudo ip tuntap del dev tap-left mode tap 2>/dev/null || true
sudo ip tuntap del dev tap-right mode tap 2>/dev/null || true
echo "Limpieza completada."
