#!/bin/bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo ip link set multicast on lo
