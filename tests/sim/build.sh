#!/usr/bin/env bash
# Compila el simulador de host del firmware (usa CNC_V2.ino sin modificarlo).
# Uso: tests/sim/build.sh [salida]   (por defecto: /tmp/cnc_sim/cnc_sim)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-/tmp/cnc_sim/cnc_sim}"
mkdir -p "$(dirname "$OUT")"
g++ -std=gnu++17 -O1 -g -Wall -Wno-unused-function -Wno-unused-variable \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DPIN_ESTOP_INPUT=33 -I"$HERE/mock" -o "$OUT" "$HERE/sim_main.cpp" -lcrypto -pthread
echo "OK -> $OUT"
