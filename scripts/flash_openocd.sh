#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${ROOT}/build/RelWithDebInfo/$(basename "${ROOT}").elf"

if [[ ! -f "${ELF}" ]]; then
  echo "Firmware image not found: ${ELF}" >&2
  exit 1
fi

openocd \
  -f interface/stlink.cfg \
  -f target/stm32g4x.cfg \
  -c "transport select hla_swd" \
  -c "adapter speed 4000" \
  -c "program ${ELF} verify reset exit"
