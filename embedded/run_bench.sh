#!/usr/bin/env bash
# Reproduce the control-law timing study (Linux; needs gcc, arm-none-eabi-gcc,
# libnewlib-arm-none-eabi and qemu-system-arm, e.g. `sudo apt install gcc-arm-none-eabi
# libnewlib-arm-none-eabi qemu-system-arm`). Writes results_m4.txt.
set -eu
cd "$(dirname "$0")"
{
echo "# Control-law cost, Na et al. AFC vs BSC vs PID ($(date -u +%F))"
echo "## host (x86-64, gcc -O2)"
gcc -O2 -o bench_host bench_host.c qc_laws.c -lm && ./bench_host
gcc -O2 -DQC_REAL=float -o bench_host_f bench_host.c qc_laws.c -lm && ./bench_host_f
echo "## Cortex-M4 under QEMU mps2-an386, -icount shift=0 (instruction counts)"
for R in double float; do
  arm-none-eabi-gcc -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -DQC_REAL=$R --specs=rdimon.specs \
    -nostartfiles -T m4.ld -o bench_m4_$R.elf bench_m4.c qc_laws.c -lm 2>/dev/null
  timeout 120 qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -semihosting -icount shift=0 \
    -kernel bench_m4_$R.elf 2>/dev/null | head -1 || true
done
arm-none-eabi-gcc -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DQC_REAL=float \
  --specs=rdimon.specs -nostartfiles -T m4.ld -o bench_m4_hf.elf bench_m4.c qc_laws.c -lm 2>/dev/null
timeout 120 qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -semihosting -icount shift=0 \
  -kernel bench_m4_hf.elf 2>/dev/null | head -1 | sed 's/float(soft)/float(FPU)/' || true
} | tee results_m4.txt
