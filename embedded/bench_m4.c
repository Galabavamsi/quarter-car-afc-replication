/* Cortex-M4 instruction-count benchmark under QEMU (-icount shift=0, so the
 * virtual clock advances exactly one tick per guest instruction; SysTick is
 * calibrated against a loop with a known instruction count). */
#include <stdio.h>
#include <stdint.h>
#include "qc_laws.h"
extern int main(void);
extern void __libc_init_array(void);
void _init(void) {}
void _fini(void) {}
extern uint32_t __etext, __data_start__, __data_end__, __bss_start__, __bss_end__, __stack_top;
extern void initialise_monitor_handles(void);
void Reset_Handler(void)
{
    *(volatile uint32_t *)0xE000ED88 |= (0xFu << 20);   /* enable FPU (no-op on soft-float builds) */
    __asm__ volatile("dsb\n isb");
    uint32_t *s = &__etext, *d = &__data_start__;
    while (d < &__data_end__) *d++ = *s++;
    for (d = &__bss_start__; d < &__bss_end__;) *d++ = 0;
    __libc_init_array();
    main();
    for (;;) __asm__ volatile("bkpt 0xab");
}
static void Default_Handler(void) { for (;;) ; }
__attribute__((section(".vectors"), used)) void (*const vectors[16])(void) = {
    (void (*)(void))&__stack_top, Reset_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, 0, 0, 0, 0, Default_Handler, Default_Handler, 0, Default_Handler, Default_Handler};

#define SYST_CSR (*(volatile uint32_t *)0xE000E010)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018)
static uint32_t ticks_since(uint32_t start) { return (start - SYST_CVR) & 0xFFFFFF; }

int main(void)
{
    initialise_monitor_handles();
    SYST_RVR = 0xFFFFFF; SYST_CVR = 0; SYST_CSR = 5;
    /* calibration: 2 instructions per iteration */
    uint32_t n = 200000, t0 = SYST_CVR;
    __asm__ volatile("1: subs %0, %0, #1\n bne 1b" : "+r"(n));
    double instr_per_tick = 400000.0 / ticks_since(t0);

    afc_t a = {1, 1, 1e-8, {0.2, 110, 100}, {0.018, 90, 80}, {3, 2, 2}, {25.5, 12, 216}};
    plant_t p = {290, 59, 16812, 0, 1000, 190000, 3.35e-4, 4.515e13, 1, 1.545e9, 10342500, 1e-6, 5e-4};
    bsc_t b = {{400, 100, 400}};
    pidc_t q = {3080, 200, 200, 0, 0.02, 5};
    real x[6] = {0.005, 0.02, 0.01, -0.03, 0.3, 1e-4};
    volatile real sink = 0;
    const int N = 200;
    double ipc[4];
    afc_fast_t af; afc_fast_init(&af, &a, (real)0.02);
    t0 = SYST_CVR; for (int i = 0; i < N; ++i) { x[0] = (real)(1e-5 * (i & 63)); sink += afc_law(&a, (real)(0.02 * i), x); } ipc[0] = instr_per_tick * ticks_since(t0) / N;
    t0 = SYST_CVR; for (int i = 0; i < N; ++i) { x[0] = (real)(1e-5 * (i & 63)); sink += bsc_law(&b, &p, x, 0.01); } ipc[1] = instr_per_tick * ticks_since(t0) / N;
    t0 = SYST_CVR; for (int i = 0; i < N; ++i) { x[0] = (real)(1e-5 * (i & 63)); sink += pid_law(&q, x); } ipc[2] = instr_per_tick * ticks_since(t0) / N;
    t0 = SYST_CVR; for (int i = 0; i < N; ++i) { x[0] = (real)(1e-5 * (i & 63)); sink += afc_fast_step(&af, x); } ipc[3] = instr_per_tick * ticks_since(t0) / N;
    printf("M4 %s: instr/tick %.2f | instructions per call AFC %.0f AFCfast %.0f BSC %.0f PID %.0f | self-check afc %.6g\n",
           sizeof(real) == 4 ? "float(soft)" : "double(soft)", instr_per_tick, ipc[0], ipc[3], ipc[1], ipc[2], (double)afc_law(&a, 0.5, (real[6]){0.005, 0.02, 0.01, -0.03, 0.3, 1e-4}));
    return 0;
}
