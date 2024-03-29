/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <arch/kernel/machine_isr.h>
#include <arch/kernel/machine_timer.h>
#include <arch/kernel/machine_uart.h>
#include <arch/kernel/preboot.h>
#include <stdint.h>

uint32_t machine_bad_isr(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                         uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7);
uint32_t machine_isr(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                     uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7);

// Machine-mode interrupt handler. THIS IS NOT A COMPLETE IMPLEMENTATION OF AN
// INTERRUPT HANDLER. It's just enough to handle calls to SBI_SET_TIMER and
// SBI_CONSOLE_PUTCHAR. More functionality can be added as needed.

__attribute__((naked, aligned(256))) void machine_vector_table() {
  asm volatile(
      ".option push       \n"
      ".option norvc      \n"
      "j machine_isr_wrapper \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      "j machine_bad_isr      \n"
      ".option pop        \n");
}

// We could potentially remove a bunch of this reg saving/loading, but it gets
// tricky due to caller/callee register saving conventions (see sbi_call).

void machine_isr_wrapper(void);
__attribute__((naked)) void machine_isr_wrapper() {
  asm volatile(
      "csrrw sp, mscratch, sp  \n"

      "sw x0,   0*4(sp)        \n"
      "sw x1,   1*4(sp)        \n"
      "sw x2,   2*4(sp)        \n"
      "sw x3,   3*4(sp)        \n"
      "sw x4,   4*4(sp)        \n"
      "sw x5,   5*4(sp)        \n"
      "sw x6,   6*4(sp)        \n"
      "sw x7,   7*4(sp)        \n"
      "sw x8,   8*4(sp)        \n"
      "sw x9,   9*4(sp)        \n"
      "sw x10, 10*4(sp)        \n"
      "sw x11, 11*4(sp)        \n"
      "sw x12, 12*4(sp)        \n"
      "sw x13, 13*4(sp)        \n"
      "sw x14, 14*4(sp)        \n"
      "sw x15, 15*4(sp)        \n"
      "sw x16, 16*4(sp)        \n"
      "sw x17, 17*4(sp)        \n"
      "sw x18, 18*4(sp)        \n"
      "sw x19, 19*4(sp)        \n"
      "sw x20, 20*4(sp)        \n"
      "sw x21, 21*4(sp)        \n"
      "sw x22, 22*4(sp)        \n"
      "sw x23, 23*4(sp)        \n"
      "sw x24, 24*4(sp)        \n"
      "sw x25, 25*4(sp)        \n"
      "sw x26, 26*4(sp)        \n"
      "sw x27, 27*4(sp)        \n"
      "sw x28, 28*4(sp)        \n"
      "sw x29, 29*4(sp)        \n"
      "sw x30, 30*4(sp)        \n"
      "sw x31, 31*4(sp)        \n"

      "csrr t0, mepc           \n"
      "addi t0, t0, 4          \n"
      "csrw mepc, t0           \n"
#if defined(CONFIG_PLAT_NEXUS)
      // Setup a0 so machine_isr can alter register state.
      "mv a0, sp               \n"
#endif
      "jal machine_isr            \n"

      "lw x0,   0*4(sp)        \n"
      "lw x1,   1*4(sp)        \n"
      "lw x2,   2*4(sp)        \n"
      "lw x3,   3*4(sp)        \n"
      "lw x4,   4*4(sp)        \n"
      "lw x5,   5*4(sp)        \n"
      "lw x6,   6*4(sp)        \n"
      "lw x7,   7*4(sp)        \n"
      "lw x8,   8*4(sp)        \n"
      "lw x9,   9*4(sp)        \n"
#if defined(CONFIG_PLAT_NEXUS)
      // On non-Nexus platforms do not reload x10 (aka a0), as
      // it contains the isr return value.
      "lw x10, 10*4(sp)        \n"
#endif
      "lw x11, 11*4(sp)        \n"
      "lw x12, 12*4(sp)        \n"
      "lw x13, 13*4(sp)        \n"
      "lw x14, 14*4(sp)        \n"
      "lw x15, 15*4(sp)        \n"
      "lw x16, 16*4(sp)        \n"
      "lw x17, 17*4(sp)        \n"
      "lw x18, 18*4(sp)        \n"
      "lw x19, 19*4(sp)        \n"
      "lw x20, 20*4(sp)        \n"
      "lw x21, 21*4(sp)        \n"
      "lw x22, 22*4(sp)        \n"
      "lw x23, 23*4(sp)        \n"
      "lw x24, 24*4(sp)        \n"
      "lw x25, 25*4(sp)        \n"
      "lw x26, 26*4(sp)        \n"
      "lw x27, 27*4(sp)        \n"
      "lw x28, 28*4(sp)        \n"
      "lw x29, 29*4(sp)        \n"
      "lw x30, 30*4(sp)        \n"
      "lw x31, 31*4(sp)        \n"

      "csrrw sp, mscratch, sp  \n"
      "mret                    \n");
}

uint32_t machine_bad_isr(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                         uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7) {
  machine_assert(false, "machine_bad_isr\n");
  return 0;
}

uint32_t machine_isr(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                     uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7) {
#if defined(CONFIG_PLAT_NEXUS)
  // NB: a0 points to the register state saved on the stack (see above).
  uint32_t *regs = (uint32_t*)a0;
  a0 = regs[10];
#endif // defined(CONFIG_PLAT_NEXUS)
  const uint32_t ENOSYS = 38; /* Invalid system call number */
  uint32_t mcause, mtval, mepc;
  asm volatile("csrr %0, mcause" : "=r"(mcause) : :);
  asm volatile("csrr %0, mtval" : "=r"(mtval) : :);
  asm volatile("csrr %0, mepc" : "=r"(mepc) : :);
  if (mcause & 0x80000000) {
    machine_assert(false, "machine_isr unhandled interrupt\n");
    return -ENOSYS;
  }

  if (mcause == 9) {
    if (a7 == 0) {
      // SBI_SET_TIMER
      opentitan_timer_ack();
      uint64_t count = ((uint64_t)a0 | (((uint64_t)a1) << 32));
      opentitan_timer_set_deadline(count);
      return 0;
    } else if (a7 == 1) {
      // SBI_CONSOLE_PUTCHAR
      machine_putchar((uint8_t)a0);
      return 0;
    } else if (a7 == 8) {
      // SBI_SHUTDOWN
      machine_printf("machine_isr sbi shutdown\n");
      while (1) {
      }
      return -ENOSYS;
    } else {
      machine_assert(false, "machine_isr bad sbi call\n");
      return -ENOSYS;
    }
#if defined(CONFIG_PLAT_NEXUS)
  } else if (mcause == 2) {
#define MATCH_RDTIME 0xc0102073
#define MATCH_RDTIMEH 0xc8102073
#define MASK_RDTIME  0xfffff07f
#define MASK_RDTIMEH  0xfffff07f
    // Emulate rdtime* for the Ibex (Renode's cpu mode implements these
    // directly so they are never used on that platform).
    int rd = (mtval >> 7) & 0x1f;
    if ((mtval & MASK_RDTIME) == MATCH_RDTIME) {
      uint64_t time = opentitan_timer_get_count();
      uint32_t ret = (uint32_t)(time & 0xFFFFFFFF);
      *(regs + rd) = ret;
    } else if ((mtval & MASK_RDTIMEH) == MATCH_RDTIMEH) {
      uint64_t time = opentitan_timer_get_count();
      uint32_t ret = (uint32_t)((time >> 32) & 0xFFFFFFFF);
      *(regs + rd) = ret;
    } else {
      machine_assert(false, "machine_isr unhandled illegal instruction\r\n");
    }
#undef MASK_RDTIMEH
#undef MASK_RDTIME
#undef MATCH_RDTIMEH
#undef MATCH_RDTIME
    return 0;
#endif // defined(CONFIG_PLAT_NEXUS)
  } else {
    machine_printf("mcause 0x%x mtval 0x%x mepc 0x%x\n", mcause, mtval, mepc);
    machine_assert(false, "machine_isr unhandled mcause\n");
  }

  return 0;
}
