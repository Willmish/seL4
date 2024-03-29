// Branched from riscv_plic0.h. Accomodates non-standard PLIC/HART mapping.
/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once
/* tell the kernel we have the set trigger feature */
#define HAVE_SET_TRIGGER 1


#include <plat/machine/devices_gen.h>
#include <arch/model/smp.h>

/* The memory map is based on the PLIC section in
 * https://static.dev.sifive.com/U54-MC-RVCoreIP.pdf
 */

#define PLIC_PPTR_BASE          PLIC_PPTR


// The SMC PLIC only services the SMC. From the standpoint of the PLIC,
// the SMC is HART 0, despite the SMC being HART 1 in the overall structure.
#define PLIC_HART_ID            0

#define PLIC_PRIO               0x0
#define PLIC_PRIO_PER_ID        0x4

#define PLIC_PENDING            0x1000
#define PLIC_EN                 0x2000
#define PLIC_EN_PER_HART        0x100
#define PLIC_EN_PER_CONTEXT     0x80


#define PLIC_THRES              0x200000
#if defined(CONFIG_PLAT_SHODAN)
#define PLIC_SVC_CONTEXT        1
#elif defined(CONFIG_PLAT_NEXUS)
#define PLIC_SVC_CONTEXT        0
#else
#error "Unknown/unsupported platform"
#endif
#define PLIC_THRES_PER_HART     0x2000
#define PLIC_THRES_PER_CONTEXT  0x1000
#define PLIC_THRES_CLAIM        0x4

#define PLIC_NUM_INTERRUPTS PLIC_MAX_IRQ

#if defined(CONFIG_PLAT_HIFIVE) || defined(CONFIG_PLAT_POLARFIRE)

/* SiFive U54-MC has 5 cores, and the first core does not
 * have supervisor mode. Therefore, we need to compensate
 * for the addresses.
 */
#define PLAT_PLIC_THRES_ADJUST(x) ((x) - PLIC_THRES_PER_CONTEXT)
#define PLAT_PLIC_EN_ADJUST(x)    ((x) - PLIC_EN_PER_CONTEXT)

#else

#define PLAT_PLIC_THRES_ADJUST(x)   (x)
#define PLAT_PLIC_EN_ADJUST(x)      (x)

#endif

static inline void write_sie(word_t value)
{
    asm volatile("csrw sie,  %0" :: "r"(value));
}

static inline word_t read_sie(void)
{
    word_t temp;
    asm volatile("csrr %0, sie" : "=r"(temp));
    return temp;
}

static inline uint32_t readl(word_t addr)
{
    return *((volatile uint32_t *)(addr));
}

static inline void writel(uint32_t val, word_t addr)
{
    *((volatile uint32_t *)(addr)) = val;
}

static inline word_t plic_enable_offset(word_t hart_id, word_t context_id)
{
    word_t addr = PLAT_PLIC_EN_ADJUST(PLIC_EN + hart_id * PLIC_EN_PER_HART + context_id * PLIC_EN_PER_CONTEXT);
    return addr;
}


static inline word_t plic_thres_offset(word_t hart_id, word_t context_id)
{
    word_t addr = PLAT_PLIC_THRES_ADJUST(PLIC_THRES + hart_id * PLIC_THRES_PER_HART + context_id * PLIC_THRES_PER_CONTEXT);
    return addr;
}

static inline word_t plic_claim_offset(word_t hart_id, word_t context_id)
{
    word_t addr = plic_thres_offset(hart_id, context_id) + PLIC_THRES_CLAIM;
    return addr;
}

static inline bool_t plic_pending_interrupt(word_t interrupt)
{
    word_t addr = PLIC_PPTR_BASE + PLIC_PENDING + (interrupt / 32) * 4;
    word_t bit = interrupt % 32;
    if (readl(addr) & BIT(bit)) {
        return true;
    } else {
        return false;
    }
}

static inline word_t get_hart_id(void)
{
#ifdef ENABLE_SMP_SUPPORT
    return cpuIndexToID(getCurrentCPUIndex());
#else
    return PLIC_HART_ID;
#endif
}

static inline irq_t plic_get_claim(void)
{
    /* Read the claim register for our HART interrupt context */
    word_t hart_id = get_hart_id();
    return readl(PLIC_PPTR_BASE + plic_claim_offset(hart_id, PLIC_SVC_CONTEXT));
}

static inline void plic_complete_claim(irq_t irq)
{
    /* Complete the IRQ claim by writing back to the claim register. */
    word_t hart_id = get_hart_id();
    writel(irq, PLIC_PPTR_BASE + plic_claim_offset(hart_id, PLIC_SVC_CONTEXT));
}

static inline void plic_mask_irq(bool_t disable, irq_t irq)
{
    uint64_t addr = 0;
    uint32_t val = 0;
    uint32_t bit = 0;

    word_t hart_id = get_hart_id();
    addr = PLIC_PPTR_BASE + plic_enable_offset(hart_id, PLIC_SVC_CONTEXT) + (irq / 32) * 4;
    bit = irq % 32;

    val = readl(addr);
    if (disable) {
        val &= ~BIT(bit);
    } else {
        val |= BIT(bit);
    }
    writel(val, addr);
}

static inline void plic_init_hart(void)
{

    word_t hart_id = get_hart_id();

    for (int i = 1; i <= PLIC_NUM_INTERRUPTS; i++) {
        /* Disable interrupts */
        plic_mask_irq(true, i);
    }

    /* Set threshold to zero */
    writel(0, (PLIC_PPTR_BASE + plic_thres_offset(hart_id, PLIC_SVC_CONTEXT)));
}

static inline void plic_init_controller(void)
{
#if !defined(CONFIG_PLAT_SHODAN)
    // TODO(b/215715756): This doesn't do anything on Rende other than cause noisy logs,
    //   so remove for Shodan (Renode-only). For nexus on Renode we live with the
    //   noise because it's required for real hardware.
    for (int i = 1; i <= PLIC_NUM_INTERRUPTS; i++) {
        /* Clear all pending bits */
        if (plic_pending_interrupt(i)) {
            readl(PLIC_PPTR_BASE + plic_claim_offset(PLIC_HART_ID, PLIC_SVC_CONTEXT));
            writel(i, PLIC_PPTR_BASE + plic_claim_offset(PLIC_HART_ID, PLIC_SVC_CONTEXT));
        }
    }
#endif // CONFIG_PLAT_SHODAN

    /* Set the priorities of all interrupts to 1 */
    for (int i = 1; i < PLIC_MAX_IRQ + 1; i++) {
        writel(2, PLIC_PPTR_BASE + PLIC_PRIO + PLIC_PRIO_PER_ID * i);
    }

}


/*
 * Provide a dummy definition of set trigger as the Hifive platform currently
 * has all global interrupt positive-level triggered.
 */
static inline void plic_irq_set_trigger(irq_t irq, bool_t edge_triggered)
{
}
