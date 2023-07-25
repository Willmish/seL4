cmake_minimum_required(VERSION 3.7.2)

declare_platform(nexus KernelPlatformNexus PLAT_NEXUS KernelArchRiscV)

if(KernelPlatformNexus)
    declare_seL4_arch(riscv32)
    config_set(KernelRiscVPlatform RISCV_PLAT "nexus")
    config_set(KernelPlatformFirstHartID FIRST_HART_ID 1)
    # Shodan has no SBI
    config_set(UseRiscVOpenSBI RISCV_OPENSBI OFF)
    config_set(UseRiscVBBL RISCV_BBL OFF)
    list(APPEND KernelDTSList "tools/dts/nexus.dts")
    include_directories("${CMAKE_BINARY_DIR}/opentitan-gen/include")
    declare_default_headers(
        # TODO(mattharvey): [rdtime_sync] When switching to ibex, rdtime will
        # cause a trap, which will end up reading the OpenTitan timer. After
        # implementing that, this can be set to any desired value, but until
        # then it needs to be set to the rate at which the value returned by
        # cpu1 rdtime is incremented.
        TIMER_FREQUENCY 1000000

        INTERRUPT_CONTROLLER drivers/irq/smc_plic.h
        # Must be >= kTopMatchaPlicIrqIdLastSmc in top_matcha.h
        PLIC_MAX_NUM_INT 43
    )
else()
    unset(KernelPlatformFirstHartID CACHE)
endif()
