/* bl_irq_forward.c  --  Calliope campus bootloader safety net.
 *
 * DIRECTION 2: instead of dead-ending unhandled/mis-routed IRQs in `b .`
 * (Default_Handler in gcc_startup_nrf52833.S), re-vector the exception to the
 * APPLICATION`s vector table and branch to the app`s real handler.
 *
 * Rationale: after an app-only BLE DFU, the app may enable a peripheral IRQ
 * (e.g. TIMER3 cap-touch) that fires *before* the app has relocated VTOR /
 * restored the SoftDevice app-vector base. The MBR trampoline then forwards
 * the IRQ to the bootloader vector table (base 0x77000) where every external
 * IRQ slot points at the shared `b .` trap -> WEDGE. This helper returns the
 * app handler address so the asm Default_Handler can bounce to it instead of
 * hanging.
 */

#include <stdint.h>
#include "nrf.h"
#include "nrf_mbr.h"          /* MBR_SIZE (0x1000)      */
#include "nrf_sdm.h"          /* SD_SIZE_GET, SD_SIZE_OFFSET */
#include "nrf_bootloader_info.h" /* BOOTLOADER_START_ADDR (UICR-derived) */

/* nRF52833 flash page = 4 KiB. */
#ifndef BL_CODE_PAGE_SIZE
#define BL_CODE_PAGE_SIZE  (0x1000u)
#endif

/* Application vector-table base = first flash page after the SoftDevice.
 * Mirrors nrf_dfu_bank0_start_addr(): ALIGN_TO_PAGE(SD_SIZE_GET(MBR_SIZE)).
 * For S113 7.0.1 on the mini V3 this evaluates to 0x1C000. Computed at run
 * time so the fix stays correct for any SD size / future SD swap. */
static uint32_t bl_app_vector_base(void)
{
    uint32_t sd_size = SD_SIZE_GET(MBR_SIZE);
    sd_size = (sd_size + (BL_CODE_PAGE_SIZE - 1u)) & ~(BL_CODE_PAGE_SIZE - 1u);
    return sd_size;
}

/* Returns the address (with Thumb bit) of the application`s handler for the
 * currently active exception, or 0 if it cannot/should not be forwarded.
 * Called from the asm Default_Handler with the CPU still in Handler mode. */
uint32_t bl_default_irq_forward_target(void)
{
    uint32_t exc_num = __get_IPSR() & IPSR_ISR_Msk;   /* 16 == IRQ0 */

    /* Only forward genuine external interrupts. System exceptions
     * (NMI/HardFault/SVC/PendSV/SysTick, exc_num < 16) are not app-owned in
     * this scenario and must not be blindly re-vectored. */
    if (exc_num < 16u)
    {
        return 0u;
    }

    uint32_t app_base = bl_app_vector_base();

    /* app base must be in application flash, below the bootloader. Use the
     * UICR-derived BOOTLOADER_START_ADDR rather than a literal so the guard is
     * correct for both the release (0x77000) and debug (0x72000) layouts. */
    if (app_base == 0u || app_base >= BOOTLOADER_START_ADDR)
    {
        return 0u;
    }

    uint32_t handler = *(volatile uint32_t *)(app_base + (exc_num << 2));

    /* Reject a slot that is empty or points back into the bootloader (which
     * would just re-enter this trap). Require a valid Thumb code address in
     * the application flash region. */
    if ((handler & 1u) == 0u)            return 0u;   /* not a Thumb address */
    if (handler < app_base)              return 0u;
    if (handler >= BOOTLOADER_START_ADDR) return 0u;

    return handler;
}
