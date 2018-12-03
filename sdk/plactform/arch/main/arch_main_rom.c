/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "arch.h"      // architetural platform definitions
#include <stddef.h>    // standard definitions
#include "boot.h"      // boot definition
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#include "uart.h"      // UART initialization
#include "flash.h"     // Flash initialization
#include "reg_remap.h" // remap register definitions
#include "reg_auxio.h" // auxio register definitions
#include "hci.h"       // hci functions

#include "nvds.h"         // NVDS definitions

/**
 *****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 ******************************************************************************************/

/*
 * DEFINES
 ****************************************************************************************
 */
/// NVDS location in FLASH : 0x000E0000 (896KB (1Mo - 128KB))
#define NVDS_FLASH_ADDRESS          (0x000E0000)

/// NVDS size in RAM : 0x00010000 (128KB)
#define NVDS_FLASH_SIZE             (0x00000800)

typedef void (* VOIDFUNCTPTR)(void);

#define ROM_LED                   0x01
#define ROM_STAY_IN_ROM_BIT       0x01

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if PLF_DEBUG
/// Variable to enable infinite loop on assert
volatile int dbg_assert_block = 1;
#endif //PLF_DEBUG
uint32_t led_timer = 0;

#if (PLF_DEBUG)

void assert_param(int param0, int param1, const char * file, int line)
{
    GLOBAL_INT_STOP();
    while(dbg_assert_block);
}

void assert_err(const char *condition, const char * file, int line)
{
    GLOBAL_INT_STOP();
    while(dbg_assert_block);
}
#endif //PLF_DEBUG

static void rom_jump_to_firmware(void)
{
    VOIDFUNCTPTR p_reset;

    // Jump in RAM at address 0 (Reset vector of the firmware we just copied to RAM)
    p_reset = (void *)(0x0);
    p_reset();
}

void rom_led_toggle(void)
{
    // Toggling of LED : 10% on, 90% off
    if (led_timer < (400 *(      10)))
    {
        auxio_auxioa_out_set(auxio_auxioa_out_get() | ROM_LED);
    }
    else
    {
        auxio_auxioa_out_set(auxio_auxioa_out_get() & ~ROM_LED);
    }

    led_timer++;
    if ( led_timer >= (400*100))
    {
        led_timer = 0;
    }
}


void platform_reset(uint32_t error)
{
    void (*pReset)(void);

    // Disable interrupts
    GLOBAL_INT_STOP();

    // Wait UART transfer finished
    uart_finish_transfers();

    #ifndef CFG_ROM
    // Store information in unloaded area
    unloaded_area->error = error;
    #endif //CFG_ROM

    if(error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
    {
        // Not yet supported
    }
    else
    {
        // Restart FW
        pReset = (void * )(0x0);
        pReset();
    }
}

/**
 ****************************************************************************************
 * @brief Main BLE Ref IP entry point.
 *
 * This function is called right after the booting process has completed.
 ****************************************************************************************
 */
void rw_main(void)
{
    // Check if FLASH has to be skipped or not
    if (!(auxio_auxiob_in_get() & ROM_STAY_IN_ROM_BIT))
    {
        // Now Remap from ROM to EMI
        remap_remap_rom_setf(0);

        // And jump to 0, to reboot from EMI space
        rom_jump_to_firmware();

        // We should never reach this line, it is just for safety
        while(1);
    }

    // Initialize the exchange memory interface
    emi_init();

    // Initialize the UART
    uart_init();

    // Initialize the Interrupt Controller
    intc_init();

    // Initialize Flash component
    flash_init();

    // Initialize NVDS module
    nvds_init((uint8_t *)NVDS_FLASH_ADDRESS, NVDS_FLASH_SIZE);

    // Initialize the System Controller
    syscntl_init();

    // Initialize the HCI
    hci_init();

    // finally start interrupt handling
    GLOBAL_INT_START();

    // And loop forever
    for (;;)
    {
        // Toggle the LEDs periodically
        rom_led_toggle();
    }
}

/// @} DRIVERS
