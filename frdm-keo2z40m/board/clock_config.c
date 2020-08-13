/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/
/*
 * How to setup clock using clock driver functions:
 *
 * 1. call CLOCK_SetSimSafeDivs() to set the system clock dividers in SIM to safe value.
 *
 * 2. If external oscillator is used Call CLOCK_SetXtal0Freq() to set XTAL0 frequency based on board settings and
 *    call CLOCK_InitOsc0() to init the OSC.
 *
 * 3. Call CLOCK_BootToXxxMode()/CLOCK_SetXxxMode() to set ICS run at the target mode.
 *
 * 4. If ICSIRCLK is needed, call CLOCK_SetInternalRefClkConfig() to enable the clock.
 *
 * 5. call CLOCK_SetSimConfig() to configure the divider in sim.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v7.0
processor: MKE02Z64xxx4
package_id: MKE02Z64VQH4
mcu_data: ksdk2_0
processor_version: 8.0.1
board: FRDM-KE02Z40M
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: Bus_clock.outFreq, value: 20 MHz}
- {id: Core_clock.outFreq, value: 40 MHz}
- {id: Flash_clock.outFreq, value: 20 MHz}
- {id: ICSFF_clock.outFreq, value: 39.0625/2 kHz}
- {id: ICSIR_clock.outFreq, value: 31.25 kHz}
- {id: LPO_clock.outFreq, value: 1 kHz}
- {id: OSCER_clock.outFreq, value: 10 MHz}
- {id: Plat_clock.outFreq, value: 40 MHz}
- {id: System_clock.outFreq, value: 40 MHz}
settings:
- {id: ICSMode, value: FEE}
- {id: ICS.BDIV.scale, value: '1', locked: true}
- {id: ICS.IREFS.sel, value: ICS.RDIV}
- {id: ICS.RDIV.scale, value: '256', locked: true}
- {id: ICS_C1_IRCLKEN_CFG, value: Enabled}
- {id: OSC_CR_OSCEN_CFG, value: Enabled}
- {id: OSC_CR_OSC_MODE_CFG, value: ModeOscLowPower}
- {id: OSC_CR_RANGE_CFG, value: High}
- {id: OSC_CR_RANGE_RDIV_CFG, value: High}
- {id: SIM.BUSDIV.scale, value: '2'}
sources:
- {id: OSC.OSC.outFreq, value: 10 MHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
const ics_config_t icsConfig_BOARD_BootClockRUN =
    {
        .icsMode = kICS_ModeFEE,                  /* FEE - FLL Engaged External */
        .irClkEnableMode = kICS_IrclkEnable,      /* ICSIRCLK enabled, ICSIRCLK disabled in STOP mode */
        .bDiv = 0x0U,                             /* Bus clock divider: divided by 1 */
        .rDiv = 0x3U,                             /* FLL external reference clock divider: divided by 256 */
    };
const sim_clock_config_t simConfig_BOARD_BootClockRUN =
    {
        .busDiv = 0x1U,                           /* BUSDIV clock divider: divided by 2 */
        .busClkPrescaler = 0x0U,                  /* bus clock optional prescaler */
    };
const osc_config_t oscConfig_BOARD_BootClockRUN =
    {
        .freq = 10000000U,                        /* Oscillator frequency: 10000000Hz */
        .workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
        .enableMode = kOSC_Enable,                /* Enable external reference clock, disable external reference clock in STOP mode */
    };

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Initializes OSC0 according to board configuration. */
    CLOCK_InitOsc0(&oscConfig_BOARD_BootClockRUN);
    CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockRUN.freq);
    /* Set ICS to FEE mode. */
    CLOCK_BootToFeeMode(icsConfig_BOARD_BootClockRUN.bDiv,
                        icsConfig_BOARD_BootClockRUN.rDiv);
    /* Configure the Internal Reference clock (ICSIRCLK). */
    CLOCK_SetInternalRefClkConfig(icsConfig_BOARD_BootClockRUN.irClkEnableMode);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

