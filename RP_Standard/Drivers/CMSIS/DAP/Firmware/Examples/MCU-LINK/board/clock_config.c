/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/
/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up wait states of the flash.
 *
 * 3. Set up all dividers.
 *
 * 4. Set up all selectors to provide selected clocks.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v7.0
processor: LPC55S69
package_id: LPC55S69JBD64
mcu_data: ksdk2_0
processor_version: 9.0.3
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "fsl_power.h"
#include "fsl_clock.h"
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
- {id: FXCOM0_clock.outFreq, value: 48 MHz}
- {id: FXCOM3_clock.outFreq, value: 48 MHz}
- {id: System_clock.outFreq, value: 150 MHz, locked: true, accuracy: '0.001'}
- {id: USB1_PHY_clock.outFreq, value: 16 MHz}
settings:
- {id: PLL0_Mode, value: Normal}
- {id: ANALOG_CONTROL_FRO192M_CTRL_ENDI_FRO_96M_CFG, value: Enable}
- {id: ENABLE_CLKIN_ENA, value: Enabled}
- {id: ENABLE_PLL_USB_OUT, value: Enabled}
- {id: ENABLE_SYSTEM_CLK_OUT, value: Enabled}
- {id: SYSCON.FCCLKSEL0.sel, value: SYSCON.FROHFDIV}
- {id: SYSCON.FCCLKSEL3.sel, value: SYSCON.FROHFDIV}
- {id: SYSCON.FRGCTRL3_DIV.scale, value: '256', locked: true}
- {id: SYSCON.FROHFDIV.scale, value: '2', locked: true}
- {id: SYSCON.MAINCLKSELB.sel, value: SYSCON.PLL0_BYPASS}
- {id: SYSCON.PLL0CLKSEL.sel, value: SYSCON.CLK_IN_EN}
- {id: SYSCON.PLL0M_MULT.scale, value: '150', locked: true}
- {id: SYSCON.PLL0N_DIV.scale, value: '8', locked: true}
- {id: SYSCON.PLL0_PDEC.scale, value: '2'}
sources:
- {id: ANACTRL.fro_hf.outFreq, value: 96 MHz}
- {id: SYSCON.XTAL32M.outFreq, value: 16 MHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
#ifndef SDK_SECONDARY_CORE
    /*!< Set up the clock sources */
    /*!< Configure FRO192M */
    POWER_DisablePD(kPDRUNCFG_PD_FRO192M);               /*!< Ensure FRO is on  */
    CLOCK_SetupFROClocking(12000000U);                   /*!< Set up FRO to the 12 MHz, just for sure */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                /*!< Switch to FRO 12MHz first to ensure we can change the clock setting */

    CLOCK_SetupFROClocking(96000000U);                   /* Enable FRO HF(96MHz) output */

    /*!< Configure XTAL32M */
    POWER_DisablePD(kPDRUNCFG_PD_XTAL32M);                        /* Ensure XTAL32M is powered */
    POWER_DisablePD(kPDRUNCFG_PD_LDOXO32M);                       /* Ensure XTAL32M is powered */
    CLOCK_SetupExtClocking(16000000U);                            /* Enable clk_in clock */
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK;       /* Enable clk_in from XTAL32M clock  */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_SYSTEM_CLK_OUT_MASK;    /* Enable clk_in to system  */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_PLL_USB_OUT_MASK;       /* Enable clk_in to HS USB  */

    POWER_SetVoltageForFreq(150000000U);                  /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    CLOCK_SetFLASHAccessCyclesForFreq(150000000U);          /*!< Set FLASH wait states for core */

    /*!< Set up PLL */
    CLOCK_AttachClk(kEXT_CLK_to_PLL0);                    /*!< Switch PLL0CLKSEL to EXT_CLK */
    POWER_DisablePD(kPDRUNCFG_PD_PLL0);                  /* Ensure PLL is on  */
    POWER_DisablePD(kPDRUNCFG_PD_PLL0_SSCG);
    const pll_setup_t pll0Setup = {
        .pllctrl = SYSCON_PLL0CTRL_CLKEN_MASK | SYSCON_PLL0CTRL_SELI(53U) | SYSCON_PLL0CTRL_SELP(31U),
        .pllndec = SYSCON_PLL0NDEC_NDIV(8U),
        .pllpdec = SYSCON_PLL0PDEC_PDIV(1U),
        .pllsscg = {0x0U,(SYSCON_PLL0SSCG1_MDIV_EXT(150U) | SYSCON_PLL0SSCG1_SEL_EXT_MASK)},
        .pllRate = 150000000U,
        .flags =  PLL_SETUPFLAG_WAITLOCK
    };
    CLOCK_SetPLL0Freq(&pll0Setup);                       /*!< Configure PLL0 to the desired values */

    /*!< Set up dividers */
    #if FSL_CLOCK_DRIVER_VERSION >= MAKE_VERSION(2, 3, 4)
      CLOCK_SetClkDiv(kCLOCK_DivFlexFrg0, 0U, false);         /*!< Set DIV to value 0xFF and MULT to value 0U in related FLEXFRGCTRL register */
    #else
      CLOCK_SetClkDiv(kCLOCK_DivFlexFrg0, 256U, false);         /*!< Set DIV to value 0xFF and MULT to value 0U in related FLEXFRGCTRL register */
    #endif
    #if FSL_CLOCK_DRIVER_VERSION >= MAKE_VERSION(2, 3, 4)
      CLOCK_SetClkDiv(kCLOCK_DivFlexFrg3, 0U, false);         /*!< Set DIV to value 0xFF and MULT to value 0U in related FLEXFRGCTRL register */
    #else
      CLOCK_SetClkDiv(kCLOCK_DivFlexFrg3, 256U, false);         /*!< Set DIV to value 0xFF and MULT to value 0U in related FLEXFRGCTRL register */
    #endif
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);         /*!< Set AHBCLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivFrohfClk, 0U, true);               /*!< Reset FROHFDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivFrohfClk, 2U, false);         /*!< Set FROHFDIV divider to value 2 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kPLL0_to_MAIN_CLK);                 /*!< Switch MAIN_CLK to PLL0 */
    CLOCK_AttachClk(kFRO_HF_DIV_to_FLEXCOMM0);                 /*!< Switch FLEXCOMM0 to FRO_HF_DIV */
    CLOCK_AttachClk(kFRO_HF_DIV_to_FLEXCOMM3);                 /*!< Switch FLEXCOMM3 to FRO_HF_DIV */

    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
#endif
}

