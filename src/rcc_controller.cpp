
#include "rcc_controller.hpp"
#include "stm32f4xx.h"
#include "sys_timer.hpp"

using namespace AirD;

#if !defined(HSE_VALUE)
#define HSE_VALUE ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz */
#endif                                /* HSE_VALUE */

#if !defined(HSI_VALUE)
#define HSI_VALUE ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif                                 /* HSI_VALUE */

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U /*!< Time out for HSE start up, in ms */
#endif

uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif

#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
    SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

    /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif                                                   /* USER_VECT_TAB_ADDRESS */
}

void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
    case 0x00: /* HSI used as system clock source */
        SystemCoreClock = HSI_VALUE;
        break;
    case 0x04: /* HSE used as system clock source */
        SystemCoreClock = HSE_VALUE;
        break;
    case 0x08: /* PLL used as system clock source */

        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
           SYSCLK = PLL_VCO / PLL_P
           */
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

        if (pllsource != 0)
        {
            /* HSE used as PLL clock source */
            pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }
        else
        {
            /* HSI used as PLL clock source */
            pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
        }

        pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
        SystemCoreClock = pllvco / pllp;
        break;
    default:
        SystemCoreClock = HSI_VALUE;
        break;
    }
    /* Compute HCLK frequency --------------------------------------------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    /* HCLK frequency */
    SystemCoreClock >>= tmp;
}

Status RccController::configurateOsc()
{

    /*------------------------------- HSE Configuration ------------------------*/
    /* Set the new HSE configuration ---------------------------------------*/
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    /* Check the HSE State */

    /* Get Start Tick */
    uint32_t tickstart = SysTimer::getTick();

    /* Wait till HSE is bypassed or disabled */

    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0U)
    {
        if ((SysTimer::getTick() - tickstart) > HSE_STARTUP_TIMEOUT)
        {
            return timeout;
        }
    }

    /*-------------------------------- PLL Configuration -----------------------*/
    /* Check the parameters */
    /* Check if the PLL is used as system clock or not */
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {

        /* Disable the main PLL. */
        CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

        /* Get Start Tick */
        tickstart = SysTimer::getTick();

        /* Wait till PLL is disabled */
        while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != 0U)
        {
            if ((SysTimer::getTick() - tickstart) > 2U)
            {
                return timeout;
            }
        }

        /* Configure the main PLL clock source, multiplication and division factors. */
        MODIFY_REG(RCC->PLLCFGR,
                   (RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ),
                   (8UL |                             // PLLM
                    336UL << 5 |                      // PLLN
                    0x2UL << RCC_PLLCFGR_PLLP_Pos |   // PLLP
                    1 << RCC_PLLCFGR_PLLSRC_HSE_Pos | // PLL SRC
                    7 << RCC_PLLCFGR_PLLQ_Pos));      // PLLQ

        /* Enable the main PLL. */
        SET_BIT(RCC->CR, RCC_CR_PLLON);

        /* Get Start Tick */
        tickstart = SysTimer::getTick();

        /* Wait till PLL is ready */
        while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
        {
            if ((SysTimer::getTick() - tickstart) > 2U)
            {
                return timeout;
            }
        }
    }

    return ok;
}

Status RccController::configurateClock()
{
    /* Increasing the number of wait states because of higher CPU frequency */
    if (FLASH_ACR_LATENCY_5WS > READ_BIT(FLASH->ACR, 0x7UL))
    {
        (*(__IO uint8_t *)0x40023C00U) = FLASH_ACR_LATENCY_5WS;
        if (READ_BIT(FLASH->ACR, 0x7UL) != FLASH_ACR_LATENCY_5WS)
        {
            return error;
        }
    }

    /* Set the highest APBx dividers in order to ensure that we do not go through
       a non-spec phase whatever we decrease or increase HCLK. */

    MODIFY_REG(RCC->CFGR, 0x7UL << 10U, 0x00001C00U);
    MODIFY_REG(RCC->CFGR, 0x7UL << 13U, (0x00001C00U << 3));
    MODIFY_REG(RCC->CFGR, 0xFUL << 4U, 0x00000000U);

    /*------------------------- SYSCLK Configuration ---------------------------*/

    /* PLL is selected as System Clock Source */
    if (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET)
    {
        return error;
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    MODIFY_REG(RCC->CFGR, 0x3UL, RCC_CFGR_SW_PLL);

    /* Get Start Tick */
    uint32_t tickstart = SysTimer::getTick();

    while ((RCC->CFGR & RCC_CFGR_SWS) != (RCC_CFGR_SW_PLL << RCC_CFGR_SWS_Pos))
    {
        if ((SysTimer::getTick() - tickstart) > 5000U)
        {
            return timeout;
        }
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, 0x00001400U);

    /*-------------------------- PCLK2 Configuration ---------------------------*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (0x00001000U << 3U));

    /* Update the SystemCoreClock global variable */
    SystemCoreClockUpdate();

    /* Configure the source of time base considering new system clocks settings */
    SysTimer::init(0UL);
    return ok;
}
