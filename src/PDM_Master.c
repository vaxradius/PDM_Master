#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "SDM.h"
#include "alexa_silent.h"

/*if not define __750KHZ, it will be 1.5MHz*/
#define __750KHZ
/*if not define __750KHZ_DMIC_CLOCK, it will be 1.5MHz*/
//#define __750KHZ_DMIC_CLOCK

#define PAD_DRIVESTRENGTH AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA

#define PDMCLK_PIN 12
#define PDMCLK_PIN_CONFIG AM_HAL_PIN_12_PDMCLK
#define PDMDATA_PIN 11
#define PDMDATA_PIN_CONFIG AM_HAL_PIN_11_PDMDATA

#define PCLK_PIN	18
#define PCLK_TIMER	5
#define PCLK_TIMER_SEG AM_HAL_CTIMER_TIMERB

#define PDM_DATA_P_PIN     	48
#define PDM_DATA_P_TIMER	3
#define PDM_DATA_P_CLOCK_SOURCE 0x1B

#define PDM_DATA_M_PIN     	26
#define PDM_DATA_M_TIMER	1
#define PDM_DATA_M_CLOCK_SOURCE 0x1B


#define SYNC_PIN						39
#define SYNC_TIMER						2
#define SYNC_TIMER_SEG 					AM_HAL_CTIMER_TIMERA
#define SYNC_TIMER_INT			AM_HAL_CTIMER_INT_TIMERA2
#define SYNC_TIMER_CLOCK_SOURCE 0x1B

#define BUF_SIZE			128

#ifdef __750KHZ
	#define OSR				(48) //over sampling rate 750KHz
#else
	#define OSR				(96) //over sampling rate 1.5MHz
	//#define OSR				(144) //over sampling rate 3MHz
#endif
//#define OSR					(192) //over sampling rate 3MHz
//#define OSR				(128) //over sampling rate 2MHz
//#define OSR				(96) //over sampling rate 1.5MHz
//#define OSR				(64) //over sampling rate 1MHz
//#define OSR				(48) //over sampling rate 750KHz
//#define OSR				(32) //over sampling rate 500KHz
//#define OSR				(24) //over sampling rate 350KHz
//#define OSR				(16) //over sampling rate 250KHz
#define BIT_RESOLUTION 	16
#define BIT_BUF_SIZE 	((BUF_SIZE*OSR)/BIT_RESOLUTION)
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
volatile bool g_bPDMDataReady = false;

int16_t i16PDMBuf[2][BUF_SIZE] = {{0},{0}};
uint16_t i16BitsBuf[2][BIT_BUF_SIZE] = {0}; //16bit*3 = 48 over sampling rate
uint32_t u32PDMPingpong = 0;
uint32_t u32BitBufPingpong = 0;

//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
void *PDMHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
	.eClkDivider = AM_HAL_PDM_MCLKDIV_1,
	.eLeftGain = AM_HAL_PDM_GAIN_P405DB,
	.eRightGain = AM_HAL_PDM_GAIN_P405DB,
#ifdef __750KHZ_DMIC_CLOCK
	.ui32DecimationRate = (24),
#else
	.ui32DecimationRate = (48),
#endif
	.bHighPassEnable = 0,
	.ui32HighPassCutoff = 0xB,
#ifdef __750KHZ_DMIC_CLOCK
	.ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,
#else
	.ePDMClkSpeed = AM_HAL_PDM_CLK_1_5MHZ,
#endif
	.bInvertI2SBCLK = 0,
	.ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
	.bPDMSampleDelay = 0,
	.bDataPacking = 1,
	.ePCMChannels = AM_HAL_PDM_CHANNEL_LEFT,
	.bLRSwap = 0,
};


static void timer_handler(void);

void pwm_out(void)
{

#if 0
	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(PCLK_TIMER,
	                            PCLK_TIMER_SEG,
	                            PCLK_PIN,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);
#else
	am_hal_gpio_pinconfig(PCLK_PIN, g_AM_HAL_GPIO_DISABLE);
#endif
	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(PCLK_TIMER,               // ui32TimerNumber
	                            PCLK_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE) );

	//
	// Set up initial timer periods.
	//
#ifdef __750KHZ
	am_hal_ctimer_period_set(PCLK_TIMER,
	                         PCLK_TIMER_SEG, 15, 8); //PCLK 750KHz
	am_hal_ctimer_aux_period_set(PCLK_TIMER,
	                       PCLK_TIMER_SEG, 15, 8);
#else
	am_hal_ctimer_period_set(PCLK_TIMER,
	                         PCLK_TIMER_SEG, 7, 4); //PCLK 1.5MHz
	am_hal_ctimer_aux_period_set(PCLK_TIMER,
	                       PCLK_TIMER_SEG, 7, 4);
#endif
	//////////////////////////////////////////////////////////////////////////////////
	//
	// Start the timer.
	//
	am_hal_ctimer_start(PCLK_TIMER, PCLK_TIMER_SEG);

}

// Timer Interrupt Service Routine (ISR)
void am_ctimer_isr(void)
{
    uint32_t ui32Status;
	//am_hal_gpio_output_toggle(6);

    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);

    am_hal_ctimer_int_service(ui32Status);
}


void
initialize_pattern128_counter(uint32_t ui32TimerNumber,
                           uint64_t ui64Pattern0,
                           uint64_t ui64Pattern1,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, AM_HAL_CTIMER_BOTH);

    am_hal_ctimer_config_single(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    | AM_HAL_CTIMER_INT_ENABLE | 
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)(ui64Pattern0 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)((ui64Pattern0 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 48) & 0xFFFF));

    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)(ui64Pattern1 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)((ui64Pattern1 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              PAD_DRIVESTRENGTH);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, AM_HAL_CTIMER_BOTH);
}


void
initialize_pattern128_counter_invert(uint32_t ui32TimerNumber,
                           uint64_t ui64Pattern0,
                           uint64_t ui64Pattern1,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, AM_HAL_CTIMER_BOTH);

    am_hal_ctimer_config_single(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    | AM_HAL_CTIMER_PIN_INVERT| 
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)(ui64Pattern0 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)((ui64Pattern0 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 48) & 0xFFFF));

    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)(ui64Pattern1 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)((ui64Pattern1 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              PAD_DRIVESTRENGTH);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, AM_HAL_CTIMER_BOTH);
}


void
initialize_pattern64_counter(uint32_t ui32TimerNumber,
                           uint32_t ui32TimerSegment,
                           uint64_t ui64Pattern,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, ui32TimerSegment);

    am_hal_ctimer_config_single(ui32TimerNumber, ui32TimerSegment,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |AM_HAL_CTIMER_INT_ENABLE |
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)(ui64Pattern & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)((ui64Pattern >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, ui32TimerSegment,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, ui32TimerSegment, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, ui32TimerSegment);
}

void
global_disable(void)
{
    CTIMER->GLOBEN = 0x0;
}

void
global_enable(void)
{
    CTIMER->GLOBEN = 0xffff;
}

void Pattern_init(void)
{
	//
	// Disable all the counters.
	//
	global_disable();

	pwm_out();

    	initialize_pattern64_counter(SYNC_TIMER, SYNC_TIMER_SEG, 0xFFFFFFFF00000000, 
				63, CTIMER_AUX0_TMRB0TRIG_DIS, SYNC_PIN,
				_VAL2FLD(CTIMER_CTRL0_TMRA0CLK, SYNC_TIMER_CLOCK_SOURCE));

	initialize_pattern128_counter(PDM_DATA_P_TIMER, 0x0000000000000000, 0x0000000000000000, 
				127, CTIMER_AUX0_TMRB0TRIG_DIS, PDM_DATA_P_PIN,
				_VAL2FLD(CTIMER_CTRL0_TMRA0CLK, PDM_DATA_P_CLOCK_SOURCE));

	initialize_pattern128_counter_invert(PDM_DATA_M_TIMER, 0x0000000000000000, 0x0000000000000000, 
				127, CTIMER_AUX0_TMRB0TRIG_DIS, PDM_DATA_M_PIN,
				_VAL2FLD(CTIMER_CTRL0_TMRA0CLK, PDM_DATA_M_CLOCK_SOURCE));

	//
	// Enable all the counters.
	//
	global_enable();


	//
	// Clear the timer Interrupt
	//
	am_hal_ctimer_int_clear(SYNC_TIMER_INT);


	//
	// Enable the timer Interrupt.
	//
	am_hal_ctimer_int_register(SYNC_TIMER_INT,
	               timer_handler);

	am_hal_ctimer_int_enable(SYNC_TIMER_INT);

	//
	// Enable the timer interrupt in the NVIC.
	//
	NVIC_EnableIRQ(CTIMER_IRQn);
}

//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void
pdm_init(void)
{
	//
	// Initialize, power-up, and configure the PDM.
	//
	am_hal_pdm_initialize(0, &PDMHandle);
	am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);
	am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);
	am_hal_pdm_enable(PDMHandle);

	//
	// Configure the necessary pins.
	//
	am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	sPinCfg.uFuncSel = AM_HAL_PIN_12_PDMCLK;
	am_hal_gpio_pinconfig(PDMCLK_PIN, sPinCfg);

	sPinCfg.uFuncSel = AM_HAL_PIN_11_PDMDATA;
	am_hal_gpio_pinconfig(PDMDATA_PIN, sPinCfg);

	
	//
	// Configure and enable PDM interrupts (set up to trigger on DMA
	// completion).
	//
	am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
	                                        | AM_HAL_PDM_INT_DCMP
	                                        | AM_HAL_PDM_INT_UNDFL
	                                        | AM_HAL_PDM_INT_OVF));

	NVIC_EnableIRQ(PDM_IRQn);

}


//*****************************************************************************
//
// Start a transaction to get some number of bytes from the PDM interface.
//
//*****************************************************************************
void
pdm_data_get(int16_t *dest)
{
	//
	// Configure DMA and target address.
	//
	am_hal_pdm_transfer_t sTransfer;
	sTransfer.ui32TargetAddr = (uint32_t ) dest;
	sTransfer.ui32TotalCount = BUF_SIZE*2;


	//
	// Start the data transfer.
	//
	am_hal_pdm_dma_start(PDMHandle, &sTransfer);

}

//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void
am_pdm0_isr(void)
{
	uint32_t ui32Status;
	//am_hal_gpio_state_write(8 , AM_HAL_GPIO_OUTPUT_CLEAR);

	//
	// Read the interrupt status.
	//
	am_hal_pdm_interrupt_status_get(PDMHandle, &ui32Status, false);
	am_hal_pdm_interrupt_clear(PDMHandle, ui32Status);

	if (ui32Status & AM_HAL_PDM_INT_DCMP)
	{
		g_bPDMDataReady = true;
		pdm_data_get(i16PDMBuf[(++u32PDMPingpong)%2]);
	}
 	//am_hal_gpio_output_toggle(8); 
}

#define TIMER_OFFSET            (&CTIMER->TMR1 - &CTIMER->TMR0)
static void timer_handler(void)
{
	volatile uint32_t *pui32CmprRegA, *pui32CmprRegB;
	volatile uint32_t *pui32CmprAuxRegA, *pui32CmprAuxRegB;
	static uint32_t g_bitflag = 0;
	uint32_t ui32Pattern01 = 0;
	uint32_t ui32Pattern23 = 0;
	static uint32_t index = 0;
	static uint16_t *pcm_idx = i16BitsBuf[0];

	g_bitflag +=1;
	ui32Pattern01 = *((uint32_t *)(pcm_idx+index));
	ui32Pattern23 = *((uint32_t *)(pcm_idx+index+2));

	if(g_bitflag%2)
	{
		pui32CmprRegA = (uint32_t *)(&CTIMERn(0)->CMPRA0 + (PDM_DATA_P_TIMER * TIMER_OFFSET));
		*pui32CmprRegA = ui32Pattern01;
		pui32CmprAuxRegA = (uint32_t *)(&CTIMERn(0)->CMPRAUXA0 + (PDM_DATA_P_TIMER * TIMER_OFFSET));
		*pui32CmprAuxRegA = ui32Pattern23;

		pui32CmprRegA = (uint32_t *)(&CTIMERn(0)->CMPRA0 + (PDM_DATA_M_TIMER * TIMER_OFFSET));
		*pui32CmprRegA = ui32Pattern01;
		pui32CmprAuxRegA = (uint32_t *)(&CTIMERn(0)->CMPRAUXA0 + (PDM_DATA_M_TIMER * TIMER_OFFSET));
		*pui32CmprAuxRegA = ui32Pattern23;

	}
	else
	{
		pui32CmprRegB = (uint32_t *)(&CTIMERn(0)->CMPRB0 + (PDM_DATA_P_TIMER * TIMER_OFFSET));
		*pui32CmprRegB = ui32Pattern01;
		pui32CmprAuxRegB = (uint32_t *)(&CTIMERn(0)->CMPRAUXB0 + (PDM_DATA_P_TIMER * TIMER_OFFSET));
		*pui32CmprAuxRegB = ui32Pattern23;

		pui32CmprRegB = (uint32_t *)(&CTIMERn(0)->CMPRB0 + (PDM_DATA_M_TIMER * TIMER_OFFSET));
		*pui32CmprRegB = ui32Pattern01;
		pui32CmprAuxRegB = (uint32_t *)(&CTIMERn(0)->CMPRAUXB0 + (PDM_DATA_M_TIMER * TIMER_OFFSET));
		*pui32CmprAuxRegB = ui32Pattern23;
	}

	index += 4;
	if(index == BIT_BUF_SIZE)
	{
		index = 0;
		pcm_idx = i16BitsBuf[(++u32BitBufPingpong)%2];
		//am_hal_gpio_output_toggle(6);
	}
		
	
	return;
}


static int32_t first_integrator = 0;
static int32_t first_result = 0;
static int32_t second_integrator = 0;
//static int32_t second_result = 0;

//Derrived from  http://www.danbullard.com/dan/A_Sigma_Delta_ADC_Simulator_in_C.pdf
void two_level_sigma_delta(uint16_t *out, int16_t in, uint16_t len)
{
	uint16_t i,j = 0;
	int32_t y = in;
	y >>= 1;

	for(i = 0; i < len ; i++)
	{
		*(out+i) = 0;
		for(j=0;j<16;j++)
		{
			second_integrator += (y -(first_result));
			first_integrator += ((second_integrator) -(first_result));

			if (first_integrator > 0) 
			{			
				first_result = 32768;
				*(out+i) |= (1 << j);
			}
			else
			{
				first_result = -32768;
			}
		}
	}
}

//Derrived from https://books.google.com.tw/books?id=Gum3CgAAQBAJ&pg=PA120&dq=Delta-sigma+Modulators+0.1158&hl=zh-TW&sa=X&ved=2ahUKEwiJ6Key4q3sAhVHyosBHQdADwQQ6AEwAHoECAAQAg#v=onepage&q&f=false
void three_level_sigma_delta(uint16_t *out, int16_t in, uint16_t len)
{
		uint16_t i,j = 0;

        static float qe0 = 0;
        static float qe1 = 0;
        static float qe2 = 0;


        static float sum_value0 = 0;
        static float sum_value1 = 0;
        static float sum_value2 = 0;
        static float last_dsm_signal = 0;

        static float k1 = 0.1158;
        static float k2 = 0.2776;
        static float k3 = 1.0267;
        static float g = 0.00153846;
		float u = ((float) in)/(65536.0);

        for(i = 0; i < len ; i++)
		{
			*(out+i) = 0;
			for(j=0;j<16;j++)
			{
                qe0 = k1 * u - k1*last_dsm_signal; 
                sum_value0 = sum_value0 + qe0;


                qe1 = sum_value0 - k2*last_dsm_signal - g*sum_value2;

                sum_value1 = sum_value1 + qe1;

                qe2 = sum_value1 - k3*last_dsm_signal;

                sum_value2 = sum_value2 + qe2;


                if(sum_value2 > 0)
				{
					last_dsm_signal = 1;
					*(out+i) |= (1 << j);                         
                } 
				else 
				{
                    last_dsm_signal = -1;
                }

        	}
        }
}


void PDM_2_PWM(uint16_t *pdm)
{
	uint32_t i,j = 0;
	uint16_t pdm_tmp;
	uint16_t idx = 0;

	for(i=0; i<(OSR/16); i++)
	{
		pdm_tmp = *(pdm+i);
		//am_util_stdio_printf("%x ",*(pdm+i));
		*(pdm+i) = 0;
		idx = 0;
		for(j=0; j<16; j++)
		{
			if(pdm_tmp >> j & 0x01)
			{
				*(pdm+i)|= (1<<idx);
				idx++;
				//am_util_stdio_printf("1 ");
			}
			//else
				//am_util_stdio_printf("0 ");
		}
		//am_util_stdio_printf("%x \n",*(pdm+i));
	}
}
void Sigma_Delta_ADC(uint16_t *pdm, int16_t *pcm)
{
	uint32_t j = 0;

	for(j=0; j<BUF_SIZE; j++)
	{
		//Am_Sigma_Delta(pdm+(j*(OSR/16)), (*(pcm+j))*2, (OSR/16));
		//PDM_2_PWM(pdm+(j*(OSR/16)));
		//two_level_sigma_delta(pdm+(j*(OSR/16)), (*(pcm+j)), (OSR/16));
		three_level_sigma_delta(pdm+(j*(OSR/16)), (*(pcm+j)), (OSR/16));
	}
}

void Enable_HFADJ(void)
{

//
    // Enable the 32KHz XTAL.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

    //
    // Wait for 1 second for the 32KHz XTAL to startup and stabilize.
    //
    am_util_delay_ms(1000);

    //
    // Enable HFADJ.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE, 0);
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
	uint32_t u32PDMpg;
	uint32_t u32BitBufpg;
	am_hal_burst_avail_e          eBurstModeAvailable;
	am_hal_burst_mode_e	eBurstMode;
	uint32_t pcm_idx = 0x30;
		
	//
	// Perform the standard initialzation for clocks, cache settings, and
	// board-level low-power operation.
	//
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	//Enable_HFADJ();
	//am_bsp_itm_printf_enable();

	am_hal_interrupt_master_disable();



	Pattern_init();


	//
	// Turn on the PDM, set it up for our chosen recording settings, and start
	// the first DMA transaction.
	//
	pdm_init();
	am_hal_pdm_fifo_flush(PDMHandle);
	am_hal_pdm_enable(PDMHandle);
	am_hal_interrupt_master_enable();
	pdm_data_get(i16PDMBuf[0]);
	

#if 1
	am_hal_burst_mode_initialize(&eBurstModeAvailable);

	am_hal_burst_mode_enable(&eBurstMode);
#endif
	//
	// Enable floating point.
	//
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	//
	// Turn ON Flash1
	//
	am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_1M);


	//
	// Power on SRAM
	//
	PWRCTRL->MEMPWDINSLEEP_b.SRAMPWDSLP = PWRCTRL_MEMPWDINSLEEP_SRAMPWDSLP_NONE;

	Am_Sigma_Delta_Init(4, OSR);

	//
	//
	// Loop forever while sleeping.
	//
	while (1)
	{
		if(g_bPDMDataReady == true)
		{
			g_bPDMDataReady = false;
			u32PDMpg = u32PDMPingpong;
			u32BitBufpg = u32BitBufPingpong;

			//am_util_delay_ms(20);
			//am_util_delay_ms(26);	//512 buffer , 	96MHz	
			//am_util_delay_ms(5);	//128 buffer , 	96MHz  GCC -Os am_sdm 4.240ms 
			//am_util_delay_us(64+32+16+8);//128 buffer , 	96MHz  , 750KHz , 5th Sigma Delta
			//am_util_delay_us(256+32+16+8+2);//128 buffer , 	96MHz  750KHz , 4th Sigma Delta
			//am_util_delay_us(6*1024+32+4+2+1);//128 buffer , 	96MHz  750KHz , 3rd Sigma Delta (0.75476074218) cpu available
#if 0
			Sigma_Delta_ADC(i16BitsBuf[(u32BitBufpg+1)%2], i16PDMBuf[(u32PDMpg-1)%2]);
#else
			Sigma_Delta_ADC(i16BitsBuf[(u32BitBufpg+1)%2], ((int16_t*)US_P3_4_F_Silent0_wav)+pcm_idx);
			pcm_idx += BUF_SIZE;
			if(pcm_idx > (US_P3_4_F_Silent0_wav_size/2) - BUF_SIZE)
				pcm_idx = 0x30;
#endif

			if(u32BitBufpg != u32BitBufPingpong)
			{
				while(1);
			}

			if(u32PDMpg != u32PDMPingpong)
			{
				while(1);
			}

		}

		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);

	}
}

