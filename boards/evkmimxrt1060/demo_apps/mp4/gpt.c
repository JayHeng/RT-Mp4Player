
#include "fsl_gpt.h"
#include "fsl_debug_console.h"
#include "gpt.h"

#define EXAMPLE_GPT GPT2
#define EXAMPLE_GPT_CLOCK_SOURCE_SELECT (0U)
#define EXAMPLE_GPT_CLK_FREQ (CLOCK_GetFreq(kCLOCK_IpgClk)) 
uint32_t gptFreq;

static uint32_t cnt1;



void gp_timer_measure_begin(void)
{
    cnt1 = gp_timer_get_cnt();
}

uint32_t gp_timer_measure_end(void)
{
    uint32_t cnt2;    
    uint32_t us = 0;
    cnt2 = gp_timer_get_cnt();
    us = gp_timer_compute_us(cnt2, cnt1);
    PRINTF("us: %d\r\n", us);
    return us;
}

uint32_t gp_timer_get_cnt(void)
{
    return GPT_GetCurrentTimerCount(EXAMPLE_GPT);
}
uint32_t gp_timer_compute_us(uint32_t cnt2, uint32_t cnt1)
{
    int delta;
    int working_frequency;

    if(cnt2 < cnt1)
        cnt2 += gptFreq;

    delta = cnt2 - cnt1;

    working_frequency = gptFreq/1000000;
/*    
    double t;
    t = gptFreq;
    t = 1/t;
    t = delta * t;
*/
    
    delta *= 2; // patch to adjust 43 to be 86

    return delta/working_frequency;
}

void gp_timer_delay(uint32_t us_delay)
{
    uint32_t cnt1;
    uint32_t cnt2;
    uint32_t us = 0;

    cnt1 = gp_timer_get_cnt();
    
    while(us < us_delay)
    {
        cnt2 = gp_timer_get_cnt();    
        us = gp_timer_compute_us(cnt2, cnt1);
    }
}

void gp_timer_init(void)
{
    gpt_config_t gptConfig;
    
    /*Clock setting for GPT*/
    CLOCK_SetMux(kCLOCK_PerclkMux, EXAMPLE_GPT_CLOCK_SOURCE_SELECT);

    GPT_GetDefaultConfig(&gptConfig);

    /* Initialize GPT module */
    GPT_Init(EXAMPLE_GPT, &gptConfig);

    /* Divide GPT clock source frequency by 3 inside GPT module */
    GPT_SetClockDivider(EXAMPLE_GPT, 3);

    /* Get GPT clock frequency */
    gptFreq = EXAMPLE_GPT_CLK_FREQ;

    /* GPT frequency is divided by 3 inside module */
    gptFreq /= 3;

    /* Set both GPT modules to 1 second duration */
    
    // Working frequency is 44MHz
    GPT_SetOutputCompareValue(EXAMPLE_GPT, kGPT_OutputCompare_Channel1, gptFreq);

    PRINTF("gptFreq: %d\n\r", gptFreq);

    /* Enable GPT Output Compare1 interrupt */
    //GPT_EnableInterrupts(EXAMPLE_GPT, kGPT_OutputCompare1InterruptEnable);

    /* Enable at the Interrupt */
    //EnableIRQ(GPT_IRQ_ID);

    /* Start Timer */
    PRINTF("\r\nStarting GPT timer ...");
    GPT_StartTimer(EXAMPLE_GPT);
    
    

    uint32_t cnt1;
    uint32_t cnt2;
    uint32_t us;

    PRINTF("\r\n");
    cnt1 = gp_timer_get_cnt();
    PRINTF("a");
    cnt2 = gp_timer_get_cnt();
    
    us = gp_timer_compute_us(cnt2, cnt1);
    PRINTF("\r\nthe following value should be 8.6*10 bit = 86 us\r\n");
    PRINTF("test uart - us: %d\r\n", us);
    
    gp_timer_delay(1000*1000);
    PRINTF("-\r\n");
    gp_timer_delay(1000*1000);
    PRINTF("-\r\n");
    gp_timer_delay(1000*1000);
    PRINTF("-\r\n");
}

