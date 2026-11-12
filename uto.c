#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>
#include <stdio.h>
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"

volatile uint8_t UART_LIBRE = 1;
volatile uint8_t DATO_R, NUEVO_DATO;
#define MASK_BUZZER (0x01UL)
#define MASK_LED (1<<22)
#define MASK_LED1 (1<<26)
#define T_BUFFER 50u
#define T_BLOQUE 50u
#define ref_apnea 12u
#define F_MUESTREO 100u
#define ESC_DAC 8u
volatile uint32_t BUFFER_DAC[2][2*T_BLOQUE];
volatile uint32_t BUFFER_DMA[2][T_BLOQUE];
static GPDMA_LLI_T lli_adc[2];
static GPDMA_LLI_T lli_dac[2];
volatile uint8_t BLOQUE_LISTO= 1u;
volatile uint8_t BAND_DMA =0u;
volatile uint8_t seg_sin_respi =0u;
volatile uint8_t BAND_APNEA =0u;
volatile uint8_t ESTADO =0u; //ESTADO=0 => ESPIRANDO ; ESTADO=1 => INSPIRANDO
volatile int32_t RECONST =0;
volatile int32_t BASE =0;
volatile uint32_t b =0u;
volatile int32_t x =0;
volatile int32_t SIGNAL_CENT =0;
volatile int32_t MAX_CICLO =0;
volatile int32_t MIN_CICLO =0;
volatile uint32_t N_DESDE_CRUCE =0u;
volatile uint32_t F_RESPI =0u;
volatile int32_t v =0;
volatile uint32_t ESFUERZO =0u;
volatile int32_t HISTER =10000;
#define TX_MAX 700u
char TX_BUF[TX_MAX];
volatile uint32_t N_TX =0u;
volatile uint8_t PUEDO_ENVIAR =0u;
volatile uint8_t EN_ALARMA =0u;

void CONFIGPINES(void){
	//Configuro LED P0.22 (rojo) para alarma de apnea
	PINSEL_CFG_T pinCfg={0};
    pinCfg.port = 0;
	pinCfg.func = 0;
    pinCfg.mode = PINSEL_PULLUP;
    pinCfg.openDrain = DISABLE;
    pinCfg.pin = 22;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(PORT_0,MASK_LED ,GPIO_OUTPUT);

    pinCfg.port = 3;
	pinCfg.func = 0;
    pinCfg.mode = PINSEL_PULLUP;
    pinCfg.openDrain = DISABLE;
    pinCfg.pin = 26;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(PORT_3,MASK_LED1 ,GPIO_OUTPUT);

    //Configuro buzzer para alarma de apnea
    pinCfg.port = 0;
	pinCfg.func = 0;
    pinCfg.mode = PINSEL_TRISTATE;
    pinCfg.openDrain = DISABLE;
    pinCfg.pin = 0;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(PORT_0, MASK_BUZZER ,GPIO_OUTPUT);

	//Configuro pines para TX y RX
	pinCfg.port = 0;
	pinCfg.func = 1;
	pinCfg.mode = PINSEL_PULLUP;
	pinCfg.openDrain = DISABLE;
	pinCfg.pin = 2;
	PINSEL_ConfigPin(&pinCfg);   // TXD0
	pinCfg.pin = 3;
	PINSEL_ConfigPin(&pinCfg);   // RXD0

	//CAMBIO: pin de salida del DAC, P0.26 = AOUT
	pinCfg.port = 0;
	pinCfg.func = 2;
	pinCfg.mode = PINSEL_TRISTATE;
	pinCfg.openDrain = DISABLE;
	pinCfg.pin = 26;
	PINSEL_ConfigPin(&pinCfg);
}
void CONFIGADC(void){
	ADC_Init(100000);
	ADC_PinConfig(ADC_CHANNEL_0);
	ADC_StartCmd(ADC_START_ON_MAT01);
	ADC_ChannelEnable(ADC_CHANNEL_0);
	ADC_EdgeStartConfig(ADC_START_ON_FALLING);
	// Arranco el Timer al final del main
	// Habilito la interrupción del canal 0 del ADC, para generar la petición al DMA
	ADC_IntEnable(ADC_INT_CH0);
}
void CONFIGDAC(void){
	DAC_CONVERTER_CFG_T dacCfg={0};
	dacCfg.doubleBuffer = 0;
	dacCfg.dmaRequest = ENABLE;
	dacCfg.dmaCounter = ENABLE;
	//Activo el contador
	DAC_Init();
	//PCLK del DAC = CCLK/8
	LPC_SC->PCLKSEL0 |= (3u << 22);
	DAC_SetBias(DAC_350uA);
	DAC_ConfigDAConverterControl(&dacCfg);
	DAC_SetDMATimeOut(((SystemCoreClock / 8u) / (F_MUESTREO * 2u) - 1u));
}
void CONFIGUART0(void){
    //Habilito PCUART0 en PCONP (bit 3)
    LPC_SC->PCONP |= (1u << 3); // Enciendo UART0
    LPC_SC->PCLKSEL0 &= ~(3u << 6); // PCLK_UART0 = CCLK / 4 (Default 25 MHz)

    //Configuro P0.2 en TXD0 (bits 5:4 = 01) y P0.3 en RXD0 (bits 7:6 = 01)
    LPC_PINCON->PINSEL0 &= ~((3u << 4) | (3u << 6));
    LPC_PINCON->PINSEL0 |=  ((1u << 4) | (1u << 6)); // Función 01 (TXD0 y RXD0)

    //FIFO y Modo DMA en FCR: FIFO Enable (bit 0), Reset RX/TX (bits 1-2), DMA Mode (bit 3)
    LPC_UART0->FCR = 0x0F;
    LPC_UART0->LCR |= (1u << 7);
    LPC_UART0->DLL  = 12u;
    LPC_UART0->DLM  = 0u;
    LPC_UART0->FDR  = (8u << 4) | 1u;     // MULVAL = 8, DIVADDVAL = 1 → 115.741 baudios (0,5 %)
    //LPC_UART0->LCR &= ~(1u << 7);
    LPC_UART0->LCR = 0x03;     // 8 bits, 1 stop, sin paridad, y de paso DLAB = 0
}
void CONFIGGPDMA_ADC(void){
	GPDMA_Init();
	NVIC_EnableIRQ(DMA_IRQn); // Habilito la ISR del DMA
	uint32_t ctl = T_BUFFER | (2u << 18) | (2u << 21) | (1u << 27) | (1u << 31);
	//Configuro GPDMA ping-pong para ADC
	lli_adc[0].srcAddr = (uint32_t)&LPC_ADC->ADDR0;
	lli_adc[0].dstAddr = (uint32_t)BUFFER_DMA[1];
	lli_adc[0].nextLLI = (uint32_t)&lli_adc[1];
	lli_adc[0].control = ctl;

	lli_adc[1].srcAddr = (uint32_t)&LPC_ADC->ADDR0;
	lli_adc[1].dstAddr = (uint32_t)BUFFER_DMA[0];
	lli_adc[1].nextLLI = (uint32_t)&lli_adc[0];
	lli_adc[1].control = ctl;

	GPDMA_Channel_CFG_T gpdmaCfg={0};
	gpdmaCfg.channelNum=GPDMA_CH_0;
	gpdmaCfg.transferSize=T_BLOQUE;
	gpdmaCfg.type=GPDMA_P2M;
	gpdmaCfg.srcMemAddr=0;
	gpdmaCfg.dstMemAddr=(uint32_t)BUFFER_DMA[0];
	gpdmaCfg.srcConn=GPDMA_ADC;
	gpdmaCfg.dstConn=0;
	gpdmaCfg.src.width=GPDMA_WORD;
	gpdmaCfg.src.burst=GPDMA_BSIZE_1;
	gpdmaCfg.src.increment=DISABLE;
	gpdmaCfg.dst.width=GPDMA_WORD;
	gpdmaCfg.dst.burst=GPDMA_BSIZE_1;
	gpdmaCfg.dst.increment=ENABLE;
	gpdmaCfg.intTC=ENABLE;
	gpdmaCfg.intErr=DISABLE;
	gpdmaCfg.linkedList=(uint32_t)&lli_adc[0];
	GPDMA_SetupChannel(&gpdmaCfg);
	GPDMA_ChannelStart(0);
}

void CONFGPDMA_DAC(void){
	uint32_t ctrl = (2u*T_BUFFER) | (2u << 18) | (2u << 21) | (1u << 26);

	lli_dac[0].srcAddr = (uint32_t)BUFFER_DAC[1];
	lli_dac[0].dstAddr = (uint32_t)&LPC_DAC->DACR;
	lli_dac[0].nextLLI = (uint32_t)&lli_dac[1];
	lli_dac[0].control = ctrl;

	lli_dac[1].srcAddr = (uint32_t)BUFFER_DAC[0];
	lli_dac[1].dstAddr = (uint32_t)&LPC_DAC->DACR;
	lli_dac[1].nextLLI = (uint32_t)&lli_dac[0];
	lli_dac[1].control = ctrl;

	GPDMA_Channel_CFG_T dmaCfg = {0};
	dmaCfg.channelNum    = GPDMA_CH_1;
	dmaCfg.transferSize  = 2u*T_BUFFER;
	dmaCfg.type          = GPDMA_M2P;
	dmaCfg.srcConn       = 0;
	dmaCfg.dstConn       = GPDMA_DAC;
	dmaCfg.srcMemAddr    = (uint32_t)BUFFER_DAC[0];
	dmaCfg.dstMemAddr    = 0;
	dmaCfg.src.width     = GPDMA_WORD;
	dmaCfg.src.burst     = GPDMA_BSIZE_1;
	dmaCfg.src.increment = ENABLE;
	dmaCfg.dst.width     = GPDMA_WORD;
	dmaCfg.dst.burst     = GPDMA_BSIZE_1;
	dmaCfg.dst.increment = DISABLE;
	dmaCfg.intTC         = DISABLE;
	dmaCfg.intErr        = DISABLE;
	dmaCfg.linkedList    = (uint32_t)&lli_dac[0];
	GPDMA_SetupChannel(&dmaCfg);
	GPDMA_ChannelStart(GPDMA_CH_1);

}
void ENVIAR_UART(uint32_t n){
	GPDMA_Channel_CFG_T uartCfg={0};
	UART_LIBRE=0u;
	uartCfg.channelNum=GPDMA_CH_2;                   // 0 = ADC, 1 = DAC
	uartCfg.transferSize=n;
	uartCfg.type=GPDMA_M2P;
	uartCfg.srcMemAddr=(uint32_t)TX_BUF;
	uartCfg.dstMemAddr=0;
	uartCfg.srcConn=0;
	uartCfg.dstConn=GPDMA_UART0_Tx;
	uartCfg.src.width=GPDMA_BYTE; // Byte, no word
	uartCfg.src.burst=GPDMA_BSIZE_1;
	uartCfg.src.increment=ENABLE; // Recorre el buffer
	uartCfg.dst.width=GPDMA_BYTE;
	uartCfg.dst.burst=GPDMA_BSIZE_1;
	uartCfg.dst.increment=DISABLE; // Siempre escribe en THR
	uartCfg.intTC=ENABLE; //Avisa cuando terminó
	uartCfg.intErr=ENABLE;
	uartCfg.linkedList=0; // Un bloque y se apaga
	if(GPDMA_SetupChannel(&uartCfg)==ERROR){ // Marca canal ocupado
		UART_LIBRE=1u;
		return;
	}
	GPDMA_ChannelStart(GPDMA_CH_2);
}
void CONFIGTIMER0(void){
	//Timer0 dispara el ADC a 100 Hz por MAT0.1 (el driver solo ofrece MAT0.1 y MAT0.3)
	TIM_TIMERCFG_T timCfg={0};
	timCfg.prescaleOpt= TIM_TICK;
	timCfg.prescaleValue= 0;
	TIM_InitTimer(LPC_TIM0, &timCfg);
	TIM_MATCHCFG_T tim0Cfg={0};
	tim0Cfg.channel = TIM_MATCH_1;
	tim0Cfg.intEn = DISABLE;
	tim0Cfg.resetEn = ENABLE;
	tim0Cfg.stopEn = DISABLE;
	tim0Cfg.extOpt = TIM_TOGGLE;
	tim0Cfg.matchValue = (SystemCoreClock / (2u * F_MUESTREO) / 4u) - 1u;
	TIM_ConfigMatch(LPC_TIM0, &tim0Cfg);
}
void CONFIGTIMER1(void){
	//Timer1 cada 1 s, cuenta segundos sin respirar
	TIM_TIMERCFG_T timCfg={0};
	timCfg.prescaleOpt= TIM_TICK;
	timCfg.prescaleValue= 0;
	TIM_InitTimer(LPC_TIM1, &timCfg);
	TIM_MATCHCFG_T tim1Cfg={0};
	tim1Cfg.channel = TIM_MATCH_0;
	tim1Cfg.intEn = ENABLE;
	tim1Cfg.resetEn = ENABLE;
	tim1Cfg.stopEn = DISABLE;
	tim1Cfg.extOpt = TIM_NOTHING;
	tim1Cfg.matchValue = (SystemCoreClock / 4u) - 1u;
	TIM_ConfigMatch(LPC_TIM1, &tim1Cfg);
	NVIC_EnableIRQ(TIMER1_IRQn);
}
void TIMER1_IRQHandler(void){ // Reemplaza a TIMER0_IRQHandler
	TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
	if(seg_sin_respi<255u) seg_sin_respi++; // Evita que el uint8_t dé la vuelta y se desborde
	if(seg_sin_respi>ref_apnea) BAND_APNEA=1u;
}

void DMA_IRQHandler(void){
	if(GPDMA_IntGetStatus(GPDMA_INTTC, GPDMA_CH_0)==SET){
		GPDMA_ClearIntPending(GPDMA_CLR_INTTC, GPDMA_CH_0);
		BLOQUE_LISTO^=1;
		BAND_DMA=1;
		LPC_GPIO3->FIOCLR = MASK_LED1;
	}
	if(GPDMA_IntGetStatus(GPDMA_INTTC, GPDMA_CH_2)==SET){
		GPDMA_ClearIntPending(GPDMA_CLR_INTTC, GPDMA_CH_2);
		UART_LIBRE=1u;
	}
	if(GPDMA_IntGetStatus(GPDMA_INTERR, GPDMA_CH_2)==SET){
		GPDMA_ClearIntPending(GPDMA_CLR_INTERR, GPDMA_CH_2);
		UART_LIBRE=1u;
	}
}

int main(void){
	// El DAC arranca en el centro (512) en vez de en 0 V, con 512 queda en la línea central desde el principio por eso necesito alcanzar los 1,65 pico a pico
	for(int j=0; j<2; j++){
		for(int k=0; k<2*T_BLOQUE; k++){
			BUFFER_DAC[j][k]=(512u<<6)|(1u<<16);
		}
	}
	CONFIGPINES();
	CONFIGUART0();
	CONFIGADC();
	CONFIGGPDMA_ADC();
	CONFGPDMA_DAC();
	CONFIGDAC();
	CONFIGTIMER0();
	CONFIGTIMER1();
	TIM_Enable(LPC_TIM0);//Arranco los timers
	TIM_Enable(LPC_TIM1);
	while(1){
		if(BAND_DMA==1){
			BAND_DMA=0u;
			GPIO_SetPins(PORT_3, MASK_LED1);
			b=BLOQUE_LISTO;
			PUEDO_ENVIAR=UART_LIBRE;
			N_TX=0u;
			for(int i=0; i<T_BUFFER; i++){
				x=(int32_t)((BUFFER_DMA[b][i]>>4)&0xFFFUL) - 2048;
				RECONST += x - (RECONST>>7);
				BASE += (RECONST-BASE)>>9;
				SIGNAL_CENT=RECONST-BASE;
				if(SIGNAL_CENT>MAX_CICLO) MAX_CICLO=SIGNAL_CENT;
				if(SIGNAL_CENT<MIN_CICLO) MIN_CICLO=SIGNAL_CENT;
				v=sat(512+(SIGNAL_CENT>>ESC_DAC));
				N_DESDE_CRUCE++;
				//Valor en bits 15:6, BIAS en el 16, cada muestra repetida dos veces
				BUFFER_DAC[b][2*i]  =((uint32_t)v<<6)|(1u<<16);
				BUFFER_DAC[b][2*i+1]=((uint32_t)v<<6)|(1u<<16);
				if(PUEDO_ENVIAR==1)
					N_TX += snprintf(TX_BUF+N_TX, TX_MAX-N_TX, "D,%d\n", (int)SIGNAL_CENT);
				if((ESTADO==0) & (SIGNAL_CENT>HISTER)){
					ESTADO=1;
					F_RESPI=(60*F_MUESTREO)/N_DESDE_CRUCE;
					N_DESDE_CRUCE=0u; //Reinicia la cuenta del período
					BAND_APNEA=0u;
					seg_sin_respi=0u;
					ESFUERZO=MAX_CICLO-MIN_CICLO;
					MAX_CICLO=0;
					MIN_CICLO=0;
					if(PUEDO_ENVIAR==1)
						N_TX+=snprintf(TX_BUF+N_TX, TX_MAX-N_TX, "R,%u,%u\n", (unsigned)F_RESPI, (unsigned)ESFUERZO);

				}
				else{
					if((ESTADO==1)&(SIGNAL_CENT<-HISTER)){
						ESTADO=0u;
					}
				}
			}
			//Se ejecuta una vez por bloque, fuera del for
			if(BAND_APNEA==1){
				GPIO_ClearPins(PORT_3, MASK_LED1);
				GPIO_SetPins(PORT_0, MASK_LED);
				GPIO_SetPins(PORT_0, MASK_BUZZER);
				//COSAS DE UART
				if((EN_ALARMA==0)&&(PUEDO_ENVIAR==1)){ // Línea A, una sola vez
					EN_ALARMA=1u;
					N_TX+=snprintf(TX_BUF+N_TX, TX_MAX-N_TX, "A,APNEA,%u\n", (unsigned)seg_sin_respi);
				}
			}
			else{
				GPIO_SetPins(PORT_3, MASK_LED1);
				GPIO_ClearPins(PORT_0, MASK_LED);
				GPIO_ClearPins(PORT_0, MASK_BUZZER);
				if((EN_ALARMA==1)&&(PUEDO_ENVIAR==1)){ //Vuelve la respiración
					EN_ALARMA=0u;
					N_TX+=snprintf(TX_BUF+N_TX, TX_MAX-N_TX, "A,OK\n");
				}
			}
			if((PUEDO_ENVIAR==1)&&(N_TX>0u)){ //Un solo envío por bloque
				ENVIAR_UART(N_TX);
			}
			}
		}
	}
