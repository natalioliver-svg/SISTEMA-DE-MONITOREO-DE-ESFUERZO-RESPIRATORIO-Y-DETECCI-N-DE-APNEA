#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>
#include <math.h> ///LINEA MARTI
#include <stdio.h>
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"


#define N_SEN 50u //LINEA MAR
#define MASK_BUZZER (0x01UL)
#define MASK_LED (1<<22)
#define T_BUFFER 50u
#define T_BLOQUE 50u
#define ref_apnea 11u
#define F_MUESTREO 100u
#define DAC_BIAS (1u<<16) // Bit BIAS de DACR (350 uA)
#define UMBRAL_INSP 5u   // Si x está por encima de este valor = inspiración
#define UMBRAL_ESP 1000u   // De forma que quede por encima del valor de Inspiración y por debajo del valor de reposo
//Declaración de arreglos para manejos principales de GPDMA
volatile uint32_t BUFFER_DAC[2][2*T_BLOQUE];//2 Bloques de 100 palabras cada uno
volatile uint32_t BUFFER_DMA[2][T_BLOQUE];//2 Bloques de 50 palabras cada uno
//Declaración global de listas enlazadas
static GPDMA_LLI_T lli_adc[2];
static GPDMA_LLI_T lli_dac[2];
//Declaración de flags para manejo entre interrupciones y main()
volatile uint8_t BLOQUE_LISTO= 1u;
volatile uint8_t UART_LIBRE = 1u;
volatile uint8_t BAND_DMA =0u;
volatile uint8_t BAND_NO_APNEA =1u;
volatile uint8_t ESTADO =0u; //ESTADO=1 => ESPIRANDO ; ESTADO=0 => INSPIRANDO
//Contadores y variables de cálculos/reconstrucción de señales
volatile uint8_t seg_sin_respi =0u;
volatile uint32_t b =0u;
volatile int32_t x =0u;
volatile int32_t v =0u;
volatile uint32_t CANT_RESPI =0u;
volatile uint32_t CANT_RESPI_UART =0u;
volatile int32_t tick =0u;
//Variables de manejo de UART
#define TX_MAX 1200u //Margen de 24 bytes para evitar líneas cortadas
char TX_BUF[TX_MAX]; //Una línea por muestra procesada
volatile uint32_t N_TX =0u;
volatile uint8_t PUEDO_ENVIAR =0u; //Vigila que no se sobreescriba TX_BUF mientras el DMA lo está transmitiendo

void CONFIGPINES(void){
	//Configuro LED P0.22 (rojo) para alarma de apnea (activo por bajo)
	PINSEL_CFG_T pinCfg={0};
    pinCfg.port = 0;
	pinCfg.func = 0;
    pinCfg.mode = PINSEL_TRISTATE; //Ni pull-up, ni pull-down
    pinCfg.openDrain = DISABLE; //Deshabilito open drain para que los niveles lógicos se lean como tal (que 1 entregue tensión)
    pinCfg.pin = 22;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(PORT_0,MASK_LED ,GPIO_OUTPUT); //Pongo como salida el pin del LED

    //Configuro buzzer para alarma de apnea
    pinCfg.port = 0;
	pinCfg.func = 0;
    pinCfg.mode = PINSEL_TRISTATE; //Ni pull-up, ni pull-down
    pinCfg.openDrain = DISABLE; //Deshabilito open drain para que los niveles lógicos se lean como tal (que 1 entregue tensión)
    pinCfg.pin = 0;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(PORT_0, MASK_BUZZER ,GPIO_OUTPUT);//Pongo como salida el pin del buzzer

}
void CONFIGADC(void){
	ADC_Init(100000);
	ADC_PinConfig(ADC_CHANNEL_0); // P0.23
	ADC_StartCmd(ADC_START_ON_MAT01); //Habilito conversión con el match del Timer0
	ADC_ChannelEnable(ADC_CHANNEL_0);
	ADC_EdgeStartConfig(ADC_START_ON_FALLING); //Match configurado por flanco de bajada
	ADC_IntEnable(ADC_INT_CH0); //Habilito interrupción para generar la petición al DMA
}
void CONFIGDAC(void){
	DAC_CONVERTER_CFG_T dacCfg={0};
	dacCfg.doubleBuffer = ENABLE; //El valor nuevo espera al fin del conteo
	dacCfg.dmaRequest = ENABLE; //Habilito petición al GPDMA al terminar el contador
	dacCfg.dmaCounter = ENABLE; //Activo el contador de tiempo del DAC
	DAC_Init(); //Configuro P0.26 como AOUT
	LPC_SC->PCLKSEL0 |= (3u << 22); // PCLK del DAC = CCLK/8 = 12,5 MHz
	DAC_SetBias(DAC_350uA);
	DAC_ConfigDAConverterControl(&dacCfg);
	/*El BUFFER del DMA tiene 50 muestras por cada mitad y el ADC entrega 100 por segundo,
	 * entonces cada mitad se llena a los 0,5s (50/100) para sacar las 100 palabras en ese tiempo de llenado (100/0,5=200Hz)
	 */
	DAC_SetDMATimeOut((SystemCoreClock / 8u) / (F_MUESTREO * 2u) - 1u);
}
void CONFIGUART0(void){
	LPC_SC->PCONP |= (1u << 3); //Enciendo UART0
	LPC_SC->PCLKSEL0 &= ~(3u << 6); // PCLK = CCLK/4 = 25 MHz
	LPC_PINCON->PINSEL0 &= ~((3u << 4) | (3u << 6));
	LPC_PINCON->PINSEL0 |=  ((1u << 4) | (1u << 6)); // P0.2 = TXD0, P0.3 = RXD0
	LPC_UART0->FCR = 0x0F; // FIFO, reset RX/TX y modo DMA
	LPC_UART0->LCR |= (1u << 7); // DLAB = 1
	LPC_UART0->DLL = 12u;
	LPC_UART0->DLM = 0u;
	LPC_UART0->FDR = (8u << 4) | 1u; // 115.741 baudios (0,5 % de error) - Bloque de 50 líneas (1000 bytes) en 90 ms < 0,5s del bloque
	LPC_UART0->LCR = 0x03; // 8N1 y DLAB = 0
}
void CONFIGGPDMA_ADC(void){
	GPDMA_Init();
	NVIC_EnableIRQ(DMA_IRQn);
	/*Tamaño de la transfer | Ancho del origen/ADDR(32b) | Ancho del dest/BUFFER_DMA(32b)
	| Habilito incremento del destino | Habilito interrupcion de fin de transfer*/
	uint32_t ctl = T_BUFFER | (2u << 18) | (2u << 21) | (1u << 27) | (1u << 31);
	lli_adc[0].srcAddr = (uint32_t)&LPC_ADC->ADDR0; //Marco que las muestras vienen del ADC
	lli_adc[0].dstAddr = (uint32_t)BUFFER_DMA[1]; //Lleno la segunda mitad de 50 muestras del BUFFER
	lli_adc[0].nextLLI = (uint32_t)&lli_adc[1]; //Al terminar paso a la siguiente lista enlazada
	lli_adc[0].control = ctl;
	lli_adc[1].srcAddr = (uint32_t)&LPC_ADC->ADDR0; //Marco que las muestras vienen del ADC
	lli_adc[1].dstAddr = (uint32_t)BUFFER_DMA[0]; //Vuelvo a llenar la primera mitad de 50 muestras del BUFFER
	lli_adc[1].nextLLI = (uint32_t)&lli_adc[0]; //Repito el ciclo de lli
	lli_adc[1].control = ctl;
	GPDMA_Channel_CFG_T gpdmaCfg={0};
	//Primera vuelta del GPDMA, después se maneja entre lli
	gpdmaCfg.channelNum=GPDMA_CH_0;
	gpdmaCfg.transferSize=T_BLOQUE;//50 muestras
	gpdmaCfg.type=GPDMA_P2M;//Periférico a memoria
	gpdmaCfg.srcMemAddr=0;
	gpdmaCfg.dstMemAddr=(uint32_t)BUFFER_DMA[0];//Destino en memoria
	gpdmaCfg.srcConn=GPDMA_ADC;//Establezco que las muestras vienen del ADC
	gpdmaCfg.dstConn=0;
	gpdmaCfg.src.width=GPDMA_WORD;
	gpdmaCfg.src.burst=GPDMA_BSIZE_1; //Configuro DMA para una ráfaga de transferencia
	gpdmaCfg.src.increment=DISABLE;
	gpdmaCfg.dst.width=GPDMA_WORD;
	gpdmaCfg.dst.burst=GPDMA_BSIZE_1;
	gpdmaCfg.dst.increment=ENABLE;
	gpdmaCfg.intTC=ENABLE;
	gpdmaCfg.intErr=DISABLE;
	gpdmaCfg.linkedList=(uint32_t)&lli_adc[0]; //Al terminar de llenar BUFFER_DMA[0] va hacia lli_adc[0]
	GPDMA_SetupChannel(&gpdmaCfg);
	GPDMA_ChannelStart(GPDMA_CH_0);
}
void CONFGPDMA_DAC(void){
	/*Tamaño de la transfer | Ancho del origen/BUFFER_DAC(32b) | Ancho del dest/DACR(32b)
	| Habilito incremento del origen*/
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
	//Primera vuelta del GPDMA, después se maneja entre lli
	dmaCfg.channelNum    = GPDMA_CH_1;
	dmaCfg.transferSize  = 2u*T_BUFFER;//100 muestras
	dmaCfg.type          = GPDMA_M2P;//Memoria a Periférico
	dmaCfg.srcConn       = 0;
	dmaCfg.dstConn       = GPDMA_DAC; //El destino es el DAC
	dmaCfg.srcMemAddr    = (uint32_t)BUFFER_DAC[0]; //El origen es el BUFFER_DAC que se escribe en el main
	dmaCfg.dstMemAddr    = 0;
	dmaCfg.src.width     = GPDMA_WORD;
	dmaCfg.src.burst     = GPDMA_BSIZE_1;
	dmaCfg.src.increment = ENABLE;
	dmaCfg.dst.width     = GPDMA_WORD;
	dmaCfg.dst.burst     = GPDMA_BSIZE_1;
	dmaCfg.dst.increment = DISABLE;
	dmaCfg.intTC         = DISABLE;
	dmaCfg.intErr        = DISABLE;
	dmaCfg.linkedList    = (uint32_t)&lli_dac[0]; //Al terminar de llenar BUFFER_DAC[0] va hacia lli_dac[0]
	GPDMA_SetupChannel(&dmaCfg);
	GPDMA_ChannelStart(GPDMA_CH_1);
}
void ENVIAR_UART(uint32_t n){
	GPDMA_Channel_CFG_T uartCfg={0};
	UART_LIBRE=0u; //Marco la UART como ocupada
	uartCfg.channelNum=GPDMA_CH_2; //El canal 0 está ocupado por el ADC, el 1 por el DAC, el 2 es el libre
	uartCfg.transferSize=n; //Cantidad de caracteres generados por bloque/ byte
	uartCfg.type=GPDMA_M2P; //Memoria a periférico
	uartCfg.srcMemAddr=(uint32_t)TX_BUF;
	uartCfg.dstMemAddr=0;
	uartCfg.srcConn=0;
	uartCfg.dstConn=GPDMA_UART0_Tx;
	uartCfg.src.width=GPDMA_BYTE;
	uartCfg.src.burst=GPDMA_BSIZE_1;
	uartCfg.src.increment=ENABLE;
	uartCfg.dst.width=GPDMA_BYTE;
	uartCfg.dst.burst=GPDMA_BSIZE_1;
	uartCfg.dst.increment=DISABLE;
	uartCfg.intTC=ENABLE;
	uartCfg.intErr=ENABLE;
	uartCfg.linkedList=0;
	if(GPDMA_SetupChannel(&uartCfg)==ERROR){
		UART_LIBRE=1u;
		return;
	}
	GPDMA_ChannelStart(GPDMA_CH_2);
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
void CONFIGTIMER0(void){
	TIM_TIMERCFG_T timCfg={0};
	timCfg.prescaleOpt= TIM_TICK; //Cuento ticks del reloj del periférico, un tick cada 40ns
	timCfg.prescaleValue= 0; //Contador sin prescaler
	TIM_InitTimer(LPC_TIM0, &timCfg);
	TIM_MATCHCFG_T tim0Cfg={0};
	tim0Cfg.channel = TIM_MATCH_1;
	tim0Cfg.intEn = DISABLE;
	tim0Cfg.resetEn = ENABLE;
	tim0Cfg.stopEn = DISABLE;
	tim0Cfg.extOpt = TIM_TOGGLE;/*Invierto el estado del pin de salida de match (MAT0.1) en cada match
	=> el pin cambia cada 5 ms, una oscilación completa tarda 10 ms*/
	tim0Cfg.matchValue = (SystemCoreClock / (2u * F_MUESTREO) / 4u) - 1u;//100M / 200 / 4 = 125 000 (5ms entre match)
	TIM_ConfigMatch(LPC_TIM0, &tim0Cfg);
}
void DMA_IRQHandler(void){
	if(GPDMA_IntGetStatus(GPDMA_INTTC, GPDMA_CH_0)==SET){ //Bloque del ADC lleno
		GPDMA_ClearIntPending(GPDMA_CLR_INTTC, GPDMA_CH_0);
		BLOQUE_LISTO^=1;
		BAND_DMA=1;
	}
	if(GPDMA_IntGetStatus(GPDMA_INTTC, GPDMA_CH_2)==SET){ //Terminó el envío por UART
		GPDMA_ClearIntPending(GPDMA_CLR_INTTC, GPDMA_CH_2);
		UART_LIBRE=1u;
	}
	if(GPDMA_IntGetStatus(GPDMA_INTERR, GPDMA_CH_2)==SET){ // Error, para no quedar trabado
		GPDMA_ClearIntPending(GPDMA_CLR_INTERR, GPDMA_CH_2);
		UART_LIBRE=1u;
	}
}
void TIMER1_IRQHandler(void){
	TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
	if(seg_sin_respi<255u) seg_sin_respi++; //Controlo que no se desborde, si llega a 255 no lo sigo incrementando
	tick++; //Contador para minuto
	if(tick>=60){ //Cada minuto
		tick=0; //Reinicia la cuenta
		CANT_RESPI_UART=CANT_RESPI; //Respiraciones del último minuto, no se limpia, solo se actualiza
		CANT_RESPI=0u;//Se limpia para contabilizar las respiraciones en un nuevo minuto
	}
}
int main(void){
	for(int j=0; j<2; j++){ //El DAC arranca en el centro (512)
		for(int k=0; k<2*T_BLOQUE; k++){
			BUFFER_DAC[j][k]=(512u<<6)|DAC_BIAS;
		}
	}
	CONFIGPINES();
	//Apago tanto el LED como el Buzzer
	GPIO_SetPins(PORT_0, MASK_LED);
	GPIO_ClearPins(PORT_0,MASK_BUZZER);
	CONFIGUART0();
	CONFIGADC(); //Primero el ADC, después su DMA
	CONFIGGPDMA_ADC();
	CONFIGDAC(); //Primero el DAC, después su DMA
	CONFGPDMA_DAC();
	CONFIGTIMER0();
	CONFIGTIMER1();
	TIM_Enable(LPC_TIM1);
	TIM_Enable(LPC_TIM0); //Recién acá arranca el ADC
	while(1){
		if(BAND_DMA==1){ //Se ejecuta cuando el bloque del ADC está lleno
			BAND_DMA=0u;
			b=BLOQUE_LISTO; //Me fijo cuál mitad es la que se llenó
			PUEDO_ENVIAR=UART_LIBRE; //Guardo el estado actual de UART_LIBRE
			N_TX=0u; //Vacío el buffer para que se escriba desde el inicio

			//Promedio del bloque (filtro la fluctuación del sensor)
			int32_t suma = 0;
			for(int i=0; i<T_BUFFER; i++){
				suma += (int32_t)((BUFFER_DMA[b][i]>>4)&0xFFFUL);
			}
			int32_t xprom = suma / (int32_t)T_BUFFER; // 0..4095
			int32_t amp = xprom >> 3; // 0..511

			//Detección, senoidal y texto por UART
			for(int i=0; i<T_BUFFER; i++){
				x = (int32_t)((BUFFER_DMA[b][i]>>4)&0xFFFUL); //Corro ADDR para guardar el resultado de la conversión desde el bit 0 en x
				if((ESTADO==0)&&(x<UMBRAL_INSP)){ //Detección de inspiración, umbral según gráfica de cuentas del ADC
					ESTADO=1u; //Cambio de estado a inspiración
					CANT_RESPI++; //Incremento del contador de la cantidad de respiraciones
					seg_sin_respi=0u; //Por cada respiración nueva reinicio la cuenta de apnea
				}
				else if((ESTADO==1)&&(x>UMBRAL_ESP)){ //Vuelve a subir al estado de reposo, se cambia de estado a espiración=0
					ESTADO=0u;
				}
				//Se reconstruye la señal para el DAC como senoidal
				v = 512 + (int32_t)((float)amp * sinf(2.0f*3.14159265f*(float)i/(float)N_SEN));
				//Se llena el BUFFER_DAC en la posición par y la siguiente impar por la diferencia de 100Hz con el ADC
				BUFFER_DAC[b][2*i]   = ((uint32_t)v<<6)|DAC_BIAS;
				BUFFER_DAC[b][2*i+1] = ((uint32_t)v<<6)|DAC_BIAS;
				/*Si la UART se encuentra libre y la posición del indice es menor al
				maximo del buffer, cargo N_TX*/
				if((PUEDO_ENVIAR==1)&&(N_TX<(TX_MAX-24u)))
					N_TX+=snprintf(TX_BUF+N_TX, TX_MAX-N_TX, "D,%d,%d,%d,%d\n", (int)x, (int)CANT_RESPI, (int)CANT_RESPI_UART, (int)v);
			}
			//Detección de apnea, LED y buzzer una sola vez por bloque, fuera del for
			BAND_NO_APNEA = (seg_sin_respi<=ref_apnea) ? 1u : 0u; //Cambia el estado de BAND_NO_APNEA según si la condición es V:F
			if(BAND_NO_APNEA==1u){
				GPIO_SetPins(PORT_0, MASK_LED);
				GPIO_ClearPins(PORT_0, MASK_BUZZER);
			}
			else{//Prendo la alarma de detección
				GPIO_ClearPins(PORT_0, MASK_LED);
				GPIO_SetPins(PORT_0, MASK_BUZZER);
			}

			if((PUEDO_ENVIAR==1)&&(N_TX>0u)){ //Pregunto si la UART está libre y si la cantidad de bytes para enviar es mayor a 0
				ENVIAR_UART(N_TX); //Ejecuto la función
			}
		}
	}
}
