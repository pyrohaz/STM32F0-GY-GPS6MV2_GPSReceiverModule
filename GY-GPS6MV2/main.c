#include <stm32f0xx_usart.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>

/*
 * GY-GPS6MV2 GPS receiver module example program using
 * the STM32F0 Discovery board available from STMicroelectronics
 *
 * Author: Harris Shallcross
 * Year: ~13/9/2014
 *
 *An example of using the GY-GPS6MV2 GPS receiver with the STM32F0
 *Discovery board. Sentences are received and separated into
 *different buffers with flags controlling whether these sentences
 *can be overwritten. Flags are also present to tell when a sentence
 *has been received.
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2014 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

//USART definitions, USART RX is PA3
#define G_UTX GPIO_Pin_2
#define G_URX GPIO_Pin_3

#define G_UTXPS GPIO_PinSource2
#define G_URXPS GPIO_PinSource3

#define G_GPIOAF GPIO_AF_1

#define G_GPIO GPIOA

#define G_USART USART2
#define G_BRate 9600

//Peripheral typedefs
GPIO_InitTypeDef G;
USART_InitTypeDef U;
NVIC_InitTypeDef N;

//USART send functions, unused in this example!
void G_Send(char C){
	USART_SendData(G_USART, C);
	while(USART_GetFlagStatus(G_USART, USART_FLAG_BUSY) == SET);
	while(USART_GetFlagStatus(G_USART, USART_FLAG_IDLE) == RESET);
	while(USART_GetFlagStatus(G_USART, USART_FLAG_TXE) == RESET);
	while(USART_GetFlagStatus(G_USART, USART_FLAG_TC) == RESET);
}

void G_SendBuf(char *C, uint32_t N){
	uint32_t Cnt;

	for(Cnt = 0; Cnt<N; Cnt++){
		G_Send(C[Cnt]);
	}
}

//Simple string compare function. This function compares strings
//up to "Amnt" places. It will also terminate the compare if the end
//of a string is met before Amnt has been met, returning 0 to say the
//strings aren't equal to Amnt places.
uint8_t StrCmp(char *StrA, char *StrB, uint8_t Amnt){
	uint8_t Cnt;

	//Check through string and compare. If Amnt > either string length
	//Ensure for loop quits at terminator character
	for(Cnt = 0; Cnt<Amnt && StrA[Cnt] != 0 && StrB[Cnt] != 0; Cnt++){
		//for(Cnt = 0; Cnt<Amnt; Cnt++){
		if(StrA[Cnt] != StrB[Cnt]){
			break;
		}
	}

	//If Cnt and Amnt are equal, return 1, otherwise return 0
	return(Cnt == Amnt);
}

//Copy a string from source to destination to Amnt places
void StrCpy(char *StrSrc, char *StrDest, uint8_t Amnt){
	uint8_t Cnt;

	for(Cnt = 0; Cnt<Amnt; Cnt++){
		StrDest[Cnt] = StrSrc[Cnt];
	}
}

//Copy a string from source to destination until a certain
//character is met in the source string. Really useful for
//copying variable length strings terminated by a specific
//character!
void StrCpyCh(char *StrSrc, char *StrDest, char Char){
	uint8_t Index = 0;

	while(StrSrc[Index] != Char){
		StrDest[Index] = StrSrc[Index];
		Index++;
	}
}

//Search through a string looking for the positions of all commas within
//that string.
void CommaPositions(char *Sentence, int16_t *Pos, int16_t *Lengths){
	uint16_t ChPos = 0, CPos = 0, CChar = 1, Cnt;

	while(CChar != '\0'){
		CChar = Sentence[ChPos];

		if(CChar == ','){
			Pos[CPos] = ChPos;
			CPos++;
		}

		ChPos++;
	}

	for(Cnt = 0; Cnt<CPos-1; Cnt++){
		Lengths[Cnt] = Pos[Cnt+1] - Pos[Cnt] - 1;
	}
}

//USART RX Interrupt variables
volatile uint8_t GSent = 0, GGAWrite = 1, GLLWrite = 1, GSAWrite = 1;
volatile uint8_t GSVWrite = 1, RMCWrite = 1, VTGWrite = 1;
volatile uint8_t GGAGot = 0, GLLGot = 0, GSAGot = 0, GSVGot = 0;
volatile uint8_t RMCGot = 0, VTGGot = 0;

//Variables to store the GPS Sentences
volatile char GGASnt[200], GLLSnt[200], GSASnt[200];
volatile char GSVSnt[200], RMCSnt[200], VTGSnt[200];

//The main part where all the magic happens, the USART RX interrupt!
void USART2_IRQHandler(void){
	char CChar = 0;
	uint8_t Cnt;
	static uint8_t SentenceBegin = 0, SentenceCnt = 0, Sentences = 0;
	static char TmpBuf[200];

	//If the USART RX Interrupt has been called
	if(USART_GetITStatus(G_USART, USART_IT_RXNE) == SET){
		//Clear the interrupt bit
		USART_ClearITPendingBit(G_USART, USART_IT_RXNE);

		//Receive the current character that triggered the USART RX
		//interrupt.
		CChar = USART_ReceiveData(G_USART);

		//Beginning of sentence character as defined by the NMEA
		//standard.
		if(CChar == '$'){
			//If the beginning character has been received, set the
			//SentenceBegin flag, reset the sentence position counter
			//and tell the main loop that a GPS sentence hasn't been
			//received (GSent = 0).
			SentenceBegin = 1;
			SentenceCnt = 0;
			GSent = 0;
		}

		//End of sentence character
		if(CChar == '\n' && SentenceBegin){
			SentenceBegin = 0;

			//Sentence has been received
			//Write each sentence to sentence buffer
			if(StrCmp(TmpBuf, "$GPGGA", 6)){
				//If sentence is alright to be written to, i.e.
				//the sentence isn't being written or read outside
				//of the interrupt.
				if(GGAWrite) StrCpyCh(TmpBuf, GGASnt, '\r');

				//Current sentence has successfully written.
				GGAGot = 1;
			}
			else if(StrCmp(TmpBuf, "$GPGLL", 6)){
				if(GLLWrite) StrCpyCh(TmpBuf, GLLSnt, '\r');
				GLLGot = 1;
			}
			else if(StrCmp(TmpBuf, "$GPGSA", 6)){
				if(GSAWrite) StrCpyCh(TmpBuf, GSASnt, '\r');
				GSAGot = 1;
			}
			else if(StrCmp(TmpBuf, "$GPGSV", 6)){
				if(GSVWrite) StrCpyCh(TmpBuf, GSVSnt, '\r');
				GSVGot = 1;
			}
			else if(StrCmp(TmpBuf, "$GPRMC", 6)){
				if(RMCWrite) StrCpyCh(TmpBuf, RMCSnt, '\r');
				RMCGot = 1;
			}
			else if(StrCmp(TmpBuf, "$GPVTG", 6)){
				if(VTGWrite) StrCpyCh(TmpBuf, VTGSnt, '\r');
				VTGGot = 1;
			}

			//Clear temporary buffer
			for(Cnt = 0; Cnt<200; Cnt++) TmpBuf[Cnt] = 0;

			//Set sentence received flag
			GSent = 1;
		}

		//If the sentence has just started, store whole sentence
		//in the temporary buffer
		if(SentenceBegin){
			TmpBuf[SentenceCnt] = CChar;
			SentenceCnt++;
		}
	}
}

int main(void)
{
	//Initialize Peripherals
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//Set up the USART GPIO pins, USART_TX isn't really needed!
	G.GPIO_Pin = G_UTX | G_URX;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_Speed = GPIO_Speed_Level_3;
	G.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &G);

	//Pin alternate function configuration
	GPIO_PinAFConfig(G_GPIO, G_UTXPS, G_GPIOAF);
	GPIO_PinAFConfig(G_GPIO, G_URXPS, G_GPIOAF);

	//Configure the USART Peripheral
	U.USART_BaudRate = G_BRate;
	U.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	U.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	U.USART_Parity = USART_Parity_No;
	U.USART_StopBits = USART_StopBits_1;
	U.USART_WordLength = USART_WordLength_8b;
	USART_Init(G_USART, &U);

	//Disable the overrun detection.
	USART_OverrunDetectionConfig(G_USART, USART_OVRDetection_Disable);

	//Clear the IT pending bit for RX and enable the interrupt!
	USART_ClearITPendingBit(G_USART, USART_IT_RXNE);
	USART_ITConfig(G_USART, USART_IT_RXNE, ENABLE);

	//Enable the USART module
	USART_Cmd(G_USART, ENABLE);

	//Enable the USART interrupt
	N.NVIC_IRQChannel = USART2_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	uint8_t Cnt;

	while(1)
	{
		//Wait until all 6 sentences have been acquired
		while(!GLLGot || !GSAGot || !GGAGot || !GSVGot || !RMCGot || !VTGGot);

		//Tell the interrupt that it can't write to the sentence buffers
		GSAWrite = GLLWrite = GGAWrite = GSVWrite = RMCWrite = VTGWrite = 0;

		//Debugging breakpoint - Read the sentences here!

		//Clear the sentence buffers, ready to receive new sentences
		for(Cnt = 0; Cnt<200; Cnt++){
			GSASnt[Cnt] = GLLSnt[Cnt] = GGASnt[Cnt] = GSVSnt[Cnt] = RMCSnt[Cnt] = VTGSnt[Cnt] = 0;
		}

		//Tell the interrupt that it can now write to the sentence buffers
		GSAWrite = GLLWrite = GGAWrite = GSVWrite = RMCWrite = VTGWrite = 1;

		//Reset all the sentence received variables.
		GSAGot = GLLGot = GGAGot = GSVGot = RMCGot = VTGGot = 0;
	}
}
