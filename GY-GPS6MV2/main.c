#include <stm32f0xx_usart.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include <math.h>

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

#define GMT

#define LatIndex 14
#define LongIndex 27

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
volatile uint8_t GSent = 0, GGAWrite = 1, GLLWrite = 1, GSAWrite = 1, SentenceWrite = 1;
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
			if(StrCmp(&TmpBuf[3], "GGA", 3)){
				//If sentence is alright to be written to, i.e.
				//the sentence isn't being written or read outside
				//of the interrupt.
				if(GGAWrite) StrCpyCh(&TmpBuf[3], GGASnt, '\r');

				//Current sentence has successfully written.
				GGAGot = 1;
			}
			else if(StrCmp(&TmpBuf[3], "GLL", 3)){
				if(GLLWrite) StrCpyCh(&TmpBuf[3], GLLSnt, '\r');
				GLLGot = 1;
			}
			else if(StrCmp(&TmpBuf[3], "GSA", 3)){
				if(GSAWrite) StrCpyCh(&TmpBuf[3], GSASnt, '\r');
				GSAGot = 1;
			}
			else if(StrCmp(&TmpBuf[3], "GSV", 3)){
				if(GSVWrite) StrCpyCh(&TmpBuf[3], GSVSnt, '\r');
				GSVGot = 1;
			}
			else if(StrCmp(&TmpBuf[3], "RMC", 3)){
				if(RMCWrite) StrCpyCh(&TmpBuf[3], RMCSnt, '\r');
				RMCGot = 1;
			}
			else if(StrCmp(&TmpBuf[3], "VTG", 3)){
				if(VTGWrite) StrCpyCh(&TmpBuf[3], VTGSnt, '\r');
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

//A integer pow function, not really required as math.h is now included
//but I like as little dependencies as possible! Returns a^x if x>1, if
//x=1, returns a, if x = 0, returns 1, on the non existent off chance,
//0 is returned though this will never happen as Pow is unsigned!
int32_t FPow(int32_t Num, uint32_t Pow){
	int32_t NumO = Num;
	uint32_t Cnt;
	if(Pow>1){
		for(Cnt = 0; Cnt<Pow-1; Cnt++){
			Num*=NumO;
		}
		return Num;
	}

	if(Pow==1) return Num;
	if(Pow==0) return 1;
	else return 0;
}

//Check the length of a number in base10 digits
uint8_t CheckNumLength(int32_t Num){
	uint8_t Len = 0, Cnt;

	for(Cnt = 0; Cnt<10; Cnt++){
		if(Num>=FPow(10, Cnt)){
			Len = Cnt;
		}
		else{
			Len = Cnt;
			break;
		}
	}

	return Len;
}

//Integer absolute value function
int32_t Abs(int32_t Num){
	if(Num<0) return -Num;
	else return Num;
}

//Floating point absolute value function
float FAbs(float Num){
	if(Num<0) return -Num;
	else return Num;
}

//Parse an integer number for length amount of characters, while
//also checking if the number is negative or not. "Sentence" is a
//pointer to a the first character of a string e.g. Str[] = "hi, 12345"
//the value of &Str[4] should be passed to the function, otherwise the
//text will be parsed and the number will be erroneous!
int32_t ParseInt(char *Sentence, uint8_t Len){
	int32_t Num = 0;
	int8_t Cnt;

	if(Sentence[0] == '-'){
		for(Cnt = 0; Cnt<Len; Cnt++){
			Num += (Sentence[Cnt+1]-'0')*FPow(10, (Len-1)-Cnt);
		}

		return -Num;
	}
	else{
		for(Cnt = 0; Cnt<Len; Cnt++){
			Num += (Sentence[Cnt]-'0')*FPow(10, (Len-1)-Cnt);
		}

		return Num;
	}
}

//Parsing a floating point number! Remember here that floating point
//numbers don't have infinite precision so this function only works
//up to a prec value of ~7.
float ParseFloat(char *Sentence, uint8_t Prec){
	uint8_t Cnt, CChar = 0, CCnt = 0, NumBuf[10] = {0,0,0,0,0,0,0,0,0,0};
	int32_t INumPre, INumPost;

	while(CChar != '.'){
		CChar = Sentence[CCnt];
		NumBuf[CCnt] = CChar;
		CCnt++;
	}

	INumPre = ParseInt(NumBuf, CCnt-1);

	for(Cnt = CCnt; Cnt<(CCnt+Prec); Cnt++){
		NumBuf[Cnt-CCnt] = Sentence[Cnt];
	}

	INumPost = ParseInt(NumBuf, Prec);

	return (float)INumPre + (float)INumPost/(float)FPow(10, Prec);
}

//Parse any decimal number returning two integers. One of these integers will be
//the digits before the decimal point, the other will be the digits after the
//decimal point. This function has a special addition because if EndPoint is <10,
//the parser will parse to a number amount of places (32bit integers only store
//up to 10 digits anyway). If the EndPoint is more than 10, it will search
//through the string looking for the EndPoint character. This is really useful for
//parsing the GGA string as Lat and Long are sent as decimal values and all text
//is delimited by commas.
void ParseDec(char *Sentence, int32_t *PreDec, int32_t *PostDec, uint8_t EndPoint){
	uint8_t Cnt, CChar = 0, CCnt = 0, CCVal, NumBuf[10] = {0,0,0,0,0,0,0,0,0,0}, Dir;
	int32_t INumPre, INumPost;

	while(CChar != '.'){
		CChar = Sentence[CCnt];
		NumBuf[CCnt] = CChar;
		CCnt++;
	}

	INumPre = ParseInt(NumBuf, CCnt-1);

	if(EndPoint<10){
		for(Cnt = CCnt; Cnt<(CCnt+EndPoint); Cnt++){
			NumBuf[Cnt-CCnt] = Sentence[Cnt];
		}

		INumPost = ParseInt(NumBuf, CCnt);
	}
	else{
		CCVal = CCnt;
		while(CChar != EndPoint){
			CChar = Sentence[CCnt];
			NumBuf[CCnt-CCVal] = CChar;
			CCnt++;
		}

		INumPost = ParseInt(NumBuf, CCnt-CCVal-1);

		if(Sentence[CCnt] == 'S' || Sentence[CCnt] == 'W'){
			INumPre = -INumPre;
		}
	}

	*PreDec = INumPre;

	*PostDec = INumPost;
}

//Parse the current time from any of the sentences containing
//time information! Time is normally presented in the form
//hhmmss.sss. This only parses hhmmss. Make sure the string
//sent to this function starts from the numbers! E.g. if you
//want to parse the string Str[] = "hi 123456.00", make sure you
//give the function &Str[3]. Time is returned to the array sent
//to the function as Time. This array will contain 3 values,
//Hours, Minutes and Seconds. As time is UTC and I live in UK,
//I've added a GMT define which shifts the time by 1 hour. Time
//is returned in 12 hour format.
void ParseTime(char *Sntce, uint8_t *Time){
#ifdef GMT
	Time[0] = (1+(Sntce[1]-'0')+(Sntce[0]-'0')*10)%12;
#else
	Time[0] = (Sntce[1]-'0')+(Sntce[0]-'0')*10;
#endif
	Time[1] = (Sntce[3]-'0')+(Sntce[2]-'0')*10;
	Time[2] = (Sntce[5]-'0')+(Sntce[4]-'0')*10;
}

//A parser to parse the latitude and longitude sent to the function
//from NMEA style (ddmm.mmmmm) into degrees, minutes and seconds. To
//make sure that floating point errors don't occur in data being
//being received by the function, ValPre and ValPost are the two numbers
//before and after the decimal point, in decimal lat/long format.
void ParseLatLong(int32_t ValPre, int32_t ValPost, int32_t *Deg, int32_t *Min, float *Sec){
	int32_t DegT, MinT;
	float SecT;

	DegT = ValPre/100;
	MinT = ValPre-DegT*100;
	SecT = (float)(ValPost*60)/(float)FPow(10, CheckNumLength(ValPost));

	*Deg = DegT;
	*Min = Abs(MinT);
	*Sec = FAbs(SecT);
}

//A function that converts the deg/min/sec lat and long into their
//decimal equivalent. This is the opposite of the above function!
float DecLatLong(int32_t Deg, int32_t Min, float Sec){
	float Num;

	Num = (float)Abs(Deg) + (float)Min/60 + Sec/3600;

	if(Deg<0) return -Num;
	else return Num;
}

//Calculate the distance and bearing between two points in km. The
//equations for this were from:
//
//http://www.ig.utexas.edu/outreach/googleearth/latlong.html
//
//These equations can be widely found everywhere so the above website
//was just one of the first clicks on google! The source lat/long,
//and destination lat/long are taken by the function and the distance
//and bearing between the two points is returned. This function
//normally uses the Haversine method though commented out is the
//spherical law of cosines method - deemed less accurate at times.
//The bearing value is returned in degrees, relative to North.
void DistanceBetweenPoints(float LatSrc, float LongSrc, float LatDst, float LongDst, float *Distance, float *Bearing){
	float DeltaLong, DeltaLat, A, C, TBearing;
	//Radius in km
	const float R = 6371.0f;

	LatSrc = LatSrc*M_PI/180.0f;
	LongSrc = LongSrc*M_PI/180.0f;

	LatDst = LatDst*M_PI/180.0f;
	LongDst = LongDst*M_PI/180.0f;

	DeltaLong = LongDst - LongSrc;
	DeltaLat = LatDst - LatSrc;

	A = sinf(DeltaLat*0.5f)*sinf(DeltaLat*0.5f) + cosf(LatSrc)*cosf(LatDst)*sinf(DeltaLong*0.5f)*sinf(DeltaLong*0.5f);
	C = 2*atan2f(sqrtf(A), sqrtf(1-A));

	*Distance = R*C;

	//*Distance = acosf(sinf(LatSrc)*sinf(LatDst)+cosf(LatSrc)*cosf(LatDst)*cosf(DeltaLong))*R;

	//Relative to north!
	TBearing = 360.0f + 180.0f*atan2f(sinf(DeltaLong)*cosf(LatDst), cosf(LatSrc)*sinf(LatDst)-sinf(LatSrc)*cosf(LatDst)*cosf(DeltaLong))/M_PI;

	//The floating point modulo equivalent! Successively subtract
	//360 until the value is less than 360.
	while(TBearing>360.0f){
		TBearing -= 360.0f;
	}
	*Bearing = TBearing;
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

	//Variables to hold various data, a lot!
	uint8_t Time[3], Cnt;

	//Lat/Long pre and post decimal point
	int32_t LatPre, LatPost;
	int32_t LongPre, LongPost;

	//Lat/Long degrees, minutes and seconds
	int32_t LatDeg, LatMin;
	int32_t LongDeg, LongMin;
	float LatSec, LongSec;

	//Lat/Long decimal values
	float LatDec, LongDec;

	//Distance and bearing variables
	float Dst, Bea;

	//An example destination
	const float LatDst = 53.465868f;
	const float LongDst = -2.348242f;

	while(1)
	{
		//Wait until all 6 sentences have been acquired
		while(!GLLGot || !GSAGot || !GGAGot || !GSVGot || !RMCGot || !VTGGot);

		//Tell the interrupt that it can't write to the sentence buffers
		GSAWrite = GLLWrite = GGAWrite = GSVWrite = RMCWrite = VTGWrite = 0;

		//Parse the current time of fix from the GLL Sentence
		ParseTime(&GLLSnt[31], Time);

		//Parse the latitude and longitude, into NMEA decimal form from the GGA
		//sentence. Numbers are parsed up to the ',' character (see, that was
		//useful really!).
		ParseDec(&GGASnt[LatIndex], &LatPre, &LatPost, ',');
		ParseDec(&GGASnt[LongIndex], &LongPre, &LongPost, ',');

		//Parse the degrees, minutes and seconds from the NMEA decimal form
		ParseLatLong(LatPre, LatPost, &LatDeg, &LatMin, &LatSec);
		ParseLatLong(LongPre, LongPost, &LongDeg, &LongMin, &LongSec);

		//Convert the deg/min/sec notation into standard decimal notation
		LatDec = DecLatLong(LatDeg, LatMin, LatSec);
		LongDec = DecLatLong(LongDeg, LongMin, LongSec);

		//Calculate the distance from our current position to the destination
		//defined above! Store the distance and bearing in the variables Dst
		//and Bea.
		DistanceBetweenPoints(LatDec, LongDec, LatDst, LongDst, &Dst, &Bea);

		//Debugging breakpoint - Read the data here!

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
