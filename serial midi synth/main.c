/*****************************************************************************/
/* USB MIDI Synth main File                                                  */
/*                                                                           */
/* Copyright (C) 2016 Laszlo Arvai                                           */
/* source: https://github.com/hightower70/MIDIUSB                            */
/* used fork: https://github.com/Cynthetika/MIDIUSB                          */
/* All rights reserved.                                                      */
/* Modified by David Rubio 08/06/2019                                        */
#define CHANNEL_COUNT 12
#include <stm32f10x.h>
#include <sample.h>
#include <drum.h>
uint8_t led_state[CHANNEL_COUNT];
const uint8_t gpios[4]={GPIO_ODR_ODR0,GPIO_ODR_ODR1,GPIO_ODR_ODR2,GPIO_ODR_ODR3};
const uint16_t gpios_modes[4]={GPIO_CRL_MODE0_0,GPIO_CRL_MODE1_0,GPIO_CRL_MODE2_0,GPIO_CRL_MODE3_0};
uint8_t count3;
//Charlieplex LED Scan Table,
// 0 = High impedance
// 1 = LOW
// 2 = HIGH
const uint8_t led_table[12][4]=
{
	{2,1,0,0},
	{1,2,0,0},
	{2,0,1,0},
	{1,0,2,0},
	{0,2,1,0},
	{0,1,2,0},
	{0,2,0,1},
	{0,1,0,2},
	{0,0,2,1},
	{0,0,1,2},
	{2,0,0,1},
	{1,0,0,2},
};
const int freq[128] =
{
  16, 17, 18, 19, 21, 22, 23, 24, 26, 28, 29, 31, 33, 35, 37, 39, 41,
  44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110,
  117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233,
  247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
  523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047,
  1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
  2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729,
  3951, 4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040,
  7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544,
  13290, 14080, 14917, 15804, 16744, 17740, 18795, 19912, 21096,
  22351, 23680, 25088
};
//formula
// ((sampling rate of sample/freq of the note)/timer1 interrupt freq)*2^11
const float increment_per_hert=1.565599539;
//variables
uint8_t midi_ready;
uint8_t chanused[CHANNEL_COUNT];
uint8_t volume[CHANNEL_COUNT];
uint8_t byte_count=0;
uint8_t note[CHANNEL_COUNT];
uint8_t chanchannel[CHANNEL_COUNT];
uint8_t playednotes[CHANNEL_COUNT];
uint8_t mididata1[3];
uint8_t percussionused[2];
uint8_t playedpercussion[2];
long counter[CHANNEL_COUNT];
uint8_t manual_sustain;
int count=0;
uint16_t count2=0;
uint8_t sustain=0;
long percussion_counter[2];
unsigned int increment[CHANNEL_COUNT];
void handlechans(uint8_t mididata,uint8_t midichannel) {
  uint32_t maxValue=0;
  uint8_t big=0;
  //check if the note has already been played on any of the channels
  //if true replay it on the same channel and return
  for (int a = 0; a != CHANNEL_COUNT; a++) {
    if (mididata == playednotes[a]) {
      //write the midi channel of this note into the chanchannel variable
      chanchannel[a]=midichannel;
      //write the midinote of this note to the note variable (confusing?)
      note[a]=mididata;
      //set the counter to 0 so that the playback starts again
      counter[a] = 0;
      //write the increment needed for the frequency that we want
      increment[a] = increment_per_hert*freq[mididata];
      //set the chanused boolean
      chanused[a] = 1;
      return;
    }
  }
  //check which channel is free so that we can play a note on it
  for (int a = 0; a != CHANNEL_COUNT; a++) {
    if (!chanused[a]) {
      chanchannel[a]=midichannel;
      note[a]=mididata;
      counter[a] = 0;
      increment[a] = increment_per_hert*freq[mididata];
      chanused[a] = 1;
      playednotes[a] = mididata;
      return;
    }
    //check which note is the oldest that has been played
    if (counter[a] > maxValue) {
      maxValue = counter[a];
      big = a;
    }
  }
  //if no channel is free overwrite the oldest one
  note[big]=mididata;
  chanchannel[big]=midichannel;
  counter[big] = 0;
  increment[big] = increment_per_hert*freq[mididata];
  chanused[big] = 1;
  playednotes[big] = mididata;

}
// same as handlechans but for percussion
void handlepercussion(uint8_t mididata) {
  uint32_t maxValue=0;
  uint8_t big=0;
  //check if the note has already been played on any of the channels
  //if true replay it on the same channel and return
  for (int a = 0; a != 2; a++) {
    if (mididata == playedpercussion[a]) {
    	//i know i could have used a switch function but anyway if conditions still work fine
    	if(mididata==35||mididata==36){
    						drum_sample[a]=kick;
    						drum_length[a]=sizeof(kick);
    					}
    						else if(mididata==37||mididata==39){
    						drum_sample[a]=clap;
    						drum_length[a]=sizeof(clap);
    					}
    						else if(mididata==38||mididata==40||mididata==41||mididata==43){
    						drum_sample[a]=snare;
    						drum_length[a]=sizeof(snare);
    					}
    						else if(mididata==42||mididata==39){
    						drum_sample[a]=close_hi_hat;
    						drum_length[a]=sizeof(close_hi_hat);
    					}
    						else if(mididata==44||mididata==46){
    						drum_sample[a]=open_hi_hat;
    						drum_length[a]=sizeof(open_hi_hat);
    					}
    						else if(mididata==49||mididata==51||mididata==52||mididata==55||mididata==57||mididata==59){
    						drum_sample[a]=splash;
    						drum_length[a]=sizeof(splash);
    					}
    						else{
    						drum_sample[a]=clap;
    						drum_length[a]=sizeof(clap);
    						}
    						percussion_counter[a]=0;
    						percussionused[a] = 1;
      return;
    }
  }
  //check which channel is free so that we can play a note on it
  for (int a = 0; a != 2; a++) {
    if (!percussionused[a]) {
    	if(mididata==35||mididata==36){
    						drum_sample[a]=kick;
    						drum_length[a]=sizeof(kick);
    					}
    						else if(mididata==37||mididata==39){
    						drum_sample[a]=clap;
    						drum_length[a]=sizeof(clap);
    					}
    						else if(mididata==38||mididata==40||mididata==41||mididata==43){
    						drum_sample[a]=snare;
    						drum_length[a]=sizeof(snare);
    					}
    						else if(mididata==42||mididata==39){
    						drum_sample[a]=close_hi_hat;
    						drum_length[a]=sizeof(close_hi_hat);
    					}
    						else if(mididata==44||mididata==46){
    						drum_sample[a]=open_hi_hat;
    						drum_length[a]=sizeof(open_hi_hat);
    					}
    						else if(mididata==49||mididata==51||mididata==52||mididata==55||mididata==57||mididata==59){
    						drum_sample[a]=splash;
    						drum_length[a]=sizeof(splash);
    					}
    						else{
    						drum_sample[a]=clap;
    						drum_length[a]=sizeof(clap);
    						}
    						percussion_counter[a]=0;
    						percussionused[a] = 1;
      playedpercussion[a] = mididata;
      return;
    }
    //check which note is the oldest that has been played
    if (percussion_counter[a] > maxValue) {
      maxValue = percussion_counter[a];
      big = a;
    }
  }
  //if no channel is free overwrite the oldest one

	if(mididata==35||mididata==36){
						drum_sample[big]=kick;
						drum_length[big]=sizeof(kick);
					}
						else if(mididata==37||mididata==39){
						drum_sample[big]=clap;
						drum_length[big]=sizeof(clap);
					}
						else if(mididata==38||mididata==40||mididata==41||mididata==43){
						drum_sample[big]=snare;
						drum_length[big]=sizeof(snare);
					}
						else if(mididata==42||mididata==39){
						drum_sample[big]=close_hi_hat;
						drum_length[big]=sizeof(close_hi_hat);
					}
						else if(mididata==44||mididata==46){
						drum_sample[big]=open_hi_hat;
						drum_length[big]=sizeof(open_hi_hat);
					}
						else if(mididata==49||mididata==51||mididata==52||mididata==55||mididata==57||mididata==59){
						drum_sample[big]=splash;
						drum_length[big]=sizeof(splash);
					}
						else{
						drum_sample[big]=clap;
						drum_length[big]=sizeof(clap);
						}
						percussion_counter[big]=0;
						percussionused[big] = 1;
  playedpercussion[big] = mididata;

}
void main(void)
{
	//set the percussion counter higher than the sample length (which is 0, so that 1 > 0)
	percussion_counter[0]=1;
	percussion_counter[1]=1;
	//set the volume and panning to its normal value on all channel generators
	for(int a=0;a!=CHANNEL_COUNT;a++){
		volume[a]=128;
		//put channel indicator of the channel generators (thats a lot of channels) out of range so that we do not write data to it accidentally
		chanchannel[a]=255;
	}
	//set the clock signal to all of these registers (GPIO port A,B and C ; Timer1,2,3 ; Alternative Function IO), USART1.
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
        //Set PC13 as Push Pull output
	GPIOC ->CRH = GPIO_CRH_MODE13_0;
        //Set PC13 High (On-board LED is active low)
	GPIOC ->ODR = GPIO_ODR_ODR13;
	//set pin PB11 and PB10 to Alternative Function Output push-pull
	GPIOB ->CRH = GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_0 | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0;
        //Set pins PA0 to PA3 to Push Pull outputs, Set PA4 As Input
	GPIOA ->CRL = GPIO_CRL_MODE3_0 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE0_0 | GPIO_CRL_CNF4_1;
        //Turn on PA4 internal Pull-Up resistor
	GPIOA ->ODR = GPIO_ODR_ODR4;
        //Set PA10 as Input 
	GPIOA ->CRH = GPIO_CRH_CNF10_1;
        //Enable USART Recieve mode and RX Interrupt
	USART1->CR1 = USART_CR1_RE | USART_CR1_RXNEIE;
        //Set Baud rate to 31250 bauds
        //Formula F_CPU/16/Baud Rate
        //the lower 4 bits of the BRR register are fractions and the Higher 12 bits are for whole numbers
	USART1->BRR=(144<<4);
	TIM1->PSC=100; //timer1 prescaler set to 100
	TIM1->ARR=18; //timer1 Auto reload set to 18
	TIM1->DIER = TIM_DIER_UIE; // Timer1 update interrupt enable
	TIM3->PSC=400; //timer3 prescaler set to 400
	TIM3->ARR=200; //timer3 Auto reload set to 200
	TIM3->DIER = TIM_DIER_UIE; // Timer1 update interrupt enable
	TIM2->PSC = 1; //timer2 prescaler set to 1
	TIM2->ARR = 1023; //timer1 Auto reload set to 1023 (this is the PWM resolution 10bit)
	AFIO->MAPR = AFIO_MAPR_TIM2_REMAP_1; //use the alternative function io outputs for timer2
	TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0; // Non inverting PWM mode on channel 4, inverting PWM mode on channel and 3
	TIM2->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E; // Enable compare on channel 4, 3
	AFIO ->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PA; //enable the external interrupt function of PA4
	EXTI->IMR|=EXTI_IMR_MR4; //enable interrupt triggering of MR4 (aka PA4,PB4,PC4. etc...)
	EXTI->FTSR |= EXTI_FTSR_TR4;//make it trigger by a falling edge of the MR4 mask (only PA4 is used)
	NVIC_EnableIRQ(EXTI4_IRQn);//enable external interrupts from 10-15 interrupt
	NVIC_SetPriority(EXTI4_IRQn,1);//set it to the second highest priority
	NVIC_EnableIRQ(TIM1_UP_IRQn); // enable timer1 interrupt update
	NVIC_EnableIRQ(TIM3_IRQn); // enable timer3 interrupt
	NVIC_EnableIRQ(USART1_IRQn); // enable USART1 RX Interrupt
	NVIC_SetPriority(USART1_IRQn,0); //set it to the second priority
	TIM1->CR1 = TIM_CR1_CEN;  //enable timer1
	TIM2->CR1 = TIM_CR1_CEN;  //enable timer2
	TIM3->CR1 = TIM_CR1_CEN;  //enable timer2
	USART1->CR1 |= USART_CR1_UE; //enable USART1
	while(1){
		for(int a=0;a!=CHANNEL_COUNT;a++){
                //Poll the state of the 12 voices and set them in to the corresponding LED
		if(chanused[a])led_state[a]=1;
		else led_state[a]=0;
	}
	if(midi_ready){
		midi_ready=0;
        //pitch bend routine
		//check if this isnմ comming from channel 10
		if((mididata1[0] & 0xF0)==0xE0 && mididata1[0]!=0xE9){
			for(int a=0;a!=CHANNEL_COUNT;a++){
				//remember the chanchannel variable?
				//well, here it is used for checking on which channel generetors we are going to apply the pitch bend
				if(chanchannel[a]==(mididata1[0] & 0x0F)){
					//if the high byte of the pitch bend controller is 64 (at the middle)
					//just write the original freq, while skipping the next mumbo jumbo
					if(note[a]){
					if(mididata1[2]==0x40){
						increment[a]=increment_per_hert*freq[note[a]];
					}
					else{
						//get the difference between the range of the pitch bend according to the note of the channel generetor (range of the pitch bend is +/- 2 semitones/midi notes)
					volatile int difference=freq[note[a]+2]-freq[note[a]-2];
					//oh my, this looks quite complex
					//let me drink some cheetah chug
					//multiply the difference by the pitch bend value (range is from 0-127)
					//then just divided by 127 making this a rule of 3
					//add the lower freq (freq[note[a]-2]) so that we are on the correct one
					//finally multiply this by the increment per hert constant
					increment[a]=increment_per_hert*(((difference*mididata1[2])>>7)+freq[note[a]-2]);
					}
					}
				}
			}
		}
        //control change data from the usb midi (ignore channel 10 aka percussion)
		if((mididata1[0] & 0xF0)==0xB0 && mididata1[0]!=0xB9){
			//sustain
			if(mididata1[1]==0x40){
				//if the sustain knob if higher than 50, turn on sustain
				if(mididata1[2]>50)sustain=1;
				else sustain=0;
			}
			//volume control
			if(mididata1[1]==0x07){
				for(int a=0;a!=CHANNEL_COUNT;a++){
					//check on which channel generetor we are going to apply the volume
					if(chanchannel[a]==(mididata1[0] & 0x0F))volume[a]=mididata1[2];
				}
			}
			//all notes off
			if(mididata1[1]==0x78){
				//this is self explanatory
				for(int a=0;a!=CHANNEL_COUNT;a++){
				//check on which channel generator to silence the playback of the notes
			    if(chanchannel[a]==(mididata1[0] & 0x0F)){
			    //put it out of range so that we do not write data to it accidentally
			    chanchannel[a]=255;
			    counter[a] = size<<11;
				chanused[a] = 0;
				note[a]=0;
				}
				}
			}
			//all controls reset
			if(mididata1[1]==0x79){
				for(int a=0;a!=CHANNEL_COUNT;a++){
					//check on which channel generator to reset the controls
					//put it out of range so that we do not write data to it accidentally
					chanchannel[a]=255;
					volume[a]=128;
			}
				sustain=0;
		  }
		}
		
		//if there is no sustain, stop the playing of the sample of the specified channel (ignore channel 10 aka percussion)
		if(!manual_sustain){
			if(((mididata1[0] & 0xF0)==0x80 || ((mididata1[0] & 0xF0)==0x90 && !mididata1[2])) && mididata1[0]!=0x89 && !sustain){
				  for (int a = 0; a != CHANNEL_COUNT; a++) {
				    if (mididata1[1]-11 == playednotes[a]) {
				    	//Note: there are bit-shifts (<<) in here since the increment per hert variable
				    	//has been multiplied by 2^11 to avoid using floats and thus making the calculations way faster at the interrupt routine
				      counter[a] = size<<11;
				      chanused[a] = 0;
					  chanchannel[a]=255;
					  playednotes[a]=255;
				      break;
				    }
				  }

			}
		}
			// play the incoming midi note (ignore channel 10 aka percussion)
			if((mididata1[0] & 0xF0)==0x90 && mididata1[2]){
				if(mididata1[0]!=0x99){
				handlechans(mididata1[1]-11,(mididata1[0] & 0x0F));
				}

				else handlepercussion(mididata1[1]);

			}


		}
}
}
uint32_t millis;
uint32_t lastDebounceTime;
void EXTI4_IRQHandler(){
	if(EXTI->PR & EXTI_PR_PR4){
		//reset the external interrupt flag (PR4)
		EXTI->PR |= EXTI_PR_PR4;
		//check if PA4 is low
		if(!(GPIOA ->IDR & GPIO_IDR_IDR4)){
			//check if the input has stay of for more than 270 ticks (tick = 1sec/900) for de-bouncing
				  if ((millis - lastDebounceTime) > 270)
				  {
				    lastDebounceTime = millis;
				    //change the value of the manual sustain "bool"
				    manual_sustain = !manual_sustain;
                                    //Use PC13 As manual sustain Indicator
					GPIOC ->ODR ^= GPIO_ODR_ODR13;
				  }
				}
	}
}
uint8_t expected_count;
void USART1_IRQHandler(){
    //Serial midi Routine
	if(USART1->SR & USART_SR_RXNE){
		USART1->SR &= ~ USART_SR_RXNE;
		uint8_t data=USART1->DR;
		 if(data & 0x80){
			 byte_count=0;
			if(((data & 0xF0) == 0xC0) || ((data & 0xF0) == 0xD0)) expected_count=2;
			else expected_count=3;
		 }
		 mididata1[byte_count]=data;
		 byte_count++;
		 if(byte_count==expected_count)midi_ready=1;
	}
}
void TIM1_UP_IRQHandler(void)
{
	//timer interrupt, this happens 40000 times per second since 72000000(F_CPU)/prescaler/ARR=40000
if(TIM1->SR & TIM_SR_UIF) // if UIF flag is set
  {

	TIM1->SR &= ~TIM_SR_UIF; //clear the UIF flag for not getting stuck on this interrupt
	 //offsets
	 volatile uint16_t pcm_data=512;
	 //audio channel mixing and pcm playing
	 for(int a=0;a!=CHANNEL_COUNT;a++){
		 //add the increment of the channel to the counter of itself, the store that value in a volatile variable
	 volatile uint32_t temp=counter[a]+increment[a];
	 //Note: there are bit-shifts (>>) in here since the increment per hert variable has been multiplied by 2^11 to avoid using floats and thus making the calculations way faster
	 //if we still donմ reach the end off the sample
	if((temp>>11)<size){
	//put the previous sum to the channel counter
	counter[a]=temp;
	//remove the dc offset of the sample (from 0 to 255 to -127 to 127)
	//get the value of the sample that we need according to the counter of a channel
	//add that value in to the n variable
	//multiplied it by the volume (0-127) then dividing it by 127 and then by 2 since there are 12 channels so that we get 1/2 of the volume thus it will not clip
	//add the panning to each channel
	//put all of these on a variable so that we do not have to recalculate all of these values
	pcm_data+=((data[counter[a]>>11]-128)*volume[a])>>8;
	}

	//if we reach the end of the sample set the chanused boolean to 0 of the specific channel
	else if (chanused[a]){
		chanused[a]=0;
		playednotes[a]=255;
		chanchannel[a]=255;
	}
	}
	 //handle the percussion channels (only 2)
	 for(int a=0;a!=2;a++){
    //if the counter the percussion has not reach the end of the sample continue
    if(percussion_counter[a]<drum_length[a]*2){
    	//check if the number is not odd
    	if(!(percussion_counter[a] % 2)){
    		//get the sample value of the pointer (*drumsample)
    		//since the interrupt freq is 40khz and this samples are 20khz just divide the counter by 2
    		//remove the dc offset
    		//make them half of the volume so that it does not clip the audio
    		//put all of these on a variable so that we do not have to recalculate all of these values
    		pcm_data+=((*(drum_sample[a]+(percussion_counter[a]/2)))-128);

    	}
    	//increase the percussion_counter
    	percussion_counter[a]++;
    }
    //if we reach the end of the sample set the percussionused boolean to 0 of the specific channel
    else if (percussionused[a]) percussionused[a]=0;
	 }
	 //write the n variable in to the pwm chan4 register
	 TIM2->CCR4=pcm_data;
	 TIM2->CCR3=pcm_data;

  }

}
//CharliePlex Routine
void TIM3_IRQHandler(void)
{	
if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
	TIM3->SR &= ~TIM_SR_UIF; //clear the UIF flag for not getting stuck on this interrupt
        //Increase the "millis" counter
	millis++;
	for(int a=0;a!=4;a++){
        //Set the charlieplex output to high impedance
	GPIOA ->CRL &= ~gpios_modes[a];
	GPIOA ->ODR &= ~gpios[a];
	}
        //Check if the led_state "bool" of the corresponding LED is set
	if(led_state[count3]){
            //Set the necessary states of the Charlieplex pins according to the CharliePlex LED Table
	for(int a=0;a!=4;a++){
		if(led_table[count3][a]){
			GPIOA ->CRL |= gpios_modes[a];
		if(led_table[count3][a] == 2 )GPIOA ->ODR |= gpios[a];
		}
	}
	}
        //incremente the count3 variable by one if we still dont reach 11;
        //else set the count3 variable to 0;
	count3==11 ? count3=0 : count3++;
  }

}

//end of file