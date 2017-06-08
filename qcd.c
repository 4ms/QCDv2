//To Calibrate the Div.Mult knob positions:
//
//Power off
//Set all knobs to 0
//Hold down the TAP button
//Power on while continuing to hold down the button
//Release the button
//The QCD will be in calibration mode
//
//To Calibrate:
//The first channel that needs to be calibrated will have its light on
//Turn the knob up one position (note: the botom two positions are the same, so you have to turn it up two positions the first time)
//The channel light should go off and the TAP button light will go on
//Tap the TAP button to save the knob position
//The channel light should go on and the TAP button light will go off
//
//Repeat the procedure (turn knob up one position, tap the button, turn knob up, tap the button...)
//...Until you get to the last position. 
//Then the light for the next channel will turn on.
//Continue calibrating the next channel, repeating for each channel
//When all channels are calibrated, a short light sequence will play.
//The calibration data is saved into EEPROM.
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

//#define DEBUG

/********************
 * GLOBAL VARIABLES *
 ********************/

#define HOLDTIMECLEAR 16000
//100 is 12ms
//80 is 10ms
#define MIN_PW 100
#define SLOW_PERIOD 160
#define DIV_ADC_DRIFT 2
#define PW_ADC_DRIFT 3
#define USER_INPUT_POLL_TIME 4


/*******************
 *   EEPROM        *
 *******************/

#define EEPROM_SET 0b10101010
#define EEPROM_CLEAR 0b00000000
#define EEPROM_UNPROGRAMMED 0b11111111

#define ISCAL_EEPROMADDR 2
#define CAL_EEPROMADDR 8

/********************
 *    DIV MULT      *
 ********************/

#define P_1 32
#define P_2 16
#define P_3 8
#define P_4 7
#define P_5 6
#define P_6 5
#define P_7 4
#define P_8 3
#define P_9 2
#define P_10 1
#define P_11 -2
#define P_12 -3
#define P_13 -4
#define P_14 -5
#define P_15 -6
#define P_16 -7
#define P_17 -8
#define P_18 -12
#define P_19 -16


const int8_t P_array[19] = { P_1, P_2, P_3, P_4, P_5, P_6, P_7, P_8, P_9, P_10, P_11, P_12, P_13, P_14, P_15, P_16, P_17, P_18, P_19 };

uint8_t midpt_array[4][19];

/*************
 *  FREE RUN *
 *************/
// Set FREERUN to 1 to allow freerunning clock (output clock doesn't stop when incoming clock stops)
// Set FREERUN to 0 to disable freerunning clock (output will stop when input stops)
// Factory default is 0

#define FREERUN 0


/*******************
 * PIN DEFINITIONS *
 *******************/

#define TAP_pin PB4
#define TAP_init DDRB &= ~(1<<TAP_pin); PORTB |= (1<<TAP_pin)
#define TAPIN (!(PINB & (1<<TAP_pin)))

#define TAPOUT_pin PB5
#define TAPOUT_init DDRB |= (1 << TAPOUT_pin)
#define TAPOUT_ON clkout_state |= (1<<5)
#define TAPOUT_OFF clkout_state &= ~(1<<5)

#define IMMEDIATE_TAPOUT_ON PORTB |= (1 << TAPOUT_pin)
#define IMMEDIATE_TAPOUT_OFF PORTB &= ~(1 << TAPOUT_pin)


#define RESET_pins 0b11110000
#define RESET_init DDRD &= ~(RESET_pins); PORTD &= ~(RESET_pins)
#define RESET(x) (PIND & (1<<(x+4)))

#define PING_pins 0b00001111
#define PING_init DDRD &= ~(PING_pins); PORTD &= ~(PING_pins)
#define PING(x) (PIND & (1<<(x)))
#define PING_ALL (PIND & PING_pins)

#define CLKOUT_pins 0b00001111
#define CLKOUT_init DDRB |= CLKOUT_pins
#define IMMEDIATE_CLKOUT_ON(x) PORTB |= (1 << (x))
#define IMMEDIATE_CLKOUT_OFF(x) PORTB &= ~(1 << (x))

#define CLKOUT_ON(x) clkout_state |= (1<<(x))
#define CLKOUT_OFF(x) clkout_state &= ~(1 << (x))

#ifndef DEBUG
	//#define CLKOUT_SETSTATE(x) PORTB = (PORTB & 0b11110000) | (x)
	#define CLKOUT_SETSTATE(x) PORTB = (PORTB & 0b11010000) | (x)

	#define DEBUGON
	#define DEBUGOFF
#else
	#define CLKOUT_SETSTATE(x) PORTB = (PORTB & 0b11111000) | (x & 0b111)

	#define DEBUGON PORTB |= 0b1000
	#define DEBUGOFF PORTB &= ~0b1000

#endif

#define ADC_DDR DDRC
#define ADC_PORT PORTC
#define ADC_mask (0b00111111)
#define ADC_pins (0b00111111)
#define ADC_DIVMULT1 0
#define ADC_DIVMULT2 1
#define ADC_DIVMULT3 2
#define ADC_DIVMULT4 3
#define ADC_PW1 4
#define ADC_PW2 5
#define ADC_PW3 6
#define ADC_PW4 7

extern uint32_t udiv32(uint32_t divisor);
volatile uint32_t tapouttmr;
volatile uint32_t tapintmr;

volatile uint32_t tmr_ping[4]={0,0,0,0};
volatile uint32_t tmr_clkout[4]={0,0,0,0};
volatile uint32_t tmr_reset[4]={0,0,0,0};
volatile uint32_t ping_irq_timestamp[4]={0,0,0,0};

uint8_t clkout_state=0;
uint8_t clkout_update_ctr=0;
uint8_t enable_clkout=0;

SIGNAL (TIMER0_OVF_vect){
//DEBUGON;
cli();
	tapintmr++;
	tapouttmr++;

	tmr_ping[0]++;
	tmr_ping[1]++;
	tmr_ping[2]++;
	tmr_ping[3]++;
	tmr_reset[0]++;
	tmr_reset[1]++;
	tmr_reset[2]++;
	tmr_reset[3]++;
	tmr_clkout[0]++;
	tmr_clkout[1]++;
	tmr_clkout[2]++;
	tmr_clkout[3]++;
sei();
//DEBUGOFF;
}

uint8_t ping_state[4]={0,0,0,0};
volatile uint8_t got_ping[4]={0,0,0,0};

SIGNAL (PCINT2_vect){

	uint8_t i;
	//uint8_t tcnt0=TCNT0;
	
	for (i=0;i<4;i++){
		if (PING(i)){
			if (!(ping_state[i])){ 	//if jack is read high and it was remembered as being low
				ping_state[i]=1;  	//remember it as being high
				ping_irq_timestamp[i] = tmr_ping[i];
				//ping_irq_timestamp[i] = (tmr_ping[i] << 8) | tcnt0 ;
				tmr_ping[i]=0;
				got_ping[i]=1;
			}
		} else 
			ping_state[i]=0;		//remember it as being low
	}

}

void init_extinterrupt(void){
	PCICR = (1<<PCIE2); //Enable pin change interrupt for PCINT[23:16]
	PCMSK2 = (1<<PCINT16) | (1<<PCINT17) | (1<<PCINT18) | (1<<PCINT19); //Enable PCINT for 16-19 (PD0-PD3)
//	PCMSK2 = (1<<PCINT17); //Enable PCINT for 16-19 (PD0-PD3)

	//MCUCR = (1<<ISC00) | (1<<ISC01); //enable the external interrupt for Rising Edge on pin INT0 (PB2)
	//GIMSK = (1<<INT0); //enable interrupt for external int
}

uint32_t get_tapintmr(void){
	uint32_t result;
	cli();
	//result = (tapintmr << 8) | TCNT0;
	result = tapintmr;
	sei();
	return result;
}
uint32_t get_tapouttmr(void){
	uint32_t result;
	cli();
	//result = (tapouttmr << 8) | TCNT0;
	result = tapouttmr;
	sei();
	return result;
}
void reset_tapouttmr(void){
	cli();
	tapouttmr=0;
	sei();
}
void reset_tapintmr(void){
	cli();
	tapintmr=0;
	sei();
}


inline uint32_t get_tmr_clkout(uint8_t chan){
	uint32_t result;
	cli();
	result = tmr_clkout[chan];
	//result = (tmr_clkout[chan] << 8) | TCNT0;
	sei();
	return result;
}
inline uint32_t get_tmr_ping(uint8_t chan){
	uint32_t result;
	cli();
	result = tmr_ping[chan];
	sei();
	return result;
}
inline uint32_t get_tmr_reset(uint8_t chan){
	uint32_t result;
	cli();
	result = tmr_reset[chan];
	//result = (tmr_reset[chan] << 8) | TCNT0;
	sei();
	return result;
}
void init_adc(void){
	ADC_DDR &= ~(ADC_mask); //adc input
	ADC_PORT &= ~(ADC_mask); //disable pullup
	DIDR0 = ADC_mask; //turn off digital input buffer
	ADCSRA = (1<<ADEN);	//Enable ADC
	ADMUX = (1<<ADLAR) | (ADC_DIVMULT1);	//Left-Adjust, MUX to the ADC_pin
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //prescale = clk/128 = 125kHz
	ADCSRA |= (1<<ADSC);//set the Start Conversion Flag in the ADC Status Register
}
void blink_four() {
	uint8_t i;
	IMMEDIATE_CLKOUT_OFF(0);
	IMMEDIATE_CLKOUT_OFF(1);
	IMMEDIATE_CLKOUT_OFF(2);
	IMMEDIATE_CLKOUT_OFF(3);
	for (i=0; i<8; i++){
		IMMEDIATE_CLKOUT_ON(i & 0b11);	
		_delay_ms(90);
		IMMEDIATE_CLKOUT_OFF(i & 0b11);	
	}
}


void inittimer(void){
	cli();

	tapouttmr=0;
	tapintmr=0;
	tmr_clkout[0]=0;
	tmr_clkout[1]=0;
	tmr_clkout[2]=0;
	tmr_clkout[3]=0;
	tmr_ping[0]=0;
	tmr_ping[1]=0;
	tmr_ping[2]=0;
	tmr_ping[3]=0;
	tmr_reset[0]=0;
	tmr_reset[1]=0;
	tmr_reset[2]=0;
	tmr_reset[3]=0;

	//Normal mode, TOP at 0xFF, OC0A and OC0B disconnected, Prescale @ FCK/8
	TCCR0A=(0<<WGM01) | (0<<WGM00) ;
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);

	TCNT0=0;

	TIMSK0 |= (1<<TOIE0); 
						// Enable timer overflow interrupt
	sei();
}

void init_pins(void){
	TAP_init;
	TAPOUT_init;

	PING_init;
	RESET_init;
	CLKOUT_init;

#ifdef DEBUG_init
	DEBUG_init;
#endif
}


void default_calibration(void)
{
	uint8_t i;
	for (i=0;i<4;i++)
	{
		midpt_array[i][0] = 5; //0
		midpt_array[i][1] = 16; //10
		midpt_array[i][2] = 31; //23
		midpt_array[i][3] = 45; //38
		midpt_array[i][4] = 59; //
		midpt_array[i][5] = 72;
		midpt_array[i][6] = 86;
		midpt_array[i][7] = 99;
		midpt_array[i][8] = 112;
		midpt_array[i][9] = 125;
		midpt_array[i][10] = 137;
		midpt_array[i][11] = 151;
		midpt_array[i][12] = 164;
		midpt_array[i][13] = 177;
		midpt_array[i][14] = 191; 
		midpt_array[i][15] = 205; //198
		midpt_array[i][16] = 218; //211
		midpt_array[i][17] = 235; //225
		midpt_array[i][18] = 255; //245
	}

}


int8_t get_clk_div_nominal(uint8_t chan, uint8_t adc_val){
	uint8_t i;
	for (i=0;i<19;i++)
	{
		if (adc_val<=midpt_array[chan][i])
			return(P_array[i]);
	}
	return(P_19);
}

uint32_t get_clk_div_time(int8_t clock_divide_amount, uint32_t clk_time){

	if (clock_divide_amount==64)  // /64
		return(clk_time<<6);
	else if (clock_divide_amount==32) // /32
		return(clk_time<<5);
	else if (clock_divide_amount==16) // /16
		return(clk_time<<4);
	else if (clock_divide_amount==8) // /8
		return(clk_time<<3);
	else if (clock_divide_amount==4) // /4
		return(clk_time<<2);
	else if (clock_divide_amount==2) // /2
		return(clk_time<<1);
	else if (clock_divide_amount==1) // =1
		return(clk_time);
	else if (clock_divide_amount==0) // =1
		return(clk_time);
	else if (clock_divide_amount==-1) // =1
		return(clk_time);
	else if (clock_divide_amount==-2) // *2
		return(clk_time>>1);
	else if (clock_divide_amount==-4) // *4
		return(clk_time>>2);
	else if (clock_divide_amount==-8) // *8
		return(clk_time>>3);
	else if (clock_divide_amount==-16) // *16
		return((clk_time>>4)+1);
		
	else if (clock_divide_amount<0)
		return(clk_time/(-1*clock_divide_amount));
	else //if (clock_divide_amount>0)
		return(clk_time*clock_divide_amount);

}


uint32_t calc_pw(uint8_t pw_adc, uint32_t period){
	uint32_t t;

	pw_adc=255-pw_adc;

	if (pw_adc<4) t=(period>>6); //1.0625%
	else if (pw_adc<14) t=(period>>5); //3.125%
	else if (pw_adc<24) t=(period>>4); //6.25%
		//	else if (pw_adc<34) t=((period>>4)+(period>>6)); //7.8125%
	else if (pw_adc<34) t=((period>>4)+(period>>5)); //9.375%
		//	else if (pw_adc<44) t=((period>>4)+(period>>5)+(period>>6)); //10.9375%
	else if (pw_adc<44) t=(period>>3); //12.5%
	else if (pw_adc<54) t=((period>>3)+(period>>5)); //15.5%
	else if (pw_adc<64) t=((period>>3)+(period>>4)); //18.75%
	else if (pw_adc<74) t=((period>>3)+(period>>4)+(period>>5)); //21.875%
	else if (pw_adc<85) t=(period>>2); //25%
	else if (pw_adc<94) t=((period>>2)+(period>>4)); //31.25%
	else if (pw_adc<104) t=((period>>2)+(period>>3)); //37.5%
	else if (pw_adc<114) t=((period>>2)+(period>>3)+(period>>4)); //43.75%

	else if (pw_adc<140) t=(period>>1); //50%

	else if (pw_adc<150) t=((period>>1)+(period>>5)); //53.125%
	else if (pw_adc<160) t=((period>>1)+(period>>4)); //56.25%
	else if (pw_adc<170) t=((period>>1)+(period>>4)+(period>>5)); //59.375%
	else if (pw_adc<180) t=((period>>1)+(period>>3)); //62.5%
	else if (pw_adc<190) t=((period>>1)+(period>>3)+(period>>5)); //65.5%
	else if (pw_adc<200) t=((period>>1)+(period>>3)+(period>>4)); //68.75%
	else if (pw_adc<210) t=((period>>1)+(period>>3)+(period>>4)+(period>>5)); //71.875%
	else if (pw_adc<220) t=((period>>1)+(period>>2)); //75%
	else if (pw_adc<230) t=((period>>1)+(period>>2)+(period>>4)); //81.25%
	else if (pw_adc<240) t=((period>>1)+(period>>2)+(period>>3)); //87.5%
	else if (pw_adc<250) t=((period>>1)+(period>>2)+(period>>3)+(period>>4)); //93.75%
	else t=period-(period>>5); //96.875%

	if (period>(SLOW_PERIOD)){   //period is at least 30ms (lower than 33Hz) so we should use MIN_PW as a min/max
		if (pw_adc<4 || t<MIN_PW) t=MIN_PW;
		if (pw_adc>=250 || t>(period-MIN_PW)) t=period-MIN_PW; 

	}

	return(t);

}

uint8_t adc_read(uint8_t cur_adc) {

	uint8_t adch=127;

	ADMUX = (1<<ADLAR) | cur_adc;	//Setup for next conversion
	ADCSRA |= (1<<ADSC);			//Start Conversion
	
	while (!(ADCSRA & (1<<ADIF))) { ; }
	
	ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"

	adch=ADCH;
	
	return adch;	
}


uint8_t calib_mode_start(void) {
	uint8_t pot_zeroed_bitfield = 0b0000;
	uint8_t tapCounter = 0;
	uint8_t i;


	for(i=0;i<100;i++) {if (TAPIN) tapCounter++;}

	//_delay_ms(10);
	//if (TAPIN) tapCounter++;
	

	for (i=0;i<4;i++){
		if (adc_read(i) <=8)
			pot_zeroed_bitfield |= (1<<i);
	}

	_delay_ms(10);

	//if (TAPIN) tapCounter++;
	
	for(i=0;i<100;i++) {if (TAPIN) tapCounter++;}

	if (tapCounter > 150){
		return(pot_zeroed_bitfield);
	}
	else return(0);
}

void calibrate_pots(uint8_t chan_bitfield){
	uint16_t stab_delay=30;
	uint16_t calib_array[19];
	uint16_t read_tot;
	uint8_t read_avg;
	uint8_t read1, read2, read3, read4;
	uint8_t i,j;
	uint16_t t;
	uint8_t diff;

	for ( i = 0; i < 4; i++ ) {
		if (chan_bitfield & (1<<i))
		{
			for ( j = 0; j < 19; j++ ) {
				IMMEDIATE_CLKOUT_OFF(i); //off = reading pot

				_delay_ms(stab_delay);
				read1 = adc_read(i);
				_delay_ms(stab_delay);
				read2 = adc_read(i);
				_delay_ms(stab_delay);
				read3 = adc_read(i);
				_delay_ms(stab_delay);
				read4 = adc_read(i);

				read_tot = read1 + read2 + read3 + read4;
				read_avg = read_tot>>2;	
				
				calib_array[j] = read_avg;

				if (j<18){		
					IMMEDIATE_CLKOUT_ON(i); //on = ready for user to change knob

					if (j==0 || j==17) diff=5;
					else diff=10;

					_delay_ms(200);

					//wait until knob is detected as being moved
					do {   
						_delay_ms(stab_delay);
						read1 = adc_read(i);
						_delay_ms(stab_delay);
						read2 = adc_read(i);
						_delay_ms(stab_delay);
						read3 = adc_read(i);
						_delay_ms(stab_delay);
						read4 = adc_read(i);

						read_tot = read1 + read2 + read3 + read4;
						read_avg = read_tot >> 2;	
					} while ((read_avg - calib_array[j]) < diff);

					IMMEDIATE_TAPOUT_ON;
					IMMEDIATE_CLKOUT_OFF(i);
					t = 0;
					while (t<100) {if (TAPIN) t++; else t=0;}
					t = 0;
					while (t<100) {if (!TAPIN) t++; else t=0;}
					IMMEDIATE_TAPOUT_OFF;
				} //if j<18
			} //for j

			//convert the calib_array values to mid-points
			for(j=0;j<18;j++){
				midpt_array[i][j] = (calib_array[j] + calib_array[j+1]) >> 1;
			}
			midpt_array[i][18] = 255;
		} //if chan_bitfield
	} //for i
}

uint8_t is_calibrated(void)
{
	uint8_t t;

	t = eeprom_read_byte((const uint8_t *)(ISCAL_EEPROMADDR));
	if (t == EEPROM_SET)
		return(1);
	else
		return(0);
}

void read_calibration(void)
{
	eeprom_read_block((void*)&midpt_array, (const void *)(CAL_EEPROMADDR), 4*19);

}

uint8_t sanity_check_calibration(void)
{
	uint8_t i,j;

	for ( i = 0; i < 4; i++ ) {
		for ( j = 0; j < 18; j++ ) {
			if (midpt_array[i][j+1] <= midpt_array[i][j])
				return(0); //fail
		}	
	}
	return(1); //pass
}

void write_calibration(void)
{
	eeprom_busy_wait();
	eeprom_write_block((const void *)&midpt_array, (void *)(CAL_EEPROMADDR), 4*19);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)(ISCAL_EEPROMADDR), (uint8_t)EEPROM_SET);
	eeprom_busy_wait();
}

/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint32_t now=0;
	uint32_t nows[4]={0,0,0,0};
	uint32_t resets[4]={0,0,0,0};

	uint32_t t=0;

	uint32_t tapout_clk_time=0;
	char tapin_down=0, tapin_up=0;

	uint32_t ping_time[4]={0,0,0,0};
	uint32_t divclk_time[4]={0,0,0,0};
	uint32_t pw_time[4]={0,0,0,0};
	uint32_t reset_offset_time[4]={0,0,0,0};

	int poll_user_input=0;

	uint8_t reset_up[4]={0,0,0,0};
	uint8_t reset_now_flag[4]={0,0,0,0};
	uint8_t ready_to_reset[4]={1,1,1,1};


	
	uint8_t num_pings_since_reset[4][19] = {
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
	};
	int8_t num_divclks_since_ping[4]={0,0,0,0};

	uint8_t cda[4][19] = {
		{P_19,P_18,P_17,P_16,P_15,P_14,P_13,P_12,P_11,P_10,P_9,P_8,P_7,P_6,P_5,P_4,P_3,P_2,P_1},
		{P_19,P_18,P_17,P_16,P_15,P_14,P_13,P_12,P_11,P_10,P_9,P_8,P_7,P_6,P_5,P_4,P_3,P_2,P_1},
		{P_19,P_18,P_17,P_16,P_15,P_14,P_13,P_12,P_11,P_10,P_9,P_8,P_7,P_6,P_5,P_4,P_3,P_2,P_1},
		{P_19,P_18,P_17,P_16,P_15,P_14,P_13,P_12,P_11,P_10,P_9,P_8,P_7,P_6,P_5,P_4,P_3,P_2,P_1}
	};
	

	uint8_t i,j;
	uint8_t cur_chan=0;

	uint8_t adch=127;
	uint8_t divmult_adc[4]={127,127,127,127};
	uint8_t pw_adc[4]={127,127,127,127};

	int8_t clock_divide_amount[4]={1,1,1,1};
	int8_t old_clock_divide_amount[4]={1,1,1,1};

	uint8_t cur_adc=0;
	uint8_t next_adc=0;

	uint8_t reset_ck[4];
	uint8_t got_pings[4];

	uint8_t user_pots_calibrate = 0;	

	/** Initialize **/

	inittimer();
	init_pins();
	init_extinterrupt();
	init_adc();

	_delay_ms(5);

	TAPOUT_OFF;


	if (is_calibrated()) //reads calibration bit to see if we're calibrated
	{
		read_calibration(); //read from eeprom and store into midpt_array
		if (!sanity_check_calibration())
			default_calibration();
	} else {
		default_calibration();
	}	

	user_pots_calibrate = calib_mode_start();

	if (user_pots_calibrate) { 
		calibrate_pots(user_pots_calibrate);
		
		write_calibration();
		blink_four();
	}


	init_adc();

	CLKOUT_OFF(0);
	CLKOUT_OFF(1);
	CLKOUT_OFF(2);
	CLKOUT_OFF(3);

	/** Main loop **/
	while(1){


		/************ PING **********/

		cli();
		got_pings[0]=got_ping[0];
		got_pings[1]=got_ping[1];
		got_pings[2]=got_ping[2];
		got_pings[3]=got_ping[3];
		sei();

		for (i=0;i<4;i++){
			if (got_pings[i]){
				cli();
					ping_time[i]=ping_irq_timestamp[i];
					got_ping[i]=0;
					ping_irq_timestamp[i]=0;
					tmr_reset[i]=0;
				sei();

				clock_divide_amount[i] = get_clk_div_nominal( i, divmult_adc[i] );
				divclk_time[i] = get_clk_div_time( clock_divide_amount[i] , ping_time[i] );
				pw_time[i] = calc_pw( pw_adc[i] , divclk_time[i] );

				num_divclks_since_ping[i]=0;

				if (clock_divide_amount[i]<=1){ 	//multiplying 
					ready_to_reset[i]=1;
					if (reset_offset_time[i]==0){ //if we're not using reset, we can reduce jitter by immediately resetting the timer on the ping
						cli();tmr_clkout[i]=0;sei();
					}
				}

				//could add a conditional to only do this block if clock_divide_amount[i]>1 ?
				for (j=0;j<19;j++){
					num_pings_since_reset[i][j]++;
					if ( num_pings_since_reset[i][j] >= cda[i][j] ){
						if ( clock_divide_amount[i] == cda[i][j] ) {
							ready_to_reset[i] = 1;
							if (reset_offset_time[i]==0){
								cli();tmr_clkout[i]=0;sei();
							}
						}
						num_pings_since_reset[i][j] = 0;
					}
				}

			} else { //(ping_irq_timestamp)
				if (!FREERUN && (get_tmr_reset(i) > (ping_time[i]<<1))) {
					//incoming clock has stopped
					divclk_time[i]=0;
					reset_offset_time[i]=0;
					num_pings_since_reset[i][i]=0;num_pings_since_reset[i][1]=0;
					num_pings_since_reset[i][2]=0;num_pings_since_reset[i][3]=0;
					num_pings_since_reset[i][4]=0;num_pings_since_reset[i][5]=0;
					num_pings_since_reset[i][6]=0;num_pings_since_reset[i][7]=0;
					num_pings_since_reset[i][8]=0;num_pings_since_reset[i][9]=0;
					num_pings_since_reset[i][10]=0;num_pings_since_reset[i][11]=0;
					num_pings_since_reset[i][12]=0;num_pings_since_reset[i][13]=0;
					num_pings_since_reset[i][14]=0;num_pings_since_reset[i][15]=0;
					num_pings_since_reset[i][16]=0;num_pings_since_reset[i][17]=0;
					num_pings_since_reset[i][18]=0;
				}
			}
		}





		/***************** RESET *******************/

		cli();
		resets[0]=tmr_reset[0];
		resets[1]=tmr_reset[1];
		resets[2]=tmr_reset[2];
		resets[3]=tmr_reset[3];
		sei();

		for (i=0;i<4;i++){
			if (RESET(i)){
				if (!reset_up[i]){
					IMMEDIATE_CLKOUT_OFF(i); //goes off for 10uS
					num_pings_since_reset[i][0]=0;num_pings_since_reset[i][1]=0;
					num_pings_since_reset[i][2]=0;num_pings_since_reset[i][3]=0;
					num_pings_since_reset[i][4]=0;num_pings_since_reset[i][5]=0;
					num_pings_since_reset[i][6]=0;num_pings_since_reset[i][7]=0;
					num_pings_since_reset[i][8]=0;num_pings_since_reset[i][9]=0;
					num_pings_since_reset[i][10]=0;num_pings_since_reset[i][11]=0;
					num_pings_since_reset[i][12]=0;num_pings_since_reset[i][13]=0;
					num_pings_since_reset[i][14]=0;num_pings_since_reset[i][15]=0;
					num_pings_since_reset[i][16]=0;num_pings_since_reset[i][17]=0;
					num_pings_since_reset[i][18]=0;
					
					reset_offset_time[i]=resets[i]; //time elapsed since last ping
					reset_now_flag[i]=1;
					reset_up[i]=1;
					ready_to_reset[i]=0;
				}
			}	else {
				reset_up[i]=0;
			}
		}
		

		if (++cur_chan>=4) cur_chan=0;

		if (reset_offset_time[cur_chan]>0){
			if (clock_divide_amount[cur_chan] > 1){ //dividing
				if (reset_offset_time[cur_chan] > divclk_time[cur_chan]) {
					reset_offset_time[cur_chan] = 0;
				}
			} else { //multiplying
				if (reset_offset_time[cur_chan] > ping_time[cur_chan]) {
					reset_offset_time[cur_chan] = 0;
				}
			}
		}



		/******************* CLK OUT *****************16us */
		cli();
		
		nows[0]=tmr_clkout[0];
		nows[1]=tmr_clkout[1];
		nows[2]=tmr_clkout[2];
		nows[3]=tmr_clkout[3];
		resets[0]=tmr_reset[0];
		resets[1]=tmr_reset[1];
		resets[2]=tmr_reset[2];
		resets[3]=tmr_reset[3];
		sei();
	
		for (i=0;i<4;i++){
			reset_ck[i]=0xFF;
			if (divclk_time[i]){
				if (nows[i]>=pw_time[i]){
					CLKOUT_OFF(i);
				}

/*
				if ((resets[i] >= reset_offset_time[i]) && ready_to_reset[i]) {
					reset_now_flag[i]=1;
					ready_to_reset[i]=0;
				}
*/
				if ((resets[i] >= reset_offset_time[i] && ready_to_reset[i]) || reset_now_flag[i]){
					reset_now_flag[i]=0;
					ready_to_reset[i]=0;
					reset_ck[i]=0;
					CLKOUT_ON(i);
					num_divclks_since_ping[i]--;
				}

				//do this only if we're using reset, or we're multiplying and this isn't the 0 pulse
				if ((reset_offset_time[i]!=0)|| ((clock_divide_amount[i]<1) && (num_divclks_since_ping[i]>clock_divide_amount[i]))){
					if (nows[i]>=divclk_time[i]){
						t=nows[i]-divclk_time[i];
						reset_ck[i]=(uint8_t)t;
						CLKOUT_ON(i);
						num_divclks_since_ping[i]--;
					}
				} else {
					//if (i==0) DEBUGON;
				}
			} else {
				CLKOUT_OFF(i);
			}
		}
		cli();
		if (reset_ck[0]!=0xFF) {tmr_clkout[0]=reset_ck[0];}
		if (reset_ck[1]!=0xFF) {tmr_clkout[1]=reset_ck[1];}
		if (reset_ck[2]!=0xFF) {tmr_clkout[2]=reset_ck[2];}
		if (reset_ck[3]!=0xFF) {tmr_clkout[3]=reset_ck[3];}
		sei();


		CLKOUT_SETSTATE(clkout_state);

			/************ TAP ************/
			if (TAPIN){
				tapin_down=0;
				now=get_tapintmr();

				if (!(tapin_up)){
					tapin_up=1;

					tapout_clk_time=now;
					reset_tapintmr();
					reset_tapouttmr();

					TAPOUT_ON;
				} else {
					if (now > HOLDTIMECLEAR){ //button has been down for more than 2 seconds
						tapout_clk_time=0;
						reset_tapouttmr();
						TAPOUT_OFF;
	//				} else {
	//					TAPOUT_ON;
					}
				}
			} else {
				tapin_up=0;
				if (!(tapin_down)){
					TAPOUT_OFF;
					tapin_down=1;
				}
			}


			if (tapout_clk_time){

				now=get_tapouttmr();

				if (now>=(tapout_clk_time>>1)){
					TAPOUT_OFF;
				}
				if (now>tapout_clk_time){
					t=now-tapout_clk_time;
					//t=(now-tapout_clk_time)>>8;
					cli();
					tapouttmr=t;
					sei();

					TAPOUT_ON;
				}

			} else {
				TAPOUT_OFF;
			}



		if (cur_chan==3){

			/***************** READ ADC ****************/
			DEBUGOFF;


			if ((++poll_user_input>USER_INPUT_POLL_TIME) && (ADCSRA & (1<<ADIF))){
				DEBUGON;
			
				poll_user_input=0;

				ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"
				adch=ADCH;

				next_adc=cur_adc+1;
				if (next_adc >= 8){
					next_adc=0;	
				}

				ADMUX = (1<<ADLAR) | next_adc;	//Setup for next conversion
				ADCSRA |= (1<<ADSC);			//Start Conversion

				if (cur_adc<4){ //Div/Mult

					if (divmult_adc[cur_adc] > adch) 
						t = divmult_adc[cur_adc] - adch;
					else 
						t = adch - divmult_adc[cur_adc];

					if (t >= DIV_ADC_DRIFT){
						divmult_adc[cur_adc] = adch;
						old_clock_divide_amount[cur_adc] = clock_divide_amount[cur_adc];

						clock_divide_amount[cur_adc] = get_clk_div_nominal(cur_adc, divmult_adc[cur_adc]);
						divclk_time[cur_adc]=get_clk_div_time(clock_divide_amount[cur_adc],ping_time[cur_adc]);
						//if (!output_is_high)
						pw_time[cur_adc]=calc_pw(pw_adc[cur_adc],divclk_time[cur_adc]);

						if (clock_divide_amount[cur_adc]==-16 && old_clock_divide_amount[cur_adc]!=-16)
							reset_offset_time[cur_adc]=0;
					}

				} else { //PW
					i=cur_adc-4;

					if (pw_adc[i]>adch) t=pw_adc[i]-adch;
					else t=adch-pw_adc[i];

					if (t>PW_ADC_DRIFT){
						pw_adc[i]=adch;

						pw_time[i]=calc_pw(pw_adc[i],divclk_time[i]);
					}
					
				}

				cur_adc=next_adc;	

			}
			
		}
	} //main loop

} //void main()






