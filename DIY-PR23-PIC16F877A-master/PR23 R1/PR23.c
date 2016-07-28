//============================================================================================================
//	Author				:CYTRON	
//	Project				:PR23
//	Project description	:Simple line follow
//  Version 			:v1.5
//============================================================================================================

//	include
//============================================================================================================
#include <pic.h> 	//include the hitech header file

//	configuration
//============================================================================================================
__CONFIG ( 0x3F32 );	//configuration bits

//	define
//============================================================================================================
#define sw1			RE0
#define sw2			RE1
#define motor_ra	RC0
#define motor_rb	RC3
#define motor_la	RC4
#define motor_lb	RC5

#define s_left		RB0
#define s_mleft		RB1
#define s_mright	RB2
#define s_right		RB3

#define buzzer 		RE2

#define	rs			RB7
#define	e			RB6
#define	lcd_data	PORTD
#define b_light     RB5
#define SPEEDL		CCPR1L
#define SPEEDR		CCPR2L

#define	CHANNEL0	0b10000001	// AN0 ( Ultrasonic )
#define	CHANNEL1	0b10001001	// AN1 ( Distance sensor )

//skps protocol
#define	p_select		0
#define p_joyl			1
#define p_joyr			2
#define p_start			3
#define p_up			4
#define p_right			5
#define p_down			6
#define p_left			7
#define	p_l2			8
#define	p_r2			9
#define p_l1			10
#define p_r1			11
#define p_triangle		12
#define p_circle		13
#define p_cross			14
#define	p_square		15
#define p_joy_lx		16
#define	p_joy_ly		17
#define p_joy_rx		18
#define p_joy_ry		19
#define p_joy_lu		20
#define p_joy_ld		21
#define p_joy_ll		22
#define p_joy_lr		23
#define p_joy_ru		24
#define p_joy_rd		25
#define p_joy_rl		26
#define p_joy_rr		27

#define	p_con_status	28
#define p_motor1		29
#define p_motor2		30

#define RX_PIN 		RC7
#define TX_PIN 		RA2
#define BOT_ADD		100



//	global variable
//============================================================================================================
unsigned char data[6] = {0};
const unsigned char line [] = {"1.LINE FOLLOW"};
const unsigned char US[] = {"2.Ultrasonic"};
const unsigned char AS[] = {"3.Analog Sensor"};
const unsigned char XB[] = {"4.SKXBEE"};
const unsigned char SKPS[] = {"5.SKPS"};
const unsigned char *mode [5] = {&line[0],&US[0],&AS[0],&XB[0],&SKPS[0]};

unsigned int result;
unsigned int To=0,T=0,TH=0;
unsigned char REC;
unsigned char i=0,raw;

unsigned int us_value (unsigned char mode);

//	function prototype
//============================================================================================================
void init(void);
void delay(unsigned long data);
void send_config(unsigned char data);
void send_char(unsigned char data);
void e_pulse(void);
void lcd_goto(unsigned char data);
void lcd_clr(void);
void send_string(const char *s);
void dis_num(unsigned long data);

void line_follow(void);
void ultrasonic(void);
void wireless_xbee(void);
void analog_sen(void);
void SKPS_PScon(void);

void forward(void);
void stop (void);
void backward (void);
void reverse (void);
void left(void);
void right(void);

void uart_send(unsigned char data);
unsigned char uart_rec(void);
unsigned char skps(unsigned char data);
void skps_vibrate(unsigned char motor, unsigned char value);
void read_adc(char config);


//	interrupt prototype
//============================================================================================================
static void interrupt isr(void)
{
	if (TMR0IF)								// TMR0 is overflow
	{
		TMR0IF = 0; 						// clear flag bit
		To +=0x100;							// count number of TMR0 overflow ( make it to 16bit TMR)
	}
		
	if(RBIF)								// there is change bit on RB4-RB7
	{
		RBIF = 0;							//	                                  ____
		if (RB4)							// Rb4 is 1 mean is rising form 0  __|         
		{
			TMR0 = 0;						// clear all counter involved, start new count for period of RB4 high
			To = 0;
		}
											//									___
		else TH = TMR0 + To;				// RB4 is 0 mean is falling form 1 	   |_____  // save TH, RB4 high period
	}

	if(RCIF)
	{
		RCIF = 0;							// clear flag bit

		if (RCREG == 'R') data[i=0]= RCREG;				// check if start byte 'R' is met 		
		else if (RCREG == 100) data[i=0]= RCREG;		// check if start byte 'd'(decimal 100) is met
		if ((data[0] == 'R'))data [i++] = RCREG;		// save the data in data array
		if (i>4) i = 4;									// if the data array reached max, set the index to 4
	}
} 


//	main function
//============================================================================================================
void main(void)
{

	unsigned char m=0,i =0;
	delay(20000);									
	init();											// initiate cnfiguration and initial condition
	buzzer = 1;										// inditcate the circuit is on with beep
	lcd_clr();										// clear the LCD screen
	send_string("Select mode");						// display "select mode"
	lcd_goto(20);									// move to 2nd line
	send_string(mode[m]);							// display string according to the mode
	buzzer = 0;										// stop beep

	while(1)										// loop
	{
		if( !sw1)									// if button SW1 is pressed
		{
			while(!sw1);							// wait ubtul button is released
			m++;
			if ( m > 4) m = 0;						// if mode is added more than three, set to zero
			lcd_goto(20);							// start display at 20
			send_string(mode[m]);					// display string depend on mode
			send_string("       ");					// space to overwrite long words
		}
		
		if (!sw2)									// if button SW2is pressed
		{
			while(!sw2);							// wait until button is released
			switch(m)								// check what is the current mode, execute the mode
			{

				case 0 :	line_follow();			// mode 1 : line follow
							break;
				case 1 :	ultrasonic();			// mode 2 : ultrasonic mode
							break;
				case 2 :	analog_sen();			// mode 3 : analog sensor mode 
							break;
				case 3 :	wireless_xbee();		// mode 4 : wireless xbee mode
							break;
				case 4 :    SKPS_PScon();			// mode 5 : PS2 Controller Mode
							break;
				default :	;			
			}
		}
	}
}

//===========================================================================================================
// Initailization
// Description : Initialize the microcontroller
//============================================================================================================
void init()
{
	// ADC configuration	
	ADCON1 = 0b10000100;				//set RA0 and RA1 as Analog Input, left justified

	// setup for capture pwm
	RBIE = 1;							// enable interrupt on change of port B

	// motor PWM configuration 
	PR2 = 255;						// set period register 
	T2CON =  	0b00000100;			// 
	CCP1CON =	0b00001100;			// config for RC1 to generate PWM	( for more detail refer datasheet section 'capture/compare/pwm')
	CCP2CON =	0b00001100;			// config for RC2 to generate PWM

	// Tris configuration (input or output)
	TRISA = 0b00000011;				//set RA0 and RA2 pin as input,other as output
	TRISB = 0b00011111;				//set RB0-RB4 pin as input, other as output
	TRISC = 0b10000000;				//set PORTC pin as output
	TRISD = 0b00000000;				//set all PORTD pin as output
	TRISE = 0b00000011;

	// TMR 0 configuation
	T0CS = 0;						
	PSA = 0;						
	PS2 = 1;						// prescale 1:32
	PS1 = 1;						//
	PS0 = 1;						//
	TMR0IE = 1;						// TMR0 Interrupt
	TMR0 = 0;

	//setup UART
	SPBRG = 0x81;						//set baud rate to 9600 for 20Mhz
	BRGH = 1;							//baud rate high speed option
	TXEN = 1;							//enable transmission
	TX9 = 0;
	CREN = 1;							//enable reception
	SPEN = 1;							//enable serial port
	RX9 = 0;
	RCIE = 1;							//enable interrupt on eachdata received

	// enable all unmasked interrupt	
	GIE = 1;
	PEIE = 1;

	// LCD configuration
	send_config(0b00000001);		//clear display at lcd
	send_config(0b00000010);		//Lcd Return to home 
	send_config(0b00000110);		//entry mode-cursor increase 1
	send_config(0b00001100);		//diplay on, cursor off and cursor blink off
	send_config(0b00111000);		//function

	TX_PIN = 1;
	b_light = 0;
	buzzer = 0;
	stop();											
}

//============================================================================================================
// Mode subroutine
//============================================================================================================
// Mode 1 : line follow subroutine
// Description: Program for the robot to follow line
// For more detail about line follow concept please refer PR5
//============================================================================================================
void line_follow()							
{
	unsigned char memory;

	lcd_clr();								// clear lcd screen
	send_string("Position");				// display "position" string


	while(1)
	{
		if ((s_left==1)&&(s_mleft==0)&&(s_mright==0)&&(s_right==0))  				// if only sensor left detected black line
		{	forward();																// motor forward
			SPEEDL = 0;																// left motor speed is 0
			SPEEDR = 255;															// right motor speed is 255(full speed)
			memory = PORTB&0b00001111;												// save current sensor position
			lcd_goto(20);															// lcd go to 2nd line 1st character 
			send_string ("right  ");												// display "right"mean the robot's position is on the right side of the line
		}
		else if ((s_left==1)&&(s_mleft==1)&&(s_mright==0)&&(s_right==0))			// if only sensor left detected black line
		{	forward();																// motor forward
			SPEEDL = 180;															// left motor speed is 180
			SPEEDR = 255;															// right motor speed is 255(full speed)
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_right2");
		}
		else if ((s_left==0)&&(s_mleft==1)&&(s_mright==0)&&(s_right==0))			// if only sensor middle left detected black line
		{	forward();																// motor forward
			SPEEDL = 200;															// left motor speed is 200
			SPEEDR = 255;															// right motor speed is 255(full speed)
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_right1  ");
		}
		else if ((s_left==1)&&(s_mleft==1)&&(s_mright==1)&&(s_right==0))			// if sensor middle left and sensor left detected black line
		{	forward();																// motor forward
			SPEEDL = 200;															// left motor speed is 200
			SPEEDR = 255;															// right motor speed is 255(full speed)
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_right1  ");
		}
		else if ((s_left==0)&&(s_mleft==1)&&(s_mright==1)&&(s_right==0))			// if sensor middle left and sensor middle right detected black line
		{	forward();																// motor forward
			SPEEDL = 255;															// left motor speed is 255(full speed)
			SPEEDR = 255;															// right motor speed is 255(full speed)
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("middle ");
		}
		else if ((s_left==0)&&(s_mleft==0)&&(s_mright==1)&&(s_right==0))			// if only sensor middle right detected black line
		{	forward();																// motor forward
			SPEEDL = 255;															// left motor speed is 255(full speed)
			SPEEDR = 200;															// right motor speed is 200
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_left1   ");
		}
		else if ((s_left==0)&&(s_mleft==1)&&(s_mright==1)&&(s_right==1))			// if sensor middle left, sensor middle right and sensor right detected black line
		{	forward();																// motor forward
			SPEEDL = 255;															// left motor speed is 255(full speed)
			SPEEDR = 200;															// right motor speed is 200
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_left1   ");
		}
		else if ((s_left==0)&&(s_mleft==0)&&(s_mright==1)&&(s_right==1))			// if sensor right and sensor middle right detected black line
		{	forward();																// motor forward
			SPEEDL = 255;															// left motor speed is 255(full speed)
			SPEEDR = 180;															// right motor speed is 180
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("m_left2 ");
		}
		else if ((s_left==0)&&(s_mleft==0)&&(s_mright==0)&&(s_right==1))			// if only sensor right detected black line
		{	forward();																// motor forward
			SPEEDL = 255;															// left motor speed is 255(full speed)
			SPEEDR = 0;																// right motor speed is 0
			memory = PORTB&0b00001111;
			lcd_goto(20);
			send_string ("left   ");
		}
		else if ((s_left==0)&&(s_mleft==0)&&(s_mright==0)&&(s_right==0))			// if all sensor coult not detected black line
		{	forward();																// motor forward
			if ((memory == 0b00000001)||(memory == 0b00000011)||(memory == 0b0000010)||(memory == 0b0000111))
			{
				SPEEDL = 0;															// left motor speed is 0
				SPEEDR = 255;														// right motor speed is 255(full speed)
			}
			else if ((memory == 0b00001000)||(memory == 0b0000100)||(memory == 0b00001100)||(memory == 0b0001110))
			{
				SPEEDL = 255;														// left motor speed is 255(full speed)
				SPEEDR = 0;															// right motor speed is 0
			}
		}
		else if ((s_left==1)&&(s_mleft==1)&&(s_mright==1)&&(s_right==1))			// if all sensor detected black line
		{
			forward();																// motor forward
		}
	}
}
//============================================================================================================
// Mode 2 : Ultrasonic
// Description: Maintain distance measure using ultrasonic between obsacle and robot
// Can choose data acquiring methode between ADC, PWM and UART
//============================================================================================================
void ultrasonic(void)		
{
	int distance;																// variable for distance measuring
	char n=1;																	// index for indicat mode
	lcd_clr();																	// clear lcd
	send_string("Measure mode");												// display string 
	lcd_goto(20);																// lcd goto 2nd line
	send_string("ADC");															// Display string "ADC"

	while(1)																	// loop forever unless sw2 is pressed
	{
		if( !sw1)																// if button s1 is pressed
		{
			while(!sw1);														// wait until sw1 is release
			n++;																// increment n
			lcd_goto(20);														// goto 2nd line
			switch (n)															// check current value of n
			{
				case 2 : 	send_string("PWM");				
							break;												// break out from switch
				case 3 : 	send_string("UART");								
							break;												// break out from switch	
				default:  	send_string("ADC");									// if not 2 or 3, set ti back to 1 and display "ADC"
							n =1;
			}

		}
		
		if (!sw2)																// if button sw2 is pressed
		{
			while(!sw2);														// wait until sw2 is release
			break;																// break out form looping
		}
	}


	lcd_clr();																	// clear the lcd
	send_string("Distance");													// display string "Distance"

	while(1)																	// loop forever
	{
		lcd_goto(20);															// lcd goto 2nd line
		distance = us_value(n);													// disance variable are equal to value return from subroutine us_value
		dis_num(distance);														// display the value of distance


	 	if (distance> 40)														// check if distance more than 40
		{
			forward();															// then forward with full speed
			SPEEDL = 255;
			SPEEDR = 255;
			buzzer = 0;
		}
		else if (distance> 30)													// check if distance more than 40
		{
			forward();															// forward with medium speed
			SPEEDL = 230;
			SPEEDR = 230;
			buzzer = 0;
		}
		else if( distance >20)													// check if distance more than 40
		{
			stop();																// then stop
			buzzer = 0;
		}
		else 																	// else, distance less than 20
		{
			backward();															// then backward with medium speed and on the buzzer
			SPEEDL = 230;
			SPEEDR = 230;
			buzzer = 1;
		}
	}

}

//============================================================================================================
// Mode 3 : Analog Distance Sensor
// Description : Maintain distance between robot and obstacle
//============================================================================================================
void analog_sen(void)
{
	int distance;																// variable for distance measuring
	lcd_clr();																	// clear lcd
	send_string("Distance");													// display string 

	while(1)
	{
		lcd_goto(20);														// lcd goto 2nd line
		read_adc(CHANNEL1);													// read adc channel 1 ( analog distance sensor input) 
		distance = result;													// assign distance as the result oft he reading
		dis_num(result);													// display the value	

		if (distance< 200)													// check if distance less than 200
		{
			backward();														// backward with full speed
			SPEEDL = 255;
			SPEEDR = 255;
			buzzer = 0;
		}
		else if (distance< 250)												// check if distance less than 250
		{
			backward();														// backward with medium speed
			SPEEDL = 230;
			SPEEDR = 230;
			buzzer = 0;
		}
		else if( distance < 300)											// check if distance less than 300
		{
			stop();															// stop
			buzzer = 0;
		}
		else 																// else, distance more than 300
		{
			forward();														//forward with medium speed and on buzzer
			SPEEDL = 230;
			SPEEDR = 230;
			buzzer = 1;
		}
	}
}

//============================================================================================================
//Mode 4 :  Xbee
// Description : Control the robot using UART ( XBEE or an UART wireless module.
//============================================================================================================
void wireless_xbee (void)		
{
	lcd_clr();																// clear the lcd
	while(1)																// looping forever
	{
		lcd_goto (0);											
		if (data[0] == 100)													// check if UART start byte is met
		{
			send_string("  XBEE CONTROL  ");								// display string
			SPEEDL = 200;													// set the motor speed
			SPEEDR = 200;
			while(1)
			{
				lcd_goto(20);
				if (RCREG == '8')												// if character '8' is detected, the robot move forward
				{
					forward();
					send_string("FORWARD        ");
				}

				else if (RCREG == '2')											// if character '2' is detected, the robot move backward
				{
					backward();
					send_string("BACKWARD        ");
				}
				else if (RCREG == '6')											// if character '6' is detected, the robot turn right
				{
					right();
					send_string("TURN RIGHT      ");
				}

				else if (RCREG == '4')											// if character '4' is detected, the robot turn left
				{
					left();
					send_string("TURN LEFT       ");
				}

				else if (RCREG == '5') 											// if character '5' is detected, then stop the robot
				{
					stop();
					send_string("INVALID COMMAND  ");
				}

				else 															// else then stop the robot
				{
					stop();
					send_string("INVALID COMMAND  ");
				}
			}
		}
		else 	send_string("COMMAND");
	}
}

//==================================================================================================
// Mode 5: SKPS
// Description: Control the robot using PS Controller
//==================================================================================================
void SKPS_PScon()
{
	unsigned char up_v, down_v, left_v, right_v;

	//Disabled Intterupt
	RCIE = 0;
	GIE = 0;
	PEIE = 0;
	TMR0IE = 0;	

	stop();			//stop all motor
	
	lcd_clr();
	lcd_goto(0);
	send_string("Controlling PR23");
	lcd_goto(20);
	send_string("   Using SKPS");
	
	while(1)
	{	
		//test button for horn
		if(skps(p_l1)==0)buzzer=1;			//if button L1 pressed, beep the buzzer
		else buzzer=0;						
		
		//read joy stick value process		
		up_v=skps(p_joy_ru);
		down_v=skps(p_joy_rd);
		left_v=skps(p_joy_ll);
		right_v=skps(p_joy_lr);		
				
		//button control for mobilility
		if(skps(p_up)==0)					//check "up" button
		{
			forward();						//move forward
			SPEEDL=230;
			SPEEDR=230;
		}
		else if(skps(p_down)==0)			//check "down" button
		{	
			backward();						//move backward
			SPEEDL=230;
			SPEEDR=230;
		}
		else if(skps(p_left)==0)			//check "left" button
		{
			left();							//rotate left
			SPEEDL=230;
			SPEEDR=230;
		}
		else if(skps(p_right)==0)			//check "right" button
		{
			right();						//rotate right
			SPEEDL=230;
			SPEEDR=230;
		}
				
		//analog control for mobility	
		else if(up_v>0)
		{
			forward();
			if(left_v>0)
			{
				if(up_v>left_v)SPEEDL=up_v-left_v+140;
				else SPEEDL=140;
				SPEEDR=up_v+140;		
			}
			else if(right_v>0)
			{
				if(up_v>right_v)SPEEDR=up_v-right_v+140;
				else SPEEDR=140;
				SPEEDL=up_v+140;
			}
			else
			{
				SPEEDL=up_v+140;
				SPEEDR=up_v+140;	
			}
		}
		else if(down_v>0)
		{
			backward();
			if(left_v>0)
			{
				if(down_v>left_v)SPEEDR=down_v-left_v+140;
				else SPEEDR=140;
				SPEEDL=down_v+140;		
			}
			else if(right_v>0)
			{
				if(down_v>right_v)SPEEDL=down_v-right_v+140;
				else SPEEDL=140;
				SPEEDR=down_v+140;
			}
			else
			{
				SPEEDL=down_v+140;
				SPEEDR=down_v+140;	
			}
		}
		else if(left_v>0)
		{
			left();
			SPEEDL=left_v+120;
			SPEEDR=left_v+120;	
		}
		else if(right_v>0)
		{
			right();
			SPEEDL=right_v+120;
			SPEEDR=right_v+120;	
		}
		else
		{
			stop();
			SPEEDL=255;
			SPEEDR=255;	
		}
	}	
}

//====================================================================================================
// Ultrasonic value
// Description : Retrive data from Ultrsonic. Can choose methode between ADC, PWM and UART
// Parameter : mode		1) using analog 
//						2) using pwm
//						3) using uart
//====================================================================================================
unsigned int us_value (unsigned char mode)											// subroutine for ultrasonic measurement
{
	unsigned int value;
	switch (mode)																	// retrive value of measured distane based on the methode selected
	{
		case 1:	read_adc(CHANNEL0);
				value = result;														// max vslue 2.55v = 2.55/5 *1024 - 1 =  522, resolution = 10mV/ inch, 10m/5*1024 =~ 2
				break;
		case 2:	value = TH;															// each value = 256*4/20mhz = 51.2us, i inch = 147us  
				break;																// can change using smaller timer prescale, but resulation fixed 147us / inch 
		case 3:	if ( data [0]=='R')	value = (data[1] - 0x30)*100+ (data[2] - 0x30)*10+ (data[3] - 0x30); // 1 = 1 inch
				else
				{	
					lcd_goto(20);													// if stater byte is not 'R', Display 'not connected'
					send_string("not connected");
					while(1);														// loop forever
				}	
		default: ;
	}
	return value;
}	


//===========================================================================================================
// read adc
// Description: subroutine for converting analog value to digital with average 200 samples 
// Parameter : config  ( select the channel ) 
//============================================================================================================
void read_adc(char config)
{
	unsigned short i;
	unsigned long result_temp=0;

	ADCON0 = config;
	delay(10000);					// delay after changing configuration 
	for(i=200;i>0;i-=1)				//looping 200 times for getting average value 
	{
		
#if ((_HTC_VER_MAJOR_ == 9) && (_HTC_VER_MINOR_ <= 80))	//if HITECH v9.80 or below is use
		ADGO = 1;					//ADGO is the bit 2 of the ADCON0 register
		while(ADGO==1);				//ADC start, ADGO=0 after finish ADC progress
#elif ((_HTC_VER_MAJOR_ == 9) && (_HTC_VER_MINOR_ > 80))	//if HITECH v9.81 or above is use
		GO_DONE = 1;					//ADGO is the bit 2 of the ADCON0 register
		while(GO_DONE==1);				//ADC start, ADGO=0 after finish ADC progress
#endif
		result=ADRESH;
		result=result<<8;			//shift to left for 8 bit
		result=result|ADRESL;		//10 bit result from ADC

		result_temp+=result;		
	}
	result = result_temp/200;		//getting average value
	ADON = 0;						//adc module is shut off
}


//===========================================================================================================
// Motor control function
// Description : subroutine to set the robot moving direction
//============================================================================================================
void forward ()
{
	motor_ra = 0;
	motor_rb = 1;
	motor_la = 0;
	motor_lb = 1;
}

void backward ()
{
	motor_ra = 1;
	motor_rb = 0;
	motor_la = 1;
	motor_lb = 0;
}

void left()
{
	motor_la = 1;
	motor_lb = 0;
	motor_ra = 0;
	motor_rb = 1;
}
void right()
{
	motor_la = 0;
	motor_lb = 1;
	motor_ra = 1;
	motor_rb = 0;
}

void stop()
{
	motor_la = 1;
	motor_lb = 1;
	motor_ra = 1;
	motor_rb = 1;
}

//===========================================================================================================
//	LCD	functions
//============================================================================================================
void delay(unsigned long data)					//delay function, the delay time
{
	for( ;data>0;data-=1);						//depend on the given value
}

void send_config(unsigned char data)			//send lcd configuration 
{
	rs=0;										//set lcd to config mode 
	lcd_data=data;								//lcd data port = data
	delay(400);								
	e_pulse();									//pulse e to confirm the data
}

void send_char(unsigned char data)				//send lcd character
{
	rs=1;										//set lcd to display mode
	lcd_data=data;								//lcd data port = data				 
	delay(400);
	e_pulse();									//pulse e to confirm the data
}

void e_pulse(void)								//pulse e to confirm the data
{
	e=1;
	delay(300);
	e=0;
	delay(300);
}

void lcd_goto(unsigned char data)				//set the location of the lcd cursor
{
 	if(data<16)									//if the given value is (0-15) the 
	{											//cursor will be at the upper line
	 	send_config(0x80+data);
	}
	else										//if the given value is (20-35) the 
	{											//cursor will be at the lower line
	 	data=data-20;							//location of the lcd cursor(2X16):
		send_config(0xc0+data);					// -----------------------------------------------------
	}											// | |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15| |
}												// | |20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35| |
												// -----------------------------------------------------	

void lcd_clr(void)								//clear the lcd
{
 	send_config(0x01);
	delay(350);	
}

void send_string(const char *s) 				//send a string to display in the lcd
{          
  	while (s && *s)send_char (*s++);
}

void dis_num(unsigned long data)
{
	unsigned char hundred_thousand;
	unsigned char ten_thousand;
	unsigned char thousand;
	unsigned char hundred;
	unsigned char tenth;

	hundred_thousand = data/100000;						// devide to get the numerator 		eg: 5234/1000 = 5
	data = data % 100000;								// modulas to get the remainder 	eg: 5234%1000 = 234
	ten_thousand = data/10000;
	data = data % 10000;
	thousand = data / 1000;
	data = data % 1000;
	hundred = data / 100;
	data = data % 100;
	tenth = data / 10;
	data = data % 10;

	send_char(hundred_thousand + 0x30);					//0x30 added to become ASCII code char '0' to '9'
	send_char(ten_thousand + 0x30);
	send_char(thousand + 0x30);
	send_char(hundred + 0x30);
	send_char(tenth + 0x30);
	send_char(data + 0x30);
}

//====================================================================================================
// uart function
//====================================================================================================
void uart_send(unsigned char data)	//function to send out a byte via uart
{	
	while(TXIF==0);					//wait for previous data to finish send out
	TXREG=data;						//send new data
}

unsigned char uart_rec(void)		//function to wait for a byte receive from uart
{
	unsigned char temp;
	while(RCIF==0);					//wait for data to received
	temp=RCREG;					
	return temp;					//return the received data
}

//==================================================================================================
// skps function
//==================================================================================================
unsigned char skps(unsigned char data)			//function to read button and joystick 
{												//information on ps controller
	uart_send(data);
	return uart_rec();
}

void skps_vibrate(unsigned char motor, unsigned char value)
{												//function to control the vibrator motor 
	uart_send(motor);							//on ps controller
	uart_send(value);	
}
