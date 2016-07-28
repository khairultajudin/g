//============================================================================================================
//	Author			:CYTRON	Technologies
//	Project			:PR23 Rev2.0
//	Project description	:DIY Project 23, Multifunction Mobile Robot
//      Version 		:v2.2
//      IDE                     :MPLAB X IDE v1.70
//      Compiler                :HI-TECH PICC v9.80 or v9,83 or X8 Compiler v1.11 (default XC)
//      Date                    :18 Mar 2013
//      Example code is provided as "it is", Cytron Technologies do not take responsibility to
//      verify, improve or explain the working of the code.
//      If you have any inquiry, welcome to discuss in our techcanil forum:
//      http://forum.cytron.com.my
//============================================================================================================

//	include library files
//============================================================================================================
#if (__XC8)
#include <xc.h>    //header file for hitech mid-range pic
#elif (HI_TECH_C)
#include <htc.h>    //header file for hitech mid-range pic
#endif
//	configuration
//============================================================================================================
#if (__XC)  //if XC Compiler is use to compile this code
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#elif (HI_TECH_C) //if HI-TECH C Compiler is use to compile this code
__CONFIG ( 0x3F32 );    //configuration bits value for
                        //High Speed oscillato
                        //Watchdog timer disable
                        //Power up Timer enable
                        //Low voltage programming disable

#endif
//	define labels or constants
//============================================================================================================
#define _XTAL_FREQ      20000000 //Frequancy of crystal oscillator, for delay, 20MHz
#define SW1		RE0     //SW1 push button is connected to RE0 of PIC
#define SW2		RE1     //SW2 push button is connected to RE1 of PIC
#define IR_L            RA4     //IR01A medium range sensor for left side is connected to RA4 of PIC
#define IR_R            RA5     //IR01A medium range sensor for right side is connected to RA5 of PIC
#define MOTOR_R1	RC0     //Control pin for motor, going through motor driver L293D, right motor
#define MOTOR_R2	RC3     //Control pin for motor, going through motor driver L293D, right motor
#define MOTOR_L1	RC4     //Control pin for motor, going through motor driver L293D, left motor
#define MOTOR_L2	RC5     //Control pin for motor, going through motor driver L293D, left motor

#define SEN_L		RB0     //Line Sensor, Left. Going through comparator. Dark high
#define SEN_ML		RB1     //Line Sensor, Middle Left. Going through comparator. Dark high
#define SEN_MR  	RB2     //Line Sensor, Middle Right. Going through comparator. Dark high
#define SEN_R		RB3     //Line Sensor, Right. Going through comparator. Dark high

#define BUZZER 		RE2     //BUZZER is connected to RE2 of PIC, active high. Is actually being share with
                                //LED
//2x16 character LCD
#define	LCD_RS		RB7     //2x16 parallel LCD RS pin is connected to RB7 of PIC
#define	LCD_E   	RB6     //2x16 parallel LCD E pin is connected to RB6 of PIC
#define	LCD_DATA	PORTD   //2x16 parallel LCD Data pins are connected to PORTD of PIC
#define LCD_BLIGHT      RB5     //2x16 parallel LCD back light is connected to RB5 of PIC, active high
#define LINE1           0
#define LINE2           1

#define SPEEDL		CCPR1L  //PWM register for left motor, to control speed
#define SPEEDR		CCPR2L  //PWM register for right motor, to control speed

//label for Analog channel
#define	CH0         0       // AN0 ( Ultrasonic LVEZ1 )
#define	CH1         1       // AN1 ( Sharp Infrared Distance Sensor )

//skps constant
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

#define	p_con_status            28
#define p_motor1		29
#define p_motor2		30

// Global Variables
//============================================================================================================
unsigned char data[6] = {0};    //general purpose array
const char diy_project [] = " Do It Yourself";
const char multifunction_robot [] = "Multifunction MR"; //Multifunction Mobile Robot
const char pr23_rev [] = "  PR23 Rev2.0";
const char cytron_tech [] = "  Cytron Tech";
const char cytron_website [] = " cytron.com.my";
const char code_version [] = "Sample Code v2.2";
const char line [] = "1.Line Following";
const char US[] = "2.Ultrasonic EZ1";
const char AS[] = "3.Analog Sensor";
const char XB[] = "4.SKXBEE";
const char SKPS[] = "5.SKPS";
const char *mode_string [5] = {&line[0],&US[0],&AS[0],&XB[0],&SKPS[0]};   //array of pointer to strings

unsigned int pulse_width = 0;   //variable to store pulse width value from timer 1

//	function prototype
//============================================================================================================
void init(void);
void delay(unsigned long data);
void delay_ms(unsigned long data);
void beep(unsigned char count);

//LCD function prototype
void lcd_init(void);    //initialize LCD
void lcd_config(unsigned char data);    //lcd send config/command data
void lcd_char(unsigned char data);  //lcd display single ASCII character
void e_pulse(void); //generate E pulse for lcd to read and process data
void lcd_goto(unsigned char data);  //lcd, move cursor to lcd box, please refer to lcd address
void lcd_home(void);    //lcd, move cursor to home (1st line, 1st column)
void lcd_2ndline(void); //lcd, move cursor to 2nd line
void lcd_clr(void); //clear lcd
void lcd_clr_line(unsigned char line);    //clear single line on LCD
void lcd_string(const char* s); //lcd display string
void lcd_dis_num(unsigned char num_digit, unsigned int value);   //lcd display number in decimal value

// modes
void line_follow(void); //function for line following
void ultrasonic(void);  //function for LVEZ1 ultrasonic range finder
void analog_sen(void);  //function for SHARP infrared distance sensor
void wireless_xbee(void);   //function for SKXBee wireless control
void SKPS_PScon(void);  //function for PS2 control with SKPS

//mobile robot navigation control function
void forward(void);
void stop (void);
void backward (void);
void reverse (void);
void left(void);
void right(void);

//UART functions
void uart_init(void);   //initialize UART module, 9600 baud
void uart_send(unsigned char data); //send data out via UART
void uart_string(const char* string);   //send string out via UART
void uart_string_nl(const char* string);    //send string out via UART + newline at the end
unsigned char uart_rec(void);   //uart receive, 1 byte
unsigned char skps(unsigned char data); //transmit command out to SKPS using UART and wait for response
void skps_vibrate(unsigned char motor, unsigned char value);    //

//ADC functions
void adc_init(void);    //initialize ADC module
unsigned int read_adc(unsigned char channel);    //read adc value and return

//timer1 functions
void timer1_init(void); //timer1 initialization
void interrupt_init(void);  //interrupt initialization
void enable_global_int(void);   //enable global interrupt
void disable_global_int(void);  //disable global interrupt
unsigned int us_value (unsigned char mode); //function to read ultrasonic value from different input method
                                            //ADC, PWM or UART(ASCII)
//	interrupt service routine defination
//============================================================================================================
void interrupt isr(void)
{
    static unsigned char i;
    unsigned char receive_data = 0;
    //RBIF set due to changes on RB4-RB7 pin, since only RB4 is input pin, RB4 changes
    //To measure the pulse width of ultrasonic LVEZ1 PWM output
    if(RBIF)
    {
                                                     //       ____
        if (RB4 == 1)	// RB4 is 1 mean is rising form 0  __|
        {
            TMR1H = 0;	// clear timer 1 high byte
            TMR1L = 0;  // clear timer 1 low byte
            TMR1ON = 1;  // active timer 1            
        }
        else
        {
            TMR1ON = 0; //deactive timer 1
            pulse_width = TMR1H;	// RB4 is 0 mean is falling edge, save the timer 1 register
            pulse_width = (pulse_width << 8) + TMR1L; //combine the High byte and low byte of timer1
        }                                                       // ____
                                                                //     |_____  //
        RBIF = 0;	//reset the interrupt flag
    }

    if(RCIF)    //for ultrasonic LVEZ1 UART output.
    {       
        if(OERR == 1)   //in case there is over run error
        {            
            CREN = 0;   //Reset the Receive engine
            CREN = 1;
            if(RCREG);  //clear the RCREG
        }
        else
        {            
            receive_data = RCREG;   //store the received data
            if (receive_data == 'R') data[i=0] = receive_data;// check if start byte of ASCII 'R', store 'R' to 1st byte
            else if (data[0] == 'R') data [++i] = receive_data;	// save the data in data array            
        }        
    }
} 


//	main function
//============================================================================================================
void main(void)
{
    unsigned char mode = 0, i = 0;
    init();     // initiate cnfiguration and initial condition
    lcd_init(); //initialize LCD
    beep(2);    // inditcate the program is running
    LCD_BLIGHT = 1;    //activate the LCD backlight
    lcd_clr();  // clear the LCD screen
    lcd_string(cytron_tech);
    lcd_2ndline();
    lcd_string(cytron_website);
    delay_ms(800);

    lcd_clr();  // clear the LCD screen
    lcd_string(diy_project);
    lcd_2ndline();
    lcd_string(multifunction_robot);
    delay_ms(800);

    lcd_clr();  // clear the LCD screen
    lcd_string(pr23_rev);
    lcd_2ndline();
    lcd_string(code_version);
    delay_ms(800);

    lcd_clr();  // clear the LCD screen
    lcd_string(mode_string[mode]);	// display string according to the mode
    lcd_2ndline();			// move to 2nd line
    lcd_string("SW1++  SW2->Run");	// display "select mode"
    beep(1);

    while(1)	// infinite loop
    {
        if(SW1 == 0)	// if button SW1 is pressed
        {
            while(SW1 == 0);	// wait for SW1 to be released
            beep(1);
            mode ++; //increase mode value
            if ( mode > 4) mode = 0;	// if mode increased is more than 4, reset to zero
            lcd_clr_line(LINE1);    //clear 1st line of LCD
            lcd_home();		// move LCD cursor back to home
            lcd_string(mode_string[mode]);	// display string base on mode selected            
        }//if(SW1 == 0)

        if (SW2 == 0)	// if button SW2 is pressed
        {
            while(SW2 == 0);	// wait until button is released
            beep(1);

            switch(mode)	// check what is the current mode, execute the mode
            {
                case 0 :
                line_follow();	// mode 1 : line follow
                break;

                case 1 :
                ultrasonic();	// mode 2 : ultrasonic mode
                break;

                case 2 :
                analog_sen();	// mode 3 : analog sensor mode
                break;

                case 3 :
                wireless_xbee();// mode 4 : wireless xbee mode
                break;

                case 4 :
                SKPS_PScon();	// mode 5 : PS2 Controller Mode
            }//switch case
        }//if(SW2 == 0)
    }//while(1)
}//main()

/***********************************************************************************************/
//Delay function defination
void delay(unsigned long data)	//delay function, the delay time
{
    for( ;data>0;data-=1);	//depend on the given value
}

//delay in millisecond
void delay_ms(unsigned long data)	//delay function, the delay time
{
    while(data -- > 0)	//depend on the given value
    __delay_ms(1);
}

void beep(unsigned char count)  //to sound buzzer
{
    while(count-- > 0)
    {
        BUZZER = 1;
        delay_ms(45);
        BUZZER = 0;
        delay_ms(30);
    }
}
//===========================================================================================================
//	LCD functions Definations
//============================================================================================================
void lcd_init(void)
{
    LCD_E = 1;
    delay_ms(15);           //delay 15ms for LCD to get ready
    lcd_config(0b00111000);	//8-bit interface
    lcd_config(0b00000110);	//entry mode-cursor increase 1
    lcd_config(0b00001100);	//diplay on, cursor off and cursor blink off
    lcd_config(0b00000001);	//clear display at lcd
    delay_ms(1);
}
void lcd_config(unsigned char data)	//send lcd configuration
{
    LCD_RS = 0;			//set lcd to config mode
    LCD_DATA = data;		//lcd data port = data
    delay_ms(1);
    e_pulse();		//pulse e to confirm the data
}

void lcd_char(unsigned char data)	//send lcd character
{
    LCD_RS = 1;			//set lcd to display mode
    LCD_DATA = data;		//lcd data port = data
    delay_ms(1);
    e_pulse();		//pulse e to confirm the data
}

void e_pulse(void)		//pulse e to confirm the data
{
    LCD_E = 1;
    delay_ms(1);
    LCD_E = 0;
    delay_ms(1);
}

void lcd_goto(unsigned char data)//set the location of the lcd cursor
{   
    lcd_config(0x80 + data);
    //address for LCD, in hexadecimal value
    // -----------------------------------------------------
    // | |00|01|02|03|04|05|06|07|08|09|0A|0B|0C|0D|0E|0F| |
    // | |40|41|42|43|44|45|46|47|48|49|4A|4B|4C|4D|4E|4F| |
    // -----------------------------------------------------
}
void lcd_home(void)    //lcd, move cursor to home (1st line, 1st column)
{
    lcd_config(0x02);
}

void lcd_2ndline(void) //lcd, move cursor to 2nd line
{
    lcd_config(0x80 + 0x40);
}

void lcd_clr(void)	//clear the lcd
{
    lcd_config(0x01);
    delay_ms(1);
}

void lcd_clr_line(unsigned char line)    //clear single row on LCD
{
    unsigned char i = 0;
    if(line == LINE1)lcd_home();    //move cursor to 1st line, home
    else if(line == LINE2)lcd_2ndline(); //move cursor to 2nd line

    for(i = 16; i > 0; i--) lcd_char(' ');  //display 16 x 'space' to clear single line on LCD
}
void lcd_string(const char* s) 		//send a string to display in the lcd
{
    while (s && *s)lcd_char (*s++);
}

void lcd_dis_num(unsigned char num_digit, unsigned int value)
{
    unsigned char digit[5] = {0};   //array to store digit value
    unsigned char i = 0, j = 0, non_zero = 0;
    unsigned int base = 10000;
    //loop to obtain 5 single digit from int value
    for(j = 4; j > 0; j--)
    {
        digit[j] = value / base;
        if(j == 1)
        {
            digit[0] = value % 10;
            continue;
        }
        value = value % base;
        base = base / 10;
    }

    //display the value on to LCD
    if(num_digit > 5) num_digit = 5;
    for(i = num_digit; i > 0; i--)
    {
        if(i == 1)
        {
            lcd_char(digit[i-1]+0x30);           
        }
        else
        {
            if((digit[i-1] == 0) && (non_zero == 0))lcd_char(' ');  //if zero display blank
            else 
            {
                lcd_char(digit[i-1]+0x30);
                non_zero ++;
            }
        }
    }//for(i = num_digit; i > 0; i--)
    
}

//====================================================================================================
// uart function
//====================================================================================================
void uart_init(void)
{
     //setup UART
    SPBRG = 0x81;	//set baud rate to 9600 at 20Mhz
    BRGH = 1;	//baud rate high speed option
    TXEN = 1;	//enable transmission
    TX9 = 0;
    CREN = 1;	//enable continuous reception
    SPEN = 1;	//enable serial port
    RX9 = 0;
    if(OERR == 1)   //check if there is overrun error.
    {
        CREN = 0;
        CREN = 1;   //reset the OERR
    }
    if(RCREG);
    if(RCREG);
    RCIE = 0;	//disable receiver interrupt initially
}

//function to send out a byte via uart
void uart_send(unsigned char data)	
{
    while(TXIF == 0);			//wait for previous data to finish send out
    TXREG = data;			//send new data
}

//function to send string to UART
void uart_string(const char* string)
{
  while (*string != '\0')
  {
      uart_send(*string++);
      //string++; //point to next character
  }
}

//function to send string to uart with new line at the end
void uart_string_nl(const char* string)
{
    uart_string(string);
    uart_send('\r');    //return home
    uart_send('\n');    //new line
}

//function to wait for a data from UART
unsigned char uart_rec(void)		//function to wait for a byte receive from uart
{
    if(OERR == 1)   //check if there is overrun error.
    {
        CREN = 0;
        CREN = 1;   //reset the OERR
        if(RCREG);  //simple read of receive buffer to clear it
        if(RCREG);
    }
    while(RCIF==0);		//wait for data to received
    return RCREG;		//return the received data
}

//==================================================================================================
// skps functions
//==================================================================================================
// send a command value to SKPS and wait for a byte of response from SKPS
unsigned char skps(unsigned char data)	//function to read button and joystick
{					//information on ps controller
    uart_send(data);
    return uart_rec();
}

//send 2 byte of value to SKPS to control the vibration motor of PS2
void skps_vibrate(unsigned char motor, unsigned char value) //function to control the vibrator motor
{
    uart_send(motor);		//on ps controller
    uart_send(value);
}

// ADC functions definifation
void adc_init(void)//initialize ADC module
{
    // ADC configuration
    ADCON0 = 0b10000000;       //conversion clock Fosc/32, channel 0, ADC is off
    ADCON1 = 0b10000100;//Configure RA0, RA1 and RA3 as Analog Input, right justified
                        //The rest of AN pin is digital pin
}
//===========================================================================================================
// read adc
// Description: subroutine for converting analog value to digital with average 200 samples
// Parameter : config  ( select the channel )
//============================================================================================================
unsigned int read_adc(unsigned char channel)
{
    unsigned char i;
    unsigned int temp_result = 0;
    unsigned long result = 0;
    if(channel == CH0) ADCON0 = 0b10000001;
    else if(channel == CH1) ADCON0 = 0b10001001;

    delay_ms(1);			// delay after changing configuration
    for(i = 200; i > 0; i--)		//looping 50 times for getting average value
    {
#if ((_HTC_VER_MAJOR_ == 9) && (_HTC_VER_MINOR_ <= 80)) //if HI-TECH v9.80 or below is used
        ADGO = 1;			//Set ADGO to start ADC conversion
        while(ADGO==1);		//wait for ADGO become 0, means conversion completed
#elif ((_HTC_VER_MAJOR_ == 9) && (_HTC_VER_MINOR_ > 80) || (__XC))  //if HI-TECH v9.81 and above or XC compiler is used
         GO_DONE = 1;			//Set ADGO to start ADC conversion
        while(GO_DONE==1);		//wait for ADGO become 0, means conversion completed
#endif
        temp_result = ADRESH;              //get the high byte of result
        temp_result = temp_result<<8;		//shift to left for 8 bit
        temp_result = temp_result|ADRESL;	//10 bit result from ADC

        result = result + temp_result;
    }
    result = result/200;		//getting average value
    ADON = 0;			//adc module is shut off
    return (unsigned int)result;        //return the result after averange
}


//initialize timer1
void timer1_init(void)
{
    TMR1H = 0;  //clear the timer 1 high byte value
    TMR1L = 0;  //clear the timer 1 low byte value
    T1CON = 0b00100001; //prescaler of 1:4, internal clock source, timer 1 off
}

//function to initialize interrupt, basically to disable the interrupt used.
void interrupt_init(void)
{
    RCIE = 0;   //disable UART receive interrupt
    RBIE = 0;   //disable PORT B on change interrupt
    TMR1IE = 0; //disable Timer 1 overflow interrupt
}

//function to enable global and peripherral interrupt bits
void enable_global_int(void)
{
    GIE = 1;    //enable global interrupt
    PEIE = 1;   //enable peripheral interrupt
}

//function to disable global and peripheral interrupt bits
void disable_global_int(void)
{
    GIE = 0;    //disable global interrupt
    PEIE = 0;   //disable peripheral interrupt
}

//===========================================================================================================
// Initailization
// Description : Initialize the microcontroller
//============================================================================================================
void init()
{
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    // Tris configuration (input or output)
    TRISA = 0b00110011;		//set RA0 and RA2 pin as input,other as output
                                //PR23 Rev2.0 has RA4 and RA5 as input for IR01A
    TRISB = 0b00011111;		//set RB0-RB4 pin as input, other as output
    TRISC = 0b10000000;		//set PORTC pin as output, RC7 is UART Receive pin (input)
    TRISD = 0b00000000;		//set all PORTD pin as output
    TRISE = 0b00000011;         //RE0 and RE1 as input (Switches) RE2 as output (LED)

    // initialize ADC module
    adc_init();
    interrupt_init();   //initialize interrupt

    // motor PWM configuration
    PR2 = 255;			// set period register for PWM
    T2CON =  	0b00000100;	// Timer Control register, timer 2 ON, prescaler = 1:1
    CCP1CON =	0b00001100;	// config for RC1 to generate PWM( for more detail refer datasheet section 'capture/compare/pwm')
    CCP2CON =	0b00001100;	// config for RC2 to generate PWM
    SPEEDL = 0;     //initial PWM is zero
    SPEEDR = 0;     //initial PWM is zero

    disable_global_int(); //disable global interrupt
    lcd_init();    //initialize LCD
    stop();	//motors are off
}

//============================================================================================================
// Mode subroutine
//============================================================================================================
// Mode 1 : line follow subroutine
// Description: Program for the mobile robot to follow line
// For more details about line follow concept please PR23 Detailed Description
//============================================================================================================
void line_follow()							
{
    unsigned char memory = 0;   //variable to memorize previous condition if sensor is out of line

    lcd_clr();                  // clear lcd screen
    lcd_string(" Line Position");    // display "position" string

    //When sensor senses line (black) it will get logic 1 (5V or HIGH) at PIC pin.
    while(1)    //infinite loop
    {
        forward(); //mobile robot will move forward
        if ((SEN_L==1)&&(SEN_ML==0)&&(SEN_MR==0)&&(SEN_R==0)) 	// if only sensor left detected black line
        {            
            SPEEDL = 0;     //left motor stop
            SPEEDR = 255;   // right motor speed is 255(full speed)
            memory = 1;     //1 = line is at left of mobile robot
            lcd_2ndline();		// lcd go to 2nd line 1st character
            lcd_string ("right  ");	// display "right"mean the robot's position is on the right side of the line
        }
        else if ((SEN_L==1)&&(SEN_ML==1)&&(SEN_MR==0)&&(SEN_R==0))	// if only sensor left detected black line
        {            
            SPEEDL = 180;   // left motor speed is 180
            SPEEDR = 255;   // right motor speed is 255(full speed)
            memory = 1;     //1 = line is at left of mobile robot
            lcd_2ndline();
            lcd_string ("m_right2");
        }
        else if ((SEN_L==0)&&(SEN_ML==1)&&(SEN_MR==0)&&(SEN_R==0))	// if only sensor middle left detected black line
        {            
            SPEEDL = 200;   // left motor speed is 200
            SPEEDR = 255;   // right motor speed is 255(full speed)
            memory = 1;     //1 = line is at left of mobile robot
            lcd_2ndline();
            lcd_string ("m_right1  ");
        }
        else if ((SEN_L==1)&&(SEN_ML==1)&&(SEN_MR==1)&&(SEN_R==0)) // if sensor middle left, middle right
                                                                   //and sensor left detected black line
        {            
            SPEEDL = 200;   // left motor speed is 200
            SPEEDR = 255;   // right motor speed is 255(full speed)
            memory = 1;     //1 = line is at left of mobile robot
            lcd_2ndline();
            lcd_string ("m_right1  ");
        }
        else if ((SEN_L==0)&&(SEN_ML==1)&&(SEN_MR==1)&&(SEN_R==0))	// if sensor middle left and sensor middle right detected black line
        {            
            SPEEDL = 255;   // left motor speed is 255(full speed)
            SPEEDR = 255;   // right motor speed is 255(full speed)
            memory = 2;     //32 = line is at middle of mobile robot
            lcd_2ndline();
            lcd_string ("middle ");
        }
        else if ((SEN_L==0)&&(SEN_ML==0)&&(SEN_MR==1)&&(SEN_R==0))	// if only sensor middle right detected black line
        {            
            SPEEDL = 255;   // left motor speed is 255(full speed)
            SPEEDR = 200;   // right motor speed is 200
            memory = 3;     //3 = line is at right of mobile robot
            lcd_2ndline();
            lcd_string ("m_left1   ");
        }
        else if ((SEN_L==0)&&(SEN_ML==1)&&(SEN_MR==1)&&(SEN_R==1))    // if sensor middle left, sensor middle right and sensor right detected black line
        {            
            SPEEDL = 255;   // left motor speed is 255(full speed)
            SPEEDR = 200;   // right motor speed is 200
            memory = 3;     //3 = line is at right of mobile robot
            lcd_2ndline();
            lcd_string ("m_left1   ");
        }
        else if ((SEN_L==0)&&(SEN_ML==0)&&(SEN_MR==1)&&(SEN_R==1))	// if sensor right and sensor middle right detected black line
        {            
            SPEEDL = 255;   // left motor speed is 255(full speed)
            SPEEDR = 180;   // right motor speed is 180
            memory = 3;     //3 = line is at right of mobile robot
            lcd_2ndline();
            lcd_string ("m_left2 ");
        }
        else if ((SEN_L==0)&&(SEN_ML==0)&&(SEN_MR==0)&&(SEN_R==1))	// if only sensor right detected black line
        {            
            SPEEDL = 255;   // left motor speed is 255(full speed)
            SPEEDR = 0;     // right motor speed is 0
            memory = 3;     //3 = line is at right of mobile robot
            lcd_2ndline();
            lcd_string ("left   ");
        }
        else if ((SEN_L==0)&&(SEN_ML==0)&&(SEN_MR==0)&&(SEN_R==0))	// if all sensor coult not detected black line
        {            
            if (memory == 1 )
            {
                SPEEDL = 0;	// left motor speed is 0
                SPEEDR = 255;	// right motor speed is 255(full speed)
            }
            else if (memory == 3)
            {
                SPEEDL = 255;	// left motor speed is 255(full speed)
                SPEEDR = 0;	// right motor speed is 0
            }
        }        
    }//while(1)
}
//============================================================================================================
// Mode 2 : Ultrasonic
// Description: Maintain distance measure using ultrasonic between obsacle and robot
// Can choose data acquiring methode between ADC, PWM and UART
//============================================================================================================
void ultrasonic(void)		
{
    unsigned int distance = 0;	// variable for distance measuring
    char ez1_mode = 1;      //variable to store EZ1 modes 
    lcd_clr();		// clear lcd
    lcd_string("Measure : ");	// display string
    lcd_goto(0x09); // lcd move cursor to address 9
    lcd_string("ADC");
    lcd_2ndline();
    lcd_string("SW1++  SW2->Run");
    //to convert value to distance please refer to LVEZ1 datasheet.
    //ADC, 10mV/inch, minimum is 6 inches
    //PWM, 147us/inch, minimum is 6 inches

    while(SW2 == 1) //while SW2 is not press			
    {
        if(SW1 == 0)//if SW1 is pressed					
        {
            while(SW1==0);  // wait for SW1 to be released
            beep(1);
            ez1_mode++;     // increment the mode for EZ1
            lcd_goto(0x09); // lcd move cursor to address 9														
            switch (ez1_mode)	// check current value of ez1_mode
            {
                case 2:
                lcd_string("PWM ");
                break;	// break out from switch

                case 3:
                lcd_string("UART");
                break;	// break out from switch

                default:
                lcd_string("ADC ");	// if not 2 or 3, set the mode back to 1 and display "ADC"
                ez1_mode = 1;
            }//switch (ez1_mode)
        }//if(SW1 == 0)
    }//while(SW2 == 1)
    while(SW2 == 0);    //wait for SW2 to be release.
    								
    lcd_clr_line(LINE2);    //clear 2ndline of LCD
    lcd_2ndline();
    lcd_string("Range:");   // display string "Range"
    lcd_goto(0x4B);
    lcd_string("inch");
    if(ez1_mode == 2)  //if PWM is selected, enable the global interrupt and RBIE
    {
        timer1_init();  //initialize timer 1 for Pulse width measurement
        if(RB4);    //simple read of RB4
        TMR1ON = 1;
        RBIF = 0;   //clear RBIF
        RBIE = 1; //if PWM is selected, enable the port B change interrupt
        enable_global_int();
    }
    else if(ez1_mode == 3)
    {
        uart_init();    //uart initialization
        RCIE = 1; //if UART is selected, enable the receive interrupt
        enable_global_int();
        delay_ms(60);   //delay to receive 1st package from LVEZ1, 49ms per package.
    }
    else
    {
        RBIE = 0;
        RCIE = 0;
        disable_global_int();
    }

    while(1)	// inifinite loop
    {
        distance = us_value(ez1_mode);	// distance variable are equal to value return from subroutine us_value
        lcd_goto(0x47);	//move cursor to after "Inches:"
        if(distance < 5)
        {
            lcd_string("XXX");
            stop();
            SPEEDL = 0;
            SPEEDR = 0;
            continue;   //skip the rest of statements in while loop
        }
        else lcd_dis_num( 3, distance);	// display the value of distance
       
        
        if (distance > 20)  //if distance detected more than 20 inches													
        {
            forward();															
            SPEEDL = 255;
            SPEEDR = 255;
            BUZZER = 0;
        }
        else if (distance > 12)     //if distance detected more than 12 inches												
        {
            forward();															
            SPEEDL = 180;
            SPEEDR = 180;
            BUZZER = 0;
        }
        else if( distance > 8)     //if distance detected is more than 8 inches											
        {
            stop();
            SPEEDL = 0;
            SPEEDR = 0;																
            BUZZER = 0;
        }
        else                //if it is too near, reverse															
        {
            backward();															
            SPEEDL = 200;
            SPEEDR = 200;
            BUZZER = 1;
        }
    }//while(1)
}

//============================================================================================================
// Mode 3 : Sharp Infrared Analog Distance Sensor
// Description : Maintain distance between robot and obstacle
//============================================================================================================
void analog_sen(void)
{
	unsigned long voltage = 0;
        unsigned char distance = 0;																
	lcd_clr();																	
	lcd_string("Sharp GP2Y0A21");
        lcd_2ndline();
        lcd_string("Range:");
        lcd_goto(0x4A);
        lcd_string("cm");

	while(1)
	{
            lcd_goto(0x47);	// move lcd cursor to after "Range cm:"
            voltage = read_adc(CH1);	// read adc channel 1 ( analog distance sensor input)
            //Although PR23 rev1.0 or rev2.0 hardware can support several type of Sharp infrared distance sensor,
            //this program is meant for model:GP2Y0A21, where the sensing range is 4cm to 80cm
            //Please do check the datasheet of this infrared distance sensor,
            //the analog voltage represent the distance detected, but not in linear graph
            //please refer to one of our tutorial to obtain the explantion about it.
            //http://tutorial.cytron.com.my/2011/08/10/project-7-%E2%80%93-analog-sensor-range-using-infrared-distance-sensor/
            //the linear formula is: y = 20.99x + 0.19
            //where y is the output voltage from sensor, x is the inverse of distance (1/cm)
            //distance = 1/x
            //x = (y - 0.19)/20.99
            //1/x = 20.99/(y - 0.19) = distance in cm
            //and the valid voltage for this linear equation is 0.4V to 2.8V
            //therefore, we will convert the ADC result into voltage and check the validation result
            // voltage = ADC result * 4.8828mV
            voltage = (voltage * 48828)/10000;    //will reserve 3 digit after floating point
            if((voltage > 400) && (voltage < 2800)) //if the voltage within the volid range
            {
                voltage = voltage - 190; //all value shift 3 digit to right, as if multiply with 1000
                distance = 20990 / voltage ;
                lcd_dis_num(2,distance);    //display distance on LCD
                lcd_char(' ');
                
                if (distance > 30)	// if distance more than 30cm
                {
                    backward();	// backward with full speed
                    SPEEDL = 255;
                    SPEEDR = 255;
                    BUZZER = 0;
                }
                 else if (distance > 20)// if distance more than 20 cm
                {
                    backward();	//backward with medium speed
                    SPEEDL = 200;
                    SPEEDR = 200;
                    BUZZER = 0;
                }
                 else if( distance > 15)// if distance more than 15cm
                {
                    stop();		// stop
                    BUZZER = 0;
                }
                 else 	// else, distance is less than 15
                {
                    forward();  //forward with medium speed and on buzzer
                    SPEEDL = 200;
                    SPEEDR = 200;
                    BUZZER = 1;
                }
            }//if((voltage > 400) && (voltage < 2800))
            else
            {
                lcd_string ("Out"); //display message that voltage is out of range
                stop();	// stop
                
            }//else
	}//while(1)
}

//============================================================================================================
//Mode 4 :  Xbee
// Description : Control the robot using UART ( XBEE or an UART wireless module.
// The XBee need to be setup properly, please refer to the User's Manual to setup.
// On computer, any terminal program can be used, HyperTerminal, X-CTU, Arduino Serial Monitor
// please press 'd' on keyboard to start the control
//============================================================================================================
void wireless_xbee (void)		
{
    unsigned char uart_data = 0;
    disable_global_int();   //disable the global interrupt
    uart_init();    //uart initialization
    lcd_clr();	// clear the lcd
    lcd_string(" SKXBee Control");
    lcd_2ndline();
    lcd_string("JP7 to UC");
    delay_ms(1000);
    lcd_clr_line(LINE2);    //clear lcd 2nd line only
    lcd_2ndline();
    lcd_string(" d on computer");

    while(uart_rec() != 'd');   //wait until UART receive a character 'd'
    lcd_clr_line(LINE2);
    lcd_2ndline();
    lcd_string("Ready to control!");
    beep(2);

    uart_string_nl("Congrats!");    //send message to computer
    uart_string_nl("Connected with XBee wireless");
    uart_string_nl("number 8 = forward");
    uart_string_nl("number 2 = backward");
    uart_string_nl("number 6 = turn right");
    uart_string_nl("number 4 = turn left");
    uart_string_nl("Other = stop and invalid command");

    SPEEDL = 200;   // set the motor speed
    SPEEDR = 200;
    
    stop(); //mobile robot stop initially

    while(1)
    {
        uart_data = uart_rec();
        lcd_2ndline();
        if (uart_data == '8')      //if data receive from SKXBee is '8'
        {
            forward();
            lcd_clr_line(LINE2);    //clear 2nd line of LCD only
            lcd_goto(0x44);
            lcd_string("Forward");  //display on LCD
            uart_string_nl("Forward!"); //send string to UART, or the computer via SKXBee
        }
        else if (uart_data == '2')  //if data receive from SKXBee is '2'
        {
            backward();
            lcd_clr_line(LINE2);
            lcd_goto(0x44);
            lcd_string("BACKWARD");
            uart_string_nl("Backward!");
        }
        else if (uart_data == '6')  //if data receive from SKXBee is '6'
        {
            right();
            lcd_clr_line(LINE2);
            lcd_goto(0x43);
            lcd_string("TURN RIGHT");
            uart_string_nl("Turn Right!");
        }
        else if (uart_data == '4')											
        {
            left();
            lcd_clr_line(LINE2);
            lcd_goto(0x43);
            lcd_string("TURN LEFT");
            uart_string_nl("Turn Left!");
        }
        else 										
        {
            stop();
            lcd_clr_line(LINE2);    //clear 2ndline of LCD
            lcd_goto(0x40);
            lcd_string("INVALID COMMAND");  //display string on LCD
            uart_string_nl("Invalid Command!"); //send string to UART
        }
    }//while(1)
}

//==================================================================================================
// Mode 5: SKPS
// Description: Control the robot using PS2 Controller
//==================================================================================================
void SKPS_PScon()
{
    unsigned char up_v, down_v, left_v, right_v;    //variables to store data from SKPS

    //Disabled Intterupt
    disable_global_int();   //disable global interrupt, we will use polling method
    stop();	//stop mobile robot

    lcd_clr();  //clear the LCD
    lcd_string("  Control PR23");
    lcd_2ndline();
    lcd_string("   with SKPS");
    delay_ms(500);
    lcd_clr_line(LINE2);    //clear lcd 2ndline
    lcd_goto(0x43);
    lcd_string("JP7 to UC");
    delay_ms(1000);
    uart_init();    //initialize UART
    beep(2);

    lcd_clr_line(LINE2);
    lcd_goto(0x45);
    lcd_string("Ready!");
    while(1)
    {
        //buttons for activate buzzer
        if((skps(p_l1)== 0) || (skps(p_l2)== 0) || (skps(p_r1)== 0) || (skps(p_r2)== 0))BUZZER = 1;
        else BUZZER = 0;

        //read joy stick value process
        up_v = skps(p_joy_ru);  //read the right analog joystick, up axis
        down_v = skps(p_joy_rd);    //read the right analog joystick, down axis
        left_v = skps(p_joy_ll);    //read the left analog joystick, left axis
        right_v = skps(p_joy_lr);   //read the left analog joystick, right axis

        //button control for mobilility
        if(skps(p_up)==0)	//check "up" button
        {
            forward();		//move forward
            SPEEDL=230;
            SPEEDR=230;
        }
        else if(skps(p_down)==0)	//check "down" button
        {
            backward();			//move backward
            SPEEDL=230;
            SPEEDR=230;
        }
        else if(skps(p_left)==0)	//check "left" button
        {
            left();			//rotate left
            SPEEDL=230;
            SPEEDR=230;
        }
        else if(skps(p_right)==0)	//check "right" button
        {
            right();			//rotate right
            SPEEDL=230;
            SPEEDR=230;
        }

        //analog joystick control for mobility
        else if(up_v > 0)   //if the right joystick is being push up
        {
            forward();  //robot move forward
            if(left_v > 0)  //if left joystick being push to left
            {
                if(up_v > left_v) SPEEDL = up_v - left_v + 140;
                else SPEEDL = 140;
                SPEEDR = up_v + 140;
            }
            else if(right_v > 0)
            {
                if(up_v > right_v) SPEEDR = up_v - right_v + 140;
                else SPEEDR = 140;
                SPEEDL = up_v + 140;
            }
            else
            {
                SPEEDL = up_v + 140;    //add 1ith 140 because the maximum value of axis is 100.
                SPEEDR = up_v + 140;
            }
        }
        else if(down_v > 0) //if the right joystick is being push down
        {
            backward(); //drive the mobile robot backward
            if(left_v > 0)
            {
                if(down_v > left_v) SPEEDR = down_v - left_v + 140;
                else SPEEDR = 140;
                SPEEDL = down_v + 140;
            }
            else if(right_v > 0)
            {
                if(down_v > right_v) SPEEDL = down_v - right_v + 140;
                else SPEEDL = 140;
                SPEEDR = down_v + 140;
            }
            else
            {
                SPEEDL = down_v + 140;
                SPEEDR = down_v + 140;
            }
        }
        else if(left_v > 0)
        {
            left();
            SPEEDL = left_v + 120;
            SPEEDR = left_v + 120;
        }
        else if(right_v > 0)
        {
            right();
            SPEEDL = right_v + 120;
            SPEEDR = right_v + 120;
        }
        else
        {
            stop();
            SPEEDL = 0;
            SPEEDR = 0;
        }
    }//while(1)
}

//====================================================================================================
// Ultrasonic value
// Description : Retrive data from Ultrsonic. Can choose methode between ADC, PWM and UART
// Parameter : mode		1) using analog 
//				2) using pwm
//				3) using uart
//====================================================================================================
unsigned int us_value (unsigned char mode)											
{
    unsigned long value;
    switch (mode)								
    {
        // case 1
        case 1:     //case 1 is to obtain the distance from LVEZ1 through ADC input
        value = read_adc(CH0);  // 10-bit ADC resolution at 5V = 5V/1024 ~ 4.88mV
                                // New datasheet for LVEZ1 stated the analog output is Vcc/512 for 1 inch, which is 9-bit
                                // Vcc/512, if Vcc = 5V, you will get ~ 9.76mV
                                // which is also 4.88mV * 2.
                                // distance in inches = (value * 4.88mV) / (9.76mV)
                                // distance in inches = value / 2
        value = value / 2;
        break;

        // case 2
        case 2:     //case 2 is to obtain the distance from LVEZ1 through PWM, the width length
                    //which being captured and store in pulse_width register by interrupt service routine
        value = pulse_width;    //timer 1 value = 4x4/20MHz = 800ns, each count in timer 1 means 800ns
                                // 1 inch from LVEZ1 = 147us width length, maximum is 254 inches
                                //distance in inches = timer1value*800ns/147us = value*5.442m
        value = (value * 5442) / 1000000;
        break;

        // case 3
        case 3: //case 3 is to obtain the distance using RS232 (PR23 convert it to UART) from LVEZ1, it send string.
                //data is store in array data which being manage by interrupt service routine
        if(data[0] == 'R')  //if the 1st element of the array is 'R', means there is package of data coming in
        {
            value = ((data[1] - 0x30)*100) + ((data[2] - 0x30)*10) + (data[3] - 0x30);
        }
        else    //else the value is 0
        {
            value = 0;
        }
    }//switch (mode)
    return (unsigned int)value;    //cast the value to unsigned int
}	

//===========================================================================================================
// Motor control function
// Description : subroutine to set the robot moving direction
//============================================================================================================
void forward () //function to enable robot to move forward, do not change the speed
{
    MOTOR_R1 = 0;
    MOTOR_R2 = 1;
    MOTOR_L1 = 0;
    MOTOR_L2 = 1;
}

void backward ()    //function to enable robot to move backward, do not change the speed
{
    MOTOR_R1 = 1;
    MOTOR_R2 = 0;
    MOTOR_L1 = 1;
    MOTOR_L2 = 0;
}

void left() //function to enable robot to turn left, do not change the speed
{
    MOTOR_R1 = 0;
    MOTOR_R2 = 1;
    MOTOR_L1 = 1;
    MOTOR_L2 = 0;
}

void right()    //function to enable robot to turn right, do not change the speed
{
    MOTOR_R1 = 1;
    MOTOR_R2 = 0;
    MOTOR_L1 = 0;
    MOTOR_L2 = 1;
}

void stop() //function to enable robot to stop, do not change the speed
{
    MOTOR_R1 = 0;
    MOTOR_R2 = 0;
    MOTOR_L1 = 0;
    MOTOR_L2 = 0;
}
