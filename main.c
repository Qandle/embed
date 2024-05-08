#define F_CPU 8000000L

/* Include Library */
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <xc.h>

/* Define LCD Port */
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR DDRC
#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR DDRD

#define RS PD0
#define E PD1
#define D4 PC0
#define D5 PC1
#define D6 PC2
#define D7 PC3

int n = 0;
char buffer[16];
uint16_t timercount;
uint16_t tempcount;

// 0 - off, 1 - on
int status = 0;

int time = 0; // Count time continuously
int start_time; 
int stop_time;  


void commitData()
{
  LCD_CTRL_PORT |= (1 << E);// Set Enable to high
  _delay_us(1500);
  LCD_CTRL_PORT &= ~(1 << E);// Set Enable to low
  _delay_us(1500);
}

void sendLCDCommand(uint8_t command)
{
  _delay_us(100);
  // pull RS Low for commanding
  LCD_CTRL_PORT &= ~(1 << RS);
  
  // send high nibble (4 bits) of the command
  LCD_DATA_PORT &= 0xF0; // clear 4 bits ?????????? 0
  LCD_DATA_PORT |= command >> 4;
  commitData();
  
  // send low nibble (4 bits) of the command
  LCD_DATA_PORT &= 0xF0; // clear 4 bits ?????????? 0
  LCD_DATA_PORT |= command & 0x0F; // 4 bits ?? ????????????? 0 ???? 4 bits ???????????????????
  commitData();
}

void sendLCDData(uint8_t command)
{
  _delay_us(100);
  // pull RS High for writing
  LCD_CTRL_PORT |= (1 << RS);
  
  // send high nibble (4 bits) of the command
  LCD_DATA_PORT &= 0xF0; // clear 4 bits ??????????? 0
  LCD_DATA_PORT |= command >> 4;
  commitData();
  
  // send low nibble (4 bits) of the command
  LCD_DATA_PORT &= 0xF0; // clear 4 bits ??????????? 0
  LCD_DATA_PORT |= command & 0x0F; // 4 bits ?? ????????????? 0 ???? 4 bits ???????????????????
  commitData();
}

void lcdDisplayString(char* str)
{
  while (*str != '\0')
  {
    sendLCDData(*str);
    str++;
  }
}

void initLCD()
{
	// Set PC0~PC3 as output
  	LCD_DATA_DDR |= 0x0F;
  	// Clear PC0~PC3 as 0
  	LCD_DATA_PORT &= 0xF0; //clear 4 bit ???????? 0
  
  	// Set PD2(RS) and PD4(E) as output
  	LCD_CTRL_DDR |= (1 << RS) | (1 << E);
  	// Clear PD2 and PD4 
  	LCD_CTRL_PORT &= ~(1 << RS) & ~(1 << E);
  
  	sendLCDCommand(0x33);
  	sendLCDCommand(0x32);
  	sendLCDCommand(0x28);
  	sendLCDCommand(0x0E);
  	sendLCDCommand(0x01);
  	sendLCDCommand(0x80);
}

void clearLCD()
{
    sendLCDCommand(0x01); 
    sendLCDCommand(0x80);  
}

// To convert int to char
// char buffer[6];
// itobase10(buffer, adcValue);
char *itobase10(char *buf, uint16_t value)
{
  sprintf(buf, "%u", value);
  return buf;
}

void initInterupt(){
    // the falling edge of interupt INT0 genrate the interupt request
    EICRA |= (1 << ISC00) | (1 << ISC11); 
    
    // Enable external interupt 0 and 1
    EIMSK |= (1 << INT0) | (1 << INT1);
            
    // enable interupt
    sei(); 
}

void initTimer(){
    // Normal mode -> don't have to set anything
    TCCR1A = 0x00;
    
    // Set Prescaler = 1024
  	TCCR1B |= (1 << CS12) | (1 << CS10);
    
    // Enable Timer Interupt
    TIMSK1 |= (1 << TOIE1);
    
    // 1 second
    TCNT1 = 65535 - 7812;    
}

int end_time = 0;
int start_time = 0;

ISR (INT0_vect) 
{
  	// button is free
    if (PIND & (1 << PORTD2)){
        stop_time = time;

        // button is pressed longer than 3 second = Reset to zero
        if ((stop_time - start_time >= 3)){
            status = 0;
          	timercount = 0;
            tempcount = 0;
        }
        
        else{
            if (status == 1){
              timercount = tempcount;
  			}
            else{
              tempcount += 5;
              timercount = tempcount;
            }
            status = 0;
        }
    }
    // button is pressed
    else{
        start_time = time;
    }
}

ISR (INT1_vect) 
{
    status = !status;
}

ISR (TIMER1_OVF_vect){
  PORTD ^= (1 << PORTD4);
  
  clearLCD();
  sprintf(buffer, "%d", timercount);
  lcdDisplayString(buffer);
  if (status && timercount != 0){
  	timercount-=1;
  }
  time++;
  
  TCNT1 = 65535 - 7812;
}

void setup()
{
 
  	// Button Input
  	PORTD |= (1 << PORTD2) |  (1 << PORTD3);
  
  	DDRD |= (1 << DDD4);
  
  	initLCD();
  	initInterupt();
  	initTimer();
}
void loop(){
}

int main(){
    setup();
    
    // Loop
    while(1){
        loop();
    }
}


