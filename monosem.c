
/** 
 * @file monosem.c
 * @author Andreas Fellnhofer
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Copyright 2012, Licensed under GPL v3, see LICENSE file
 *
 * WORK IN PROGRESS
 */
 
 
#define F_CPU   16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include "lcdpcf8574/lcdpcf8574.h"

#define STATE_OFF -2
#define STATE_INTRO -1
#define STATE_RUNNING 0
#define STATE_HA 1
#define STATE_SETUP_TOOTH1 2
#define STATE_SETUP_TOOTH2 3
#define STATE_SWITCHOFF 4

#define STATE_NUMBERS 5

#define E_RAD1 ((uint8_t*)7)
#define E_RAD2 ((uint8_t*)8)

#define PORT_6LEDS PORTB
#define DDR_6LEDS  DDRB

#define PORT_REEDS PORTD
#define DDR_REEDS  DDRD
#define PIN_REEDS  PIND

#define PORT_BTN PORTC
#define DDR_BTN  DDRC
#define PIN_BTN  PINC
#define BTN_PRESSED 0x01
#define BTN_OK_PRESSED 0x02


uint8_t zaehne1[] = { 10, 12, 14 };
uint8_t zaehne2[] = { 10, 11, 13, 17, 19, 21 };
volatile uint8_t rad1 = 0;
volatile uint8_t rad2 = 0;

volatile int8_t state = STATE_INTRO; /*STATE_OFF;*/
volatile uint16_t ticks = -1;
volatile uint32_t turnarounds = 0;
uint16_t scnt = 0;
//volatile uint16_t qua_m=0;
volatile uint8_t _ok = 0;
char s[20] = { 0 };


uint16_t ticks_to_speed(uint16_t ticks)
{
  if(ticks == ((uint16_t) -1)) 
  {
    return 0;
  }    

  return zaehne2[rad2] * 45677 / (((uint32_t)ticks)*zaehne1[rad1]);        // 61t/s * 2.11m * 3.6
}


uint32_t turnarounds_to_squm(uint32_t turnarounds) 
{
  return turnarounds*1456*zaehne2[rad2]/zaehne1[rad1]/1000; // d=0.67 -> 2.08*0.7
}


uint32_t e_read_qua_m_total(void)
{
  uint32_t temp;
  
  temp = ((uint32_t) eeprom_read_word((uint16_t*)0)) & 0xffff;
  temp |= ((uint32_t) eeprom_read_word((uint16_t*)2)) << 16;
  
  return temp;
}


void e_write_qua_m_total(uint32_t qua)
{
  eeprom_write_word ((uint16_t*)0, qua&0xffff);
  eeprom_write_word ((uint16_t*)2, (qua>>16)&0xffff);
}


uint32_t e_read_turnarounds(void)
{
  uint32_t temp;
  
  temp = ((uint32_t) eeprom_read_word((uint16_t*)10)) & 0xffff;
  temp |= ((uint32_t) eeprom_read_word((uint16_t*)12)) << 16;
  
  return temp;
}


void e_write_turnarounds(uint32_t turns)
{
  eeprom_write_word ((uint16_t*)10, turns&0xffff);
  eeprom_write_word ((uint16_t*)12, (turns>>16)&0xffff);
}


void ioinit(void)      
{  
  DDR_6LEDS = 0x3f;
  PORT_6LEDS = 0x20;
  
  DDR_REEDS = 0x00;
  PORT_REEDS = 0xFC;
  
  DDR_BTN = 0x00;
  PORT_BTN = 0x03; /* pullups */

  //TCCR1B = 0x05;
  TCCR0B = 0x05;   // "1 0 1" clkI/O / 1024 (From prescaler)   16MHz / 1024  --> 61 overflows/sek 

  turnarounds = e_read_turnarounds();
  rad1 = eeprom_read_byte(E_RAD1);
  rad2 = eeprom_read_byte(E_RAD2);

  TIMSK0 = 0x01;
  
  sei();
}



void shutdown(void) 
{
  TIMSK0 = 0;

  e_write_turnarounds(turnarounds);
  eeprom_write_byte(E_RAD1, rad1);
  eeprom_write_byte(E_RAD2, rad2);

  lcd_init(LCD_DISP_OFF);
  _delay_ms(100);
  
  DDR_6LEDS = 0x00;
  PORT_6LEDS = 0x00;
}


uint8_t debounce(uint8_t val) 
{
  static uint8_t last = 0;
  static uint8_t cnt;
  
  if((val & last) == val)
  {
    cnt++;
  }
  last = val;
  
  if(cnt >= (6 * (F_CPU/4000000)) ) 
  {
    return val;
  }
  else 
  {
    return 0;
  }
}


void init(void)
{
  ioinit();  
  scnt = 0;
  lcd_init(LCD_DISP_ON);
  lcd_led(0);
  TCNT0 = 0;
  _delay_ms(1);
  state = STATE_INTRO;

}


void run(void)
{

  uint8_t btn = 0;
  uint8_t i, btn_last, btn_event;
  uint32_t speed, temp;

  init();
  
  while(1) 
  {
    btn_last = btn;
    btn = debounce((~PIN_BTN) & 0x03);  
    
    if(btn && (btn != btn_last)) 
    {
      btn_event=btn;
      scnt=0;
    }
    else btn_event = 0;

    if(btn_event == BTN_PRESSED) 
    {
      state++;
      if(state>=STATE_NUMBERS) state=0;
      lcd_clrscr();
    }

    if(!(scnt%8)) {

      switch(state) {
  
        case STATE_INTRO:
          lcd_gotoxy(0,0);
          lcd_puts(" ++++ Monosem ++++");
          lcd_gotoxy(0,1);
          lcd_puts("       v2.0");
  
          if(scnt >= 80) {
            lcd_clrscr();
            state=STATE_RUNNING;
          }
          break;
          
        case STATE_RUNNING:
          lcd_gotoxy(0,2);
          lcd_puts(" ");
          for(i=0;i<6;i++) { 
            lcd_putc('1'+i);
            lcd_putc( (_ok & (1 << (5-i))) ? 0xFF : 0xDB);
            lcd_putc(' ');
          }
          lcd_gotoxy(0,3);
          lcd_puts(" ");
          for(i=0;i<6;i++) { 
            lcd_putc( (_ok & (1 << (5-i))) ? 0xFF : 0xDB);
            lcd_putc( (_ok & (1 << (5-i))) ? 0xFF : 0xDB);
            lcd_putc(' ');
          }
		  
          if(btn_event == BTN_OK_PRESSED) scnt = 0;  // workaround
          
          if(!(scnt%32)) {
            lcd_gotoxy(0,0);
                        //     sekunden     * mrad
            speed = ticks_to_speed(ticks);            
            sprintf(s, "%2d.%dkmh", (short)speed/100%100, (short)(speed/10)%10);
            lcd_puts(s);
            
            temp = turnarounds_to_squm(turnarounds);
            sprintf(s, "%3d.%03dha", (short)(temp/10000), (short)(temp/10%1000));      
            lcd_puts(s);
          }
          break;
          
        case STATE_HA:
          lcd_gotoxy(0,0);
          
          temp = turnarounds_to_squm(turnarounds);
          sprintf(s, "%3d.%03dha RESET", (short)(temp/10000), (short)(temp/10%1000));      
          lcd_puts(s);
          
          temp += e_read_qua_m_total();
          sprintf(s, " %6d.%03d ha", (short)(temp/10000), (short)(temp/10%1000));
          lcd_gotoxy(0,1);
          lcd_puts(s);

          if(btn_event == BTN_OK_PRESSED) 
          {
            if(e_read_qua_m_total() == -1) e_write_qua_m_total(0); 
            else e_write_qua_m_total(temp);
            turnarounds = 0;
            e_write_turnarounds(turnarounds);
          }
          break;
          
        case STATE_SETUP_TOOTH1:
          lcd_gotoxy(0,0);
          lcd_puts("Kettenrad 1");  
          if(btn_event == BTN_OK_PRESSED) {
            rad1++;
            if(rad1>=3) rad1=0;
          }
          lcd_gotoxy(0,1);
          sprintf(s, "%c: %d Zaehne", rad1+'A', zaehne1[rad1]);
          lcd_puts(s);
          break;
          
        case STATE_SETUP_TOOTH2:
          lcd_gotoxy(0,0);
          lcd_puts("Kettenrad 2");  
          if(btn_event == BTN_OK_PRESSED) 
          {
            rad2++;
            if(rad2>=6) rad2=0;
          }
          lcd_gotoxy(0,1);
          sprintf(s,"%d: %d Zaehne", 6-rad2, zaehne2[rad2]);    
          lcd_puts(s);        
          break;
          
        case STATE_SWITCHOFF:
          lcd_gotoxy(0,0);
          lcd_puts("Ausschalten");
          if(btn_event == BTN_OK_PRESSED) 
          {
            state=STATE_OFF;
          }
          break;    
          
        case STATE_OFF:
            shutdown();
            PCIFR = 0xff;  /* Clear Pin Change Interrupt Flags */
            PCMSK1 = 0x03; /* Pin Change Mask Register 1, PCINT9 PCINT8 */
            PCICR = 0x02;  /* PCINT[14:8] pins PCIE1 enabled */
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_mode();
            init();
          break;
      
      }
    }
    
    scnt++;
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
}


int main (void)
{
  run();
    return 0;  // never reached
}



SIGNAL(TIMER0_OVF_vect)    
{
  static uint16_t cnt[6] = {0};
  static uint16_t scnt[6] = {0};
  static uint16_t vorgabe;
  uint8_t i, pinvalues;
  static uint8_t test = 0;
  
  
  //if(state != STATE_RUNNING) return;
  
  if(!_ok)
  {
    vorgabe = 200;
    ticks = -1;
  }
  
  for(i = 0; i < 6; i++) 
  {
    pinvalues  = PIN_REEDS >> 2;
    if(!(pinvalues & (1 << i))) 
    {        // taste
      if(cnt[i] > 2) 
      {          // entprellen
        if(cnt[i]+scnt[i]<=13*vorgabe/10) 
        {
          turnarounds++;
          vorgabe = ticks = cnt[i] + scnt[i];
          scnt[i] = 0;
          _ok |= (1<<i);
        } 
      }
      scnt[i]++;
      cnt[i] = 0;            // taste -> cnt zurück
    }
    
    if((cnt[i] + scnt[i]) >= (13*vorgabe/10)) 
    {      // timeout
      _ok &= ~(1<<i);        // led aus
      cnt[i]--;  
      scnt[i]=0;
    } 
    cnt[i]++;
  }
  
  //PORT_6LEDS &= 0xC0;
  //PORT_6LEDS |= ((~_ok) & 0x3F);
  
  test++;
  if(test==0)
    PORT_6LEDS ^= 0x20;
}


SIGNAL(PCINT1_vect)
{
  PCMSK1 = 0; /* disable external interrupts after wakeup */
  PCICR =  0;  /* PCINT[14:8] pins PCIE1 enabled */
}

