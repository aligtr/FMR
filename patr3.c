#pragma config OSC = HS      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 10000000

#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 40000000 // ����������� �������� �������  40 ���  ��� ������� __delay_ms
#endif
#define E_pulse_with 50 //������������ �������� ��������� LCD � ���
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // ������������� ������ D4-D7 LCD
void lcd_clk(void) /*��������� �������� �� ���� EN*/
{
  LCD_E = 1;
  __delay_us(E_pulse_with);
  LCD_E = 0;
  __delay_us(E_pulse_with);
}
void lcd_command(unsigned char outbyte) /*��������� ������� (4-������ ��-��� ������) */
{
  LCD_RS=0; //����� �������� �������
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // �������� ������� ������� ���
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // �������� ������� ��-����� ���
  lcd_clk();
  __delay_ms(1);
}
void lcd_putc(char outbyte) /* ��������� ������ (4-������ ��������) */
{
  LCD_RS=1; //����� �������� ������
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
  lcd_clk();
}
void lcd_puts(unsigned char line,const char *p) // *����� ������ �� �����*
{
	lcd_origin();         // ������� � �������� ������ LCD
	lcd_command(line);			// ���������� ����� LCD 00H
	while(*p)                  // ���������, ����� �� ��������� 0
	{
	 lcd_putc(*p);             // ��������� ������ �� LCD
	 p++;                      // ��������� ����� �� 1
	}
}
void inttolcd(unsigned char posi, long value) //����� �� ����� �������� ����-������
{
	char buff[16];
	itoa(buff,value,10);
	lcd_puts(posi,buff);
}
void lcd_init() // ������������� LCD-�������
{
  TRISD &= 0x03;// ������� ������� RD4-RD7� ����� �������
  LCD_Data4 &= 0b00001111;//��������� ����� �� ����� �������� ������
  LCD_E=0;
  LCD_RS=0;
  __delay_ms(1);
/*������������� �������*/
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
 __delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
 lcd_clk();
__delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
  __delay_ms(1);
/*---------------------------------*/
  LCD_Data4=(LCD_Data4&0x0f)|0x20;	// ����������� �� 4-������ ����� ��-��-����
  lcd_clk();
  __delay_ms(1);
  lcd_command(0x28);//���������� N=1, F=0 (��� ������, ������ ������� 5*8 �����
  lcd_command(0x01);	// �������� ��
  lcd_command(0x06);	// ������������� ����������� ������ ����� �����
  lcd_command(0x0C);	// ������� �������, ������� ���, �� �������
  lcd_command(0x02);	// ��������� �������
  lcd_command(0x01);	// �������� �� �����
}
int msec=0;
void interrupt low_priority kickCont (void)
{
 if (INTCONbits.INT0IF)
 {
    __delay_ms(10);// �������� ��� ���������� ������� ��������� ������-
    __delay_ms(10);
    __delay_ms(10);
    if (PORTBbits.RB0==0)
    {
        T0CONbits.TMR0ON=!T0CONbits.TMR0ON; 
    }
 }
}

void interrupt Tover (void)
{
 if (INTCONbits.TMR0IF)
 {
INTCONbits.TMR0IF=0;
 msec++; 
 TMR0L=0b10110010;
 }
} 

void main(void) {
int min=0;
int sec=0;

    TRISBbits.RB3=0;//��������� RB3 �� ����� 
    T0CONbits.T08BIT=0;//��������� ������� �0 �� 16-������ ����� ������
T0CONbits.PSA=1;//���������� ������������ ������������
//T0CONbits.T0PS=0b110;//������������ ����� 128
T0CONbits.T0CS=0;//����� ����������� ��������� �������� ���������
//TMR0H=103;//������ �������� ����� ���������� ��������
TMR0L=0b10110010;//������ �������� ����� ���������� �������� 
T0CONbits.TMR0ON=1;
RCONbits.IPEN=1; // ���������� ������������� ����������
INTCON2bits.TMR0IP=1;//���������� ���������� ������� ����������
INTCONbits.TMR0IF=0;//��������� ����� ���������� �� ������������ ������� 0
INTCONbits.GIEH=1; //���������� ��������������� ����������
INTCONbits.GIEL=1; //���������� �������������� ����������
INTCONbits.TMR0IE=1;//���������� ���������� �� ������������ ������� 0 
  TRISBbits.RB0=1;//��������� RB0 �� ����
  INTCON2bits.INTEDG0=0; //���������� �� ������������ ������ �������
 INTCONbits.INT0IF=0; //��������� ����� ���������� �� �������� �����-
 INTCONbits.INT0IE=1;//���������� ���������� �� �������� ���������
 LATBbits.LATB3=1;
 lcd_init();
T0CONbits.TMR0ON=1; 

 while(1){
	if (msec==60)
	{
		msec=0;
		sec++;
	}
	if (sec==60)
	{
		sec=0;
		min++;
	}
	inttolcd(0xC0, min);
	inttolcd(0xC0, i);
	inttolcd(0xC0, sec);
	inttolcd(0xC0, i);
     	inttolcd(0xC0, msec);
 }
 
}       


  

