#define F_CPU  (1000000*16)
#define BAUD 9600
#define TESTSTRING(s) for(int j=0;j<=sizeof(s);j++)s[j]='q'

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <asf.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
/************************************************************************/
/* VERSCHALTUNG                                                         */
/************************************************************************/
//Ultraschallsensor
// Steuerungpin an PINA.4
// Ausgangspin an INT0


//Triangulationssensor
// Ausgang an PINa.5
// 
//Hallsensor
// Pina.1
/************************************************************************/
/* VARIABLEN FÜR ULTRASCHALLSENSOR                                      */
/************************************************************************/
volatile unsigned int uls_zeit_mios_int0;	
/**speichert den Interruptüberlauf in millionstel sekunden*/
volatile unsigned int uls_zeit_taus;		
/**speichert den Interruptüberlauf in tausendstel sekunden*/
volatile unsigned int uls_zeit_ganz;		
/**speichert den Interruptüberlauf in Sekunden*/
volatile unsigned int uls_MessungInProgress=0;
volatile unsigned int uls_SteigendeFlanke=1;
int current_adc_channel=1;

float erg_uls=0;
float erg_tria=0;
float erg_hall1=0;
/* 1 -> Triangulation Kanal 5
 * 2 -> Hall Kanal X
 */


/************************************************************************/
/* TIMERFUNKTIONEN                                                      */
/************************************************************************/
void long_delay(uint16_t ms)//Debug und Testfunktion
//wartet Zeit in Millisekunden
{
	for (; ms>0; ms--) _delay_ms(1);
}
void timer0Stop(void)
{
	TCCR0 = 0b0;//Prescaler = 0 % Timer hält an
}
void timer0Start(void)
{
	uls_zeit_ganz=0;
	uls_zeit_taus=0;
	uls_zeit_mios_int0=0;//Reset der Hilfsvariablen
	TCNT0 = 0x00;//Reset des aktuellen Timerwertes
	TCCR0 = 0x01;//Prescaler = 1 % Für max Genauigkeit
}

/************************************************************************/
/* USART FUNKTIOENN                                                     */
/************************************************************************/
void uartPutChar(char data)
{
	while (!(UCSRA&32));
	UDR=data;
}
char uartGetChar(void)
{
	char data=0;
	while (!(UCSRA&128));
	data=UDR;
	return data;
}
void uartPutString(const char *buffer)
{
	volatile int i=0;
	while(buffer[i+1] != '\0')
	{
		uartPutChar (buffer[i]);
		i=i+1;
	}
	return;
}

/************************************************************************/
/* INITIALISIERUNG                                                      */
/************************************************************************/
void Init(void)
{
	/***UART INITIALISIERUNG**/
	UCSRB = (1<< RXEN) | (1 << TXEN);
	UBRRL=(uint8_t)(F_CPU/(BAUD*16L))-1;
	UBRRH=(uint8_t)((F_CPU/(BAUD*16L))-1)>>8;

	/***TIMER 0 INITIALISIERUNG **/
	TCCR0 = 0b00;		//prescaler
	TIMSK |= (1<<TOIE0);//Interrupt eingeschaltet

	/***INTERRUPT INITIALISIERUNG **/
	GICR|=(1<<INT0);	//Interrupt0
	MCUCR|=(0<<ISC01)|(1<<ISC00);//jede Flanke
	
	/***ADC TINITIALISIERUNG***/
  
	ADMUX = (1<<REFS0);//Versorgungsspannung als Referenz
	ADMUX |= (0<<MUX2) | (0<<MUX0);//Kanal 5
	ADMUX |= (1<<ADLAR);
	
  
	ADCSRA =  (1<<ADPS2);     // Frequenzvorteiler
	ADCSRA |= (1<<ADEN);      // ADC aktivieren
}

/************************************************************************/
/* ADC-MESSUNG                                                          */
/************************************************************************/
unsigned char getADC()
{
	ADCSRA 	|= (1<<ADSC);	// Start ADC
	while(ADCSRA & 0x40) 	// Warten bis fertig
	{}
	unsigned char wert=ADCH; 	// Einlesen des Analogwertes
	return wert;
}

/************************************************************************/
/* ADC-WECHSEL                                                          */
/************************************************************************/
//nummer 1--> Triangulation Kanal 5
//nummer 2--> Hall Kanal 0
int changeADC(int nummer)
{
	
	if (nummer==current_adc_channel)return 1; //Abbruch falls ADC nicht 
	// gewechselt werden muss
	
	switch (nummer)
	{
		
		case 1: {
					ADMUX = (1<<REFS0);//Versorgungsspannung als Referenz
					ADMUX |= (1<<MUX2) | (1<<MUX0);//Kanal 5
					ADMUX |= (1<<ADLAR);
				}
		break;
		case 2:{
					ADMUX = (1<<REFS0);//Versorgungsspannung als Referenz
					ADMUX |= (0<<MUX2) | (1<<MUX0);//Kanal 0
					ADMUX |= (1<<ADLAR);
				}
		break;
		default:return 0;//falsche Zahl eingegeben
	}
	//erste Messung funktioniert nicht nach Wechsel daher eine Testmessung
	getADC();
	current_adc_channel=nummer;
	return 1;
}




/************************************************************************/
/* INTERRUPTS                                                           */
/************************************************************************/
ISR (TIMER0_OVF_vect)
{
	//Zeitvariablen werden hochgezählt
	uls_zeit_mios_int0=uls_zeit_mios_int0+16;
	//16 millionstel Sekunden als kleinste Messbare Zeiteinheit
	if ( uls_zeit_mios_int0 >= 1000)
	{
		uls_zeit_taus++;
		uls_zeit_mios_int0=uls_zeit_mios_int0-1000;
		if (uls_zeit_taus >= 1000)
		{
			uls_zeit_ganz++;
			uls_zeit_taus=0;
		}
	}
}
ISR(INT0_vect)
{
	volatile char buffer[10];
	
	if ( uls_SteigendeFlanke == 0)
	{
		//Bei fallender Flanke: Timer anhalten und Messung durchführen
		timer0Stop();
		uls_SteigendeFlanke=1;
		
		//AUSGABE
		volatile uint16_t uls_zeit_mios_rest;
		volatile float uls_zeit_mios_total;
		volatile char buffer[10];
		float uls_schallgeschwindigkeit=330;
		
		/*Streckenausgabe*/
		uls_zeit_mios_rest=0;//TCNT0/255;		
		uls_zeit_mios_total=(float)uls_zeit_mios_int0+uls_zeit_mios_rest;
		float uls_strecke=1.03525*((uls_schallgeschwindigkeit/2)*((float)uls_zeit_ganz+((float)uls_zeit_taus/1000)+((float)uls_zeit_mios_total/1000000)));
		/*Zeitausgabe*/
		//dtostrf(uls_zeit_ganz, sizeof(buffer), 3, buffer );
		//uartPutString(buffer);
		//dtostrf(uls_zeit_taus, sizeof(buffer), 3, buffer );
		//uartPutString(buffer);
		//dtostrf( uls_zeit_mios_total, sizeof(buffer), 3, buffer );
		//uartPutString(buffer);
		//uartPutChar(13);
		erg_uls=uls_strecke;		
		uls_MessungInProgress=0;
		
	}
	else
	{
		//Steigende Flanke: Zeitmessung starten
		timer0Start();
		uls_SteigendeFlanke=0;
	}
}
/************************************************************************/
/* STARTE DER MESSUNGEN                                                 */
/************************************************************************/
int uls_messungstarten()
{
	if (uls_MessungInProgress==0)
	{
		//abfrage ob ULS in benutzung ist
		uls_MessungInProgress=1;
		uls_SteigendeFlanke=1;
		PORTA=PORTA|0b00010000;
		_delay_us(100);
		PORTA=PORTA&0b11101111;
		return 1;
	}
	else
	{
		return 0;
	}
}
int tria_messungstarten()
{
	changeADC(1);//Auswahl des ADC Kanals
	volatile char buffer[10];
	uint16_t result;
	float Spannung;
	float Strecke;
	
	result = getADC();//ADC Messung
	Spannung = (float)result*5/256;//Umrechnungen
	if(Spannung<0.01) return 0;
	else if (Spannung>0.7)
	{
		Strecke=10.1135/Spannung+0.2383;
	}
	else
	{
		Strecke = 13.4802/Spannung-0.0292;

	}
	
	erg_tria=Strecke;
	return 1;
}
int halla_messungstarten()
{
	changeADC(2);
	
	uint16_t result;
	float Spannung;
	float Flussdichte;//in Gauß
	
	result = getADC();
	Spannung = (float)result*5/256;
	Flussdichte=(Spannung-2.5)*320;
	
	erg_hall1=Flussdichte;	
	return 1;
}
int ausgabe(const char *sensor,float zahl, const char *einheit )
{
	volatile char buffer[10];
	uartPutString(sensor);
	dtostrf(zahl, sizeof(buffer), 3, buffer );
	uartPutString(buffer);
	uartPutString(einheit);
	
	uartPutChar(13);
}
int main (void)
{
	board_init();           
	Init();
	DDRA=0b00010001;	
	long_delay(1000);
	PORTA=0x00;
	sei();
	/* ausschalten wenn uls nicht angeschlossen ist um 
	 * flackern des Interrupts zu vermeiden
	 * alternativ: Pull-Down Widerstand 
	 */
	
	
	
	while(true)
	{
		long_delay(1000);//Wartezeit zwischen einzelnen Messungen
		
		/*Ultraschallsensor*/
		if(uls_messungstarten()==0)
		{
			uartPutString("Uls Messung failed!");
			uartPutChar(13);
		}
		else
		{
			ausgabe("Ultraschall: ",erg_uls, " Meter ");
		}
		
		/*Triangulationssensor*/
		if(tria_messungstarten()==0)
		{
			uartPutString("Tri Messung failed!");
			uartPutChar(13);
		}
		else
		{
			ausgabe("Triangulation: ",erg_tria, " cm ");
		}
		
		/*Hallsensor*/
		if(halla_messungstarten()==0)
		{
			uartPutString("Hall Messung failed!");
			uartPutChar(13);
		}
		else
		{
			ausgabe("Hallsensor: ",erg_hall1, " Gauss ");
		}
	}
}
