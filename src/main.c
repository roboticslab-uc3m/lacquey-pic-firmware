#include <p18F2580.h>
#include "ecan.h"
#include <delays.h>
#include "timers.h"
#include <pwm.h>

#pragma config OSC = HS, FCMEN = OFF, IESO = OFF
#pragma config PWRT = ON, BOREN = OFF, BORV = 1
#pragma config WDT = OFF, WDTPS = 32768
#pragma config PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON
#pragma config STVREN = OFF, LVP = OFF, XINST = OFF


#define nodeId 0x640  //Única ID a la que responderá la mano

#define TRISDIR1 TRISAbits.RA0 
#define TRISDIR2 TRISAbits.RA1 
#define TRISENABLE TRISCbits.RC2 
#define PORTDIR1 PORTAbits.RA0
#define PORTDIR2 PORTAbits.RA1
#define PORTENABLE PORTCbits.RC2
#define LATDIR1 LATAbits.LATA0
#define LATDIR2 LATAbits.LATA1
#define LATENABLE LATCbits.LATC2

#define TRISINDI TRISCbits.RC5
#define TRISABRIR TRISCbits.RC6
#define TRISCERRAR TRISCbits.RC7
#define PORTINDI PORTCbits.RC5
#define PORTABRIR PORTCbits.RC6
#define PORTCERRAR PORTCbits.RC7
#define LATINDI LATCbits.LATC5
#define LATABRIR LATCbits.LATC6
#define LATCERRAR LATCbits.LATC7

void delayms(int t);
void stopPWM(void);
void startPWM(void);
void setId(int ID);
void main(void){
	//Variables que almacenarán los datos recibidos:
	unsigned long IDOrden;
	BYTE Orden[8];	//Los dos últimos bits Orden[0] serán los bits menos significativos del PWM
	BYTE OrdenLon;
    ECAN_RX_MSG_FLAGS OrdenFlags;

	BYTE OrdenBits; // Parte extraida del mensaje que contiene la orden
	unsigned int PWMBits;//parte extraida del mensaje que contiene el valor del PWM

	//Variables que almacenarán los datos a enviar:
	unsigned long IDOrden1=0b11001000000;
	BYTE Orden1[8];
	BYTE OrdenLon1=2;
	ECAN_RX_MSG_FLAGS OrdenFlags1=0;
	OrdenFlags1=ECAN_TX_STD_FRAME;

	//Configurar e inicializar puertos
	TRISDIR1=0;
	TRISDIR2=0;
	TRISENABLE=0;
	LATDIR1=0;
	LATDIR2=0;
	LATENABLE=1;//Este valor es muy importante y debe ser 1, pues es la señal que se emitirá
				//cuando se apague el PWM en la excepción de 1023 (si no hay un pequeño
				//bajo y el osciloscpio no es capaz de detectar bien el ciclo de trabajo).

	TRISINDI=0;
	TRISABRIR=0;
	TRISCERRAR=0;
	LATINDI=1;
	LATABRIR=0;
	LATCERRAR=0;

	//Configuración PWM:
	OpenPWM1(255);    // Configura el PWM para la máxima resolución, con una frec de 1,22 kHz
 	OpenTimer2(TIMER_INT_OFF & T2_PS_1_16); // Enciende y configura el timer 2 con preescalado de 16
											//para tener la frec de 1,22 kHz
  	SetDCPWM1(0);   // Inicializa a nivel bajo (PWM 0%)
	 
	//Configuración CAN. Inicializa para 1Mbps con estos valores:
	//SJW, BRP, PHSEG1, PHSEG2, PROPSEG
	// 1,   1,    5,      3,      1
	ECANInitialize();
	//Activa filtros y asigna la aceptación del ID que se ha marcado
	setId(nodeId);

	while(1){
		//				 0xA0 -> Mantener abriendo
		//				 0xC0 -> Mantener cerrando
		//				 0xF0 -> Parar (dejar de actuar, enable a 0)

		while(!ECANReceiveMessage(&IDOrden,Orden,&OrdenLon,&OrdenFlags));
		OrdenBits=Orden[0] & 0b11110000;
		PWMBits=((unsigned int)Orden[1]<<2)|(Orden[0] & 0b00000011);
		switch (OrdenBits){
		case 0xA0:
			LATDIR1=1;
			LATDIR2=0;
			delayms(10);
			if (PWMBits==1023)
				stopPWM();
			else{
				startPWM();
				SetDCPWM1(PWMBits);
			}
			LATABRIR=1;
			LATCERRAR=0;
			break;
		case 0xC0:
			LATDIR1=0;
			LATDIR2=1;
			delayms(10);
			if (PWMBits==1023)
				stopPWM();
			else{
				startPWM();
				SetDCPWM1(PWMBits);
			}				
			LATABRIR=0;
			LATCERRAR=1;
			break;
		case 0xF0:
			startPWM();
			SetDCPWM1(0);
			LATABRIR=0;
			LATCERRAR=0;
			delayms(5);
			LATDIR1=0;
			LATDIR2=0;
			break;
		default:
			startPWM();
			SetDCPWM1(0);
			LATABRIR=0;
			LATCERRAR=0;
			delayms(5);
			LATDIR1=0;
			LATDIR2=0;
		
		}
	}
}

void delayms(int t){
	int i;
	for(i=0;i<t;i++){
		Delay1KTCYx(5); //1 ms para reloj de 20MHz, contando con que tarda 4 ciclos por instrucción
	}
}

void startPWM(void){
	CCP1CON |= 0b00001100;
}
void stopPWM(void){
	CCP1CON &= 0b11110011;
}

void setId(int ID){
	RXF5SIDH=RXF4SIDH=RXF3SIDH=RXF2SIDH=RXF1SIDH=(BYTE)(ID>>3);
	RXF5SIDL=RXF4SIDL=RXF3SIDL=RXF2SIDL=RXF1SIDL=(BYTE)((ID<<5)&0b11100000);
	
	RXM0SIDH= 0b11111111;   //MASK every identifier bit
	RXM0SIDL= 0b11100000;
	RXM1SIDH= 0b11111111;   //MASK every identifier bit
	RXM1SIDL= 0b11100000; 
}
