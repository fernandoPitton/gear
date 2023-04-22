#include <Arduino.h>

#include <SPI.h>
#include <SD.h>
#define SSpin 53
boolean estadotarjeta = 0;
File archivo;

int menuANTIFALLOSLENTO(String *arrayMenu,int size);
void ActivarControldeTraccion();
void lecturaSdel();
void lecturaStras();
void digitalPotWrite(int value);


byte address = 0x00; // direccion spi pot digital
byte CS = 44;  // pin salida CS pot digital

#include <Adafruit_NeoPixel.h>    // importa libreria led pixel 
Adafruit_NeoPixel tira = Adafruit_NeoPixel(8,41, NEO_GRB + NEO_KHZ800); 

#include <EEPROM.h>
#include <Streaming.h>

#include <LiquidCrystal.h>

//LCD display pins
#define LCD_RS 9
#define LCD_EN 8
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5 
#define LCD_D7 4

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
/*
#include "HX711.h"
const int DOUT=A9;  //DT
const int CLK=A10;   //SCK
HX711 sensorpalanca; 
*/
//#include <GyverHX711.h>
//GyverHX711 sensorpalanca(A9, A10, HX_GAIN64_A);
// HX_GAIN128_A - canal A ganancia 128 
// HX_GAIN32_B - canal B ganancia 32 
// HX_GAIN64_A - canal A ganancia 64 

///-------------------------variables menu--------------------------

int Estado = 2; 
int Sig_Estado = 2;
 byte contador =0;
 byte contador2 = 1;
 int opcionAnterior =1;
 int opcionAnteriorSub =1;
#define salidaBeep 12

//Botonera
#define botArriba 22   
#define botAbajo 25
#define botDerecha 24
#define botIzquierda 23

///-----------------variables Boost Control------------------------
#define salidaBoost 2
byte PWMBoost =0;
byte PWMBoostP =0;
byte PWMBoostold =0;
byte boostBurnOut =0;

byte boostLargada =150;
byte boost1 =10;
byte boost2 =25;
byte boost3 =40;
byte boost4 =60;
byte boost5 =100;
int boostVel1 =50;
int boostVel2 =100;
int boostVel3 =150;
int boostVel4 =200;
int boostVel5 =250;


byte burnOutset = 0;
///-----------------variables control de traccion-------------------------
int rpsDel =0;
int rpsTras =0;
int Trastick =0;

float diametroruedaTras=0.54; //constante diametro da rueda trasera
float diametroruedaDel=0.54; //constante diametro da rueda delantera
float ajusteTras = 3.3; //ajusta la lectura de velocidad
float ajusteDel = 3.3;



int velocidadDel =0;
int velocidadTras=0;

float perimetroRueda =0;
float Radio =0;
float parcial18m = 0;
float parcial100m = 0;
float parcial201m = 0;
float Nticks18m  = 0;
float Nticks100m  = 0;
float Nticks201m  = 0;
int Vparcial18m =0;
int Vparcial100m =0;
int Vparcial201m =0;
float tiempoLargada =0;
float tLogger =0; 

volatile unsigned tiempoActual =0;
volatile unsigned tiempoAnterior =0;
volatile unsigned tiempoAntTraccion =0;
volatile unsigned tiempoAntLogger =0;

volatile unsigned tiempoanteriorciclo =0;
volatile unsigned tiempodeciclo =0;


byte contadorEstadoCT = 0;

byte Destraccion = 0; // cantidad de velocidad extra permitida de ruedadel sodre ruedatras 

byte Destraccion1 = 20; //porcentajes de patinada de la rueda delantera en el control de traccion
byte Destraccion2 = 25;
byte Destraccion3 = 30;
byte Destraccion4 = 35;
byte Destraccion5 = 40;

int Agresividad = 0;

byte VelInicial = 60;

byte aceleracionInicial = 10;


 
byte estadoControlT = 0;


///-------------------------------------------------------------------

#define TPS A8
int lecturaTPS = 0;
byte TPSporcentaje;

int sensibilidadPalanca = 200; //valor de 1 a 1000 donde 1 es muy sencible 

int calPalanca = 0; //para calibrar el 0 de la palanca

int voltageSensorPalanca=0;  //almacena el valor del sensor de palanca 
int voltageSensorPalanca2=0; //almacena el valor del sensor de palanca para la funcion de proteccion de cambio accidental
byte controlador = 0;  // almacena el estado del pulsador del controlador 
#define cortesalida  33 //define el pin  como salida de controlador a la 350
byte cortesalidalog = 1;
//bool actSerial = false;
bool Rele=false; 
//---para almacenar el estado anteriar del sensor de palanca de cambio
int  estadoanteriorHI = 0;
int  estadoanteriorLOW = 0;
//---- variables para flacos de subida y bajada de disparos de cambios--------
int TriggerHi= 50;
int TriggerLOW= -50;
int rearmHI = 20;
int rearmLOW = -20;

//----------------------------------
int Lec0 = 0;
int Lec1024 =0;
byte setTPS = 80; 
 
int tiemposhift = 0;       //se le asigna el tiempo q demora en entrar el cabio segun etapa        
int tiempocorte1 = 250;       //se le asigna el tiempo q demora en entrar el cabio segun etapa
int tiempocorte2 = 200;       //se le asigna el tiempo q demora en entrar el cabio segun etapa
int tiempocorte3 = 150;       //se le asigna el tiempo q demora en entrar el cabio segun etapa
int tiempocorte4 = 100;       //se le asigna el tiempo q demora en entrar el cabio segun etapa
int offset = 1500;            //tiempo muerto despues de un disparo de cambio


unsigned long T_Apagar=0;           // Para controlar el tiempo consumido de activacio nde cambio
unsigned long T_Apagar2=0;         // Para controlar el tiempo consumido de activacio de offset
byte contadorShift =0;           //contador para el swich de las etapas de cambios
int calinicio = 0;
// Variables lectura de velocidad Delantera

 byte pprDel = 60;  // ranuras del encoder delantero
const unsigned long tiempoZeroDel = 200000;  // tiempo maximo entre pulsos del encoder para detectar que la rueda deja de girar
                                             // entre100000 300000. mientras menor sea es mas sensible 
volatile unsigned long tiempoAnteriorDel;  
volatile unsigned long periodoEntrePulsosDel = tiempoZeroDel+1000;                      
volatile unsigned long periodoPromedioDel = tiempoZeroDel+1000;  
unsigned long frecSinProcesarDel;  
unsigned long frecRealDel; 
unsigned int pulsosContadorDel = 1;    
unsigned long sumaPeriodoDel; 
unsigned long cicloTiempoAnteriorDel = tiempoAnteriorDel; 
unsigned long actualMicrosDel = micros(); 
unsigned int cantidadLecturaDel = 7;
unsigned int reboteAdiCeroDel ; 
// Suavisado De Lectura de Velocidad:
const byte numReadingsDel = 3;  // Número de muestras para alisar. Cuanto más alto, más suavizado, pero va a
                             // reacciona más lento a los cambios. 1 = sin suavizado. Predeterminado: 2.

unsigned long readingsDel[numReadingsDel];  // The input.
unsigned long readIndexDel;  // The index of the current reading.
unsigned long totalDel;  // The running total.
unsigned long averageDel;  // The RPM value after applying the smoothing.
 

// Variables lectura de velocidad Trasera

 byte pprTras = 16;  // ranuras del encoder trasero
const unsigned long tiempoZeroTras = 200000; 
volatile unsigned long tiempoAnteriorTras;
volatile unsigned long periodoEntrePulsosTras = tiempoZeroTras+1000; 
volatile unsigned long periodoPromedioTras = tiempoZeroTras+1000; 
unsigned long frecSinProcesarTras;
unsigned long frecRealTras;
unsigned int pulsosContadorTras = 1;
unsigned long sumaPeriodoTras; 
unsigned long cicloTiempoAnteriorTras = tiempoAnteriorTras;
unsigned long actualMicrosTras = micros(); 
unsigned int cantidadLecturaTras = 2;
unsigned int reboteAdiCeroTras ;
// Suavisado De Lectura de Velocidad:
const byte numReadingsTras = 3;  // Número de muestras para alisar. Cuanto más alto, más suavizado, pero va a
                             // reacciona más lento a los cambios. 1 = sin suavizado. Predeterminado: 2.

unsigned long readingsTras[numReadingsTras];  // The input.
unsigned long readIndexTras;  // The index of the current reading.
unsigned long totalTras;  // The running total.
unsigned long averageTras;  // The RPM value after applying the smoothing.



void setup()
{
 
  pinMode(botArriba,INPUT_PULLUP);   
  pinMode(botAbajo, INPUT_PULLUP);   
  pinMode(botDerecha,INPUT_PULLUP);
  pinMode(botIzquierda,INPUT_PULLUP);
 
  lcd.begin(16, 2);
  lcd.setCursor (4, 0);
  lcd.print("FerTech");
  lcd.setCursor (0, 1);
  lcd.print("Drag Controller"); 

 
  pinMode (lecturaTPS, INPUT);
 
  pinMode (cortesalida, OUTPUT);
  pinMode (salidaBeep, OUTPUT);
  pinMode (salidaBoost, OUTPUT);
  Serial.begin(9600);
  //sensorpalanca.begin(DOUT, CLK); //inicia sensor de palanca
  
  

  EEPROM.get( 0,tiempocorte1 ); 
  EEPROM.get( 2,tiempocorte2 ); 
  EEPROM.get( 4,tiempocorte3 ); 
  EEPROM.get( 6, tiempocorte4 ); 
  EEPROM.get( 9,offset ); 
  EEPROM.get( 11,TriggerHi ); 
  EEPROM.get( 13,rearmHI ); 
  EEPROM.get( 15,TriggerLOW ); 
  EEPROM.get( 17,rearmLOW ); 
  EEPROM.get( 19,setTPS ); 
  EEPROM.get( 21,Lec0 ); 
  EEPROM.get( 23,Lec1024 ); 
  EEPROM.get( 25,boostLargada );
  EEPROM.get( 27,boost1 );
  EEPROM.get( 29,boost2 );
  EEPROM.get( 31,boost3 );
  EEPROM.get( 33,boost4 );
  EEPROM.get( 35,boost5 );
  EEPROM.get( 37,boostVel1 );
  EEPROM.get( 39,boostVel2 );
  EEPROM.get( 41,boostVel3 );
  EEPROM.get( 43,boostVel4 );
  EEPROM.get( 45,boostVel5 );
  EEPROM.get( 49,diametroruedaTras );
  EEPROM.get( 53,diametroruedaDel ); 
  EEPROM.get( 57,pprTras );
  EEPROM.get( 61,pprDel );
  EEPROM.get( 65,ajusteTras );
  EEPROM.get( 69,ajusteDel );
  EEPROM.get( 73,sensibilidadPalanca );
  EEPROM.get( 77,Destraccion1 );
  EEPROM.get( 81,Destraccion2 );
  EEPROM.get( 85,Destraccion3 );
  EEPROM.get( 89,Destraccion4 );
  EEPROM.get( 93,Destraccion5 );
  EEPROM.get( 97,Agresividad );
  EEPROM.get( 101,VelInicial );
  EEPROM.get( 105,estadoControlT );
  EEPROM.get( 109,boostBurnOut);

  tira.begin();       // inicializacion de la tira
  tira.show();        // muestra datos en pixel
 tira.setBrightness(60);
 //tira.setPixelColor(0, 255, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
 //   tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(1, 255, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
tira.setPixelColor(2, 255, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(3, 0, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(4, 0, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(5, 0, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(6, 255, 0, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 tira.setPixelColor(7, 255, 0, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
 delay(250);
 
 tira.clear(); 
  tira.show(); 

 //------------ Inicializando POT DIGITAL-------------
 pinMode (CS, OUTPUT);
 SPI.begin();
 
  //------------ Inicializando tarjeta SD-------------
  
//Serial.println("Inicializando tarjeta SD ...");  // texto en ventana de monitor
  if (!SD.begin(SSpin)) {     // inicializacion de tarjeta SD
    estadotarjeta = 0;
   tira.setPixelColor(0, 255, 0, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();      // muestra datos en pixel 
    }
//----- imprime datos para data logger----------
  

  else {
    estadotarjeta = 1;
  tira.setPixelColor(0, 0, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();  
  //Serial.println("inicializacion correcta");  // texto de inicializacion correcta
  archivo = SD.open("Logger.msl", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt  
  archivo.print("Time");  // escritura de una linea de texto en archivo
  archivo.print("\t");
  archivo.print("CONTROLADORIN");
  archivo.print("\t"); 
  archivo.print("VSHIFT");
  archivo.print("\t"); 
  archivo.print("Gear");
  archivo.print("\t"); 
  archivo.print("CONTROLADOROUT");
  archivo.print("\t");
  archivo.print("PWMBoost");
  archivo.print("\t");
  archivo.print("TPS");
  archivo.print("\t");
  archivo.print("ConTrac");
  archivo.print("\t");
  archivo.print("T18m");
  archivo.print("\t");
  archivo.print("T100m");
  archivo.print("\t");
  archivo.print("T201m");
  archivo.print("\t");
  archivo.print("Vtras");
  archivo.print("\t");
  archivo.println("VDel"); 

  archivo.print("s");
  archivo.print("\t");
  archivo.print("ON");
  archivo.print("\t"); 
  archivo.print("V");
  archivo.print("\t"); 
  archivo.print("Numero");
  archivo.print("\t");
  archivo.print("ON");
  archivo.print("\t");
  archivo.print("%");
  archivo.print("\t");
  archivo.print("%");
  archivo.print("\t");
  archivo.print("Nivel");
  archivo.print("\t");
  archivo.print("Seg");
  archivo.print("\t");
  archivo.print("Seg");
  archivo.print("\t");
  archivo.print("Seg");
  archivo.print("\t");
  archivo.print("Km/h");
  archivo.print("\t");
  archivo.println("Km/h");
  archivo.close();        // cierre del archivo
    
  } 
  

//-----control de tracion---------------
attachInterrupt(4, lecturaSdel, RISING);             // linea para añadir una interrupciòn a un PIN
attachInterrupt(5, lecturaStras, RISING);           // linea para añadir una interrupciòn a un PIN


//----- perimetro rueda
Radio = diametroruedaTras/2;
perimetroRueda = 2*PI*Radio;
  
  //---------calculo ticks para distancia
Nticks18m = 18 * pprTras /perimetroRueda;
Nticks100m = 100 * pprTras /perimetroRueda;
Nticks201m = 201 * pprTras /perimetroRueda;

    
TCCR3B = TCCR3B & B11111000 | B00000101; //cambia la frecuencia del pin d2 a 30hz PWM



}

void loop()
{
  tiempoActual = millis();


 
//-------------****MENU****-----------
  
  Estado = Sig_Estado;
  
  if(Estado == 1)
  {
     
      int menu;
      String arrayMenu[] = {"Salir","Boost Control","Gear Control","Trac Control","Configuracion","TPS Calibracion"};
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      
       menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1;
      else if(menu == 1){Sig_Estado = 2;delay(500);}
      else if(menu == 2)Sig_Estado = 3;
      else if(menu == 3)Sig_Estado = 4;
      else if(menu == 4)Sig_Estado = 7;
      else if(menu == 5)Sig_Estado = 6;
      else if(menu == 6)Sig_Estado = 5;
  }
  else if( Estado == 2)
  {
   
    if(digitalRead(botDerecha) ==0){Sig_Estado = 1;}
    if (digitalRead(botIzquierda) ==0){Sig_Estado = 22;estadoControlT =0;}
    if (digitalRead(botAbajo) ==LOW && contador < 4){contador++; delay(200);} 
    if (digitalRead(botArriba) ==LOW && contador > 0 ){contador--;delay(200);}
    
   
    switch (contador) {
        case 0: {
       if (tiempoActual - tiempoAnterior >= 350){
        lcd.setCursor(0, 0);lcd.print("TPS");lcd.print(TPSporcentaje);lcd.print("%  ");
        lcd.setCursor(7, 0);lcd.print("  Vs:" );lcd.print(voltageSensorPalanca2);lcd.print("    ");
        lcd.setCursor(0, 1);lcd.print("PWM");lcd.print(PWMBoostP);lcd.print("%   ");
        lcd.setCursor(8, 1);lcd.print(contadorShift + 1);lcd.print("Gear     ");
        tiempoAnterior= tiempoActual;  
       }


        break;
}
        case 1:{
          if (tiempoActual - tiempoAnterior >= 350){
          lcd.setCursor(0, 0); lcd.print("Traccion:");lcd.print(velocidadDel);lcd.print("km/h  ");
          
          lcd.setCursor(0, 1);lcd.print("Arrastre:");lcd.print(velocidadTras);lcd.print("km/h  ");
          tiempoAnterior= tiempoActual;
       }

          break;
          
          }
    
        case 2:{
          lcd.setCursor(0, 0);
        lcd.print("       18m      ");
        lcd.setCursor(0, 1);
        lcd.print(parcial18m);
        lcd.print("s   ");
         lcd.setCursor(8, 1);
        lcd.print("@  ");
        lcd.setCursor(11, 1);
        lcd.print(Vparcial18m);
        lcd.print("km/h");   
          break;
          }
       case 3:{
          lcd.setCursor(0, 0);
        lcd.print("      100m      ");
        lcd.setCursor(0, 1);
        lcd.print(parcial100m);
        lcd.print("s   ");
        lcd.setCursor(8, 1);
        lcd.print("@  ");
        lcd.setCursor(11, 1);
        lcd.print(Vparcial100m);
        lcd.print("km/h");    
          break;
          }
       case 4:{
          lcd.setCursor(0, 0);
        lcd.print("      201m      ");
        lcd.setCursor(0, 1);
        lcd.print(parcial201m);
        lcd.print("s   ");
         lcd.setCursor(8, 1);
        lcd.print("@  ");
        lcd.setCursor(11, 1);
        lcd.print(Vparcial201m);
        lcd.print("km/h");
          break;
          }

       
    }
    
    
  
}
 else if (Estado == 22){
    
    if (digitalRead(botAbajo) ==LOW && boostBurnOut > 0){boostBurnOut= (boostBurnOut -2); delay(150);} 
    if (digitalRead(botArriba) ==LOW && boostBurnOut <100){boostBurnOut= (boostBurnOut +2); delay(150);}
    burnOutset =1;
    lcd.setCursor(0, 0);
        lcd.print(" BURNOUT ACTIVO ");
        lcd.setCursor(2, 1);
       lcd.print("PWMBoost ");
       lcd.print(boostBurnOut);
       lcd.print("%   ");
    
     if(digitalRead(botDerecha) ==0){Sig_Estado = 2; burnOutset =0;EEPROM.update( 109,boostBurnOut );estadoControlT =1;delay(200);}
  
  }

  else if( Estado == 3)
  {
      int menu;
      String arrayMenu[] = {"PresionLargada","Presion1","Presion2","Presion3","Presion4","Presion5","Velocidad1","Velocidad2","Velocidad3","Velocidad4","Velocidad5"};
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      
      menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1; 
      else if(menu == 1){Sig_Estado = 31;lcd.clear();}
      else if(menu == 2){Sig_Estado = 32;lcd.clear();}
      else if(menu == 3){Sig_Estado = 33;lcd.clear();}
      else if(menu == 4){Sig_Estado = 34;lcd.clear();}
      else if(menu == 5){Sig_Estado = 35;lcd.clear();}
      else if(menu == 6){Sig_Estado = 36;lcd.clear();}
      else if(menu == 7){Sig_Estado = 37;lcd.clear();}
      else if(menu == 8){Sig_Estado = 38;lcd.clear();}
      else if(menu == 9){Sig_Estado = 39;lcd.clear();}
      else if(menu == 10){Sig_Estado = 310;lcd.clear();}
      else if(menu == 11){Sig_Estado = 311;lcd.clear();}

     
  }
      else if( Estado == 31)
      {
        
        if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update(25, boostLargada);}
    if (digitalRead(botAbajo) ==LOW && boostLargada > 0){boostLargada= (boostLargada -2); delay(150);} 
    if (digitalRead(botArriba) ==LOW && boostLargada <100){boostLargada= (boostLargada +2); delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Presion Largada");
       lcd.setCursor(4, 1);
       lcd.print("PWM");
       lcd.print(boostLargada);
       lcd.print("% ");
       if(digitalRead(botDerecha) ==LOW){Sig_Estado = 32; EEPROM.update(25, boostLargada);delay(250);lcd.clear();}

      }
      else if( Estado == 32)
      {
       
        if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update(27,boost1);}
    if (digitalRead(botAbajo) ==LOW && boost1>0){boost1= (boost1 -2);delay(150);} 
    if (digitalRead(botArriba) ==LOW && boost1<100){boost1= (boost1 +2);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Presion 1");
       lcd.setCursor(0, 1);
       lcd.print("0/");lcd.print(boostVel1);
       lcd.setCursor(9, 1);
       lcd.print("PWM");
       lcd.print(boost1);
       lcd.print("% ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 33;EEPROM.update(27,boost1);delay(250);lcd.clear();}

      }

      else if( Estado == 33)
      {
              
          if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update(29,boost2);}
    if (digitalRead(botAbajo) ==LOW && boost2>0){boost2= (boost2 -2);delay(150);} 
    if (digitalRead(botArriba) ==LOW && boost2<100){boost2= (boost2 +2);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Presion 2");
       lcd.setCursor(0, 1);
       lcd.print(boostVel1);lcd.print("/");lcd.print(boostVel2);
       lcd.setCursor(9, 1);
       lcd.print("PWM");
       lcd.print(boost2);
       lcd.print("% ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 34;EEPROM.update(29,boost2);delay(250);lcd.clear();}

      }

      else if( Estado == 34)
      {
         
          if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 31,boost3 );}
    if (digitalRead(botAbajo) ==LOW && boost3>0){boost3= (boost3 -2);delay(150);} 
    if (digitalRead(botArriba) ==LOW && boost3<100){boost3= (boost3 +2);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Presion 3");
       lcd.setCursor(0, 1);
       lcd.print(boostVel2);lcd.print("/");lcd.print(boostVel3);
       lcd.setCursor(9, 1);
       lcd.print("PWM");
       lcd.print(boost3);
       lcd.print("% ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 35;EEPROM.update( 31,boost3 );delay(250);lcd.clear();}

      }

      else if( Estado == 35)
      {
         
          if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 33,boost4 );}
    if (digitalRead(botAbajo) ==LOW && boost4>0){boost4= (boost4 -2);delay(150);} 
    if (digitalRead(botArriba) ==LOW && boost4<100){boost4= (boost4 +2);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Presion 4");
       lcd.setCursor(0, 1);
       lcd.print(boostVel3);lcd.print("/");lcd.print(boostVel4);
       lcd.setCursor(9, 1);
       lcd.print("PWM");
       lcd.print(boost4);
       lcd.print("% ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 36;EEPROM.update( 33,boost4 );delay(250);lcd.clear();}

      }

      else if( Estado == 36)
      {
          
          if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 35,boost5 );}
    if (digitalRead(botAbajo) ==LOW && boost5>0){boost5= (boost5 -2); delay(150);} 
    if (digitalRead(botArriba) ==LOW && boost5<100){boost5= (boost5 +2);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Presion 5");
       lcd.setCursor(0, 1);
       lcd.print(boostVel4);lcd.print("/");lcd.print(boostVel5);
       lcd.setCursor(9, 1);
       lcd.print("PWM");
       lcd.print(boost5);
       lcd.print("% ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 37;EEPROM.update( 35,boost5 );delay(250);lcd.clear();}

      }

      else if( Estado == 37)
      {
        
        if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 37,boostVel1 );}
    if (digitalRead(botAbajo) ==LOW && boostVel1 > 5){boostVel1= (boostVel1 -5);delay(150);} 
    if (digitalRead(botArriba) ==LOW){boostVel1= (boostVel1 +5);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Velocidad 1");
       lcd.setCursor(4, 1);
       lcd.print(boostVel1);
       lcd.print("KM/h ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 38;EEPROM.update( 37,boostVel1 );delay(250);lcd.clear();}

      }

      else if( Estado == 38)
      {
           if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 39,boostVel2 );}
    if (digitalRead(botAbajo) ==LOW && boostVel2>5){boostVel2= (boostVel2 -5);delay(150);} 
    if (digitalRead(botArriba) ==LOW){boostVel2= (boostVel2 +5);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Velocidad 2");
       lcd.setCursor(4, 1);
       lcd.print(boostVel2);
       lcd.print("KM/h ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 39;EEPROM.update( 39,boostVel2 );delay(250);lcd.clear();}

      }

      else if( Estado == 39)
      {
           if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 41,boostVel3 );}
    if (digitalRead(botAbajo) ==LOW && boostVel3>5){boostVel3= (boostVel3 -5); delay(150);} 
    if (digitalRead(botArriba) ==LOW){boostVel3= (boostVel3 +5); delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Velocidad 3");
       lcd.setCursor(4, 1);
       lcd.print(boostVel3);
       lcd.print("KM/h ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 310;EEPROM.update( 41,boostVel3 );delay(250);lcd.clear();}

      }

      else if( Estado == 310)
      {
           if(digitalRead(botIzquierda) ==0){Sig_Estado = 3;EEPROM.update( 43,boostVel4 );}
    if (digitalRead(botAbajo) ==LOW && boostVel4>5){boostVel4= (boostVel4 -5);delay(150);} 
    if (digitalRead(botArriba) ==LOW){boostVel4= (boostVel4 +5);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Velocidad 4");
       lcd.setCursor(4, 1);
       lcd.print(boostVel4);
       lcd.print("KM/h ");
           if(digitalRead(botDerecha) ==LOW){Sig_Estado = 311;EEPROM.update( 43,boostVel4 );delay(250);lcd.clear();}

      }

      else if( Estado == 311)
      {
           if(digitalRead(botIzquierda) ==0 || digitalRead(botDerecha) ==LOW){Sig_Estado = 3;EEPROM.update( 45,boostVel5 );}
    if (digitalRead(botAbajo) ==LOW && boostVel5>5){boostVel5= (boostVel5 -5);delay(150);} 
    if (digitalRead(botArriba) ==LOW){boostVel5= (boostVel5 +5);delay(150);}

    lcd.setCursor(3, 0);
       lcd.print("Velocidad 5");
       lcd.setCursor(4, 1);
       lcd.print(boostVel5);
       lcd.print("KM/h ");
      }

      
  else if( Estado == 4)
  {
    int menu;
      String arrayMenu[] = {"Act/Des Corte","Ancho Corte1","Ancho Corte2","Ancho Corte3","Ancho Corte4","Retardo Rearme","Disparo Alto","Rearme Alto","Disparo Bajo","Rearme Bajo","Acelerador MIN","Sensib Palanca"};
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1;
      else if(menu == 1){Sig_Estado = 41;lcd.clear();}
      else if(menu == 2){Sig_Estado = 42;lcd.clear();}
      else if(menu == 3){Sig_Estado = 43;lcd.clear();}
      else if(menu == 4){Sig_Estado = 44;lcd.clear();}
      else if(menu == 5){Sig_Estado = 45;lcd.clear();} 
      else if(menu == 6){Sig_Estado = 46;lcd.clear();}
      else if(menu == 7){Sig_Estado = 47;lcd.clear();}
      else if(menu == 8){Sig_Estado = 48;lcd.clear();}
      else if(menu == 9){Sig_Estado = 49;lcd.clear();}
      else if(menu == 10){Sig_Estado = 410;lcd.clear();}   
      else if(menu == 11){Sig_Estado = 411;lcd.clear();}
      else if(menu == 12){Sig_Estado = 412;lcd.clear();}
      
  }
      else if( Estado == 41)
      {
        if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;}
    if (digitalRead(botAbajo) ==LOW){contador2 =0;delay(200);} 
    if (digitalRead(botArriba) ==LOW){contador2 =1;delay(200);}
      
        lcd.setCursor(6, 0);
      lcd.print("Corte:");
      if (contador2 == 1){
        lcd.setCursor(4, 1);
        lcd.print("Enabled "); 
        }
      else {
        lcd.setCursor(4, 1);
        lcd.print("Disabled");
        }
        if(digitalRead(botDerecha) ==LOW){Sig_Estado = 42;delay(250);lcd.clear();}
      }
      else if( Estado == 42)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 0,tiempocorte1 );}
    if (digitalRead(botAbajo) ==LOW && tiempocorte1>0){tiempocorte1= (tiempocorte1 -10);delay(150);} 
    if (digitalRead(botArriba) ==LOW && tiempocorte1<1000){tiempocorte1= (tiempocorte1 +10);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Duracion Corte");
       lcd.setCursor(0, 1);
       lcd.print("1>2 Gear");
       lcd.setCursor(9, 1);
       lcd.print(tiempocorte1);
       lcd.print("mS    ");
       if(digitalRead(botDerecha) ==LOW){Sig_Estado = 43;EEPROM.update( 0,tiempocorte1 );delay(250);lcd.clear();}
      }

       else if( Estado == 43)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 2,tiempocorte2 ); }
    if (digitalRead(botAbajo) ==LOW && tiempocorte2>0){tiempocorte2= (tiempocorte2 -10);delay(150);} 
    if (digitalRead(botArriba) ==LOW && tiempocorte2<1000){tiempocorte2= (tiempocorte2 +10);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Duracion Corte");
       lcd.setCursor(0, 1);
       lcd.print("2>3 Gear");
       lcd.setCursor(9, 1);
       lcd.print(tiempocorte2);
       lcd.print("mS    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 44;EEPROM.update( 2,tiempocorte2 ); delay(250);lcd.clear();}

      }

      else if( Estado == 44)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 4,tiempocorte3 );}
    if (digitalRead(botAbajo) ==LOW && tiempocorte3>0){tiempocorte3= (tiempocorte3 -10);delay(150);} 
    if (digitalRead(botArriba) ==LOW && tiempocorte3<1000){tiempocorte3= (tiempocorte3 +10);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Duracion Corte");
       lcd.setCursor(0, 1);
       lcd.print("3>4 Gear");
       lcd.setCursor(9, 1);
       lcd.print(tiempocorte3);
       lcd.print("mS    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 45;EEPROM.update( 4,tiempocorte3 );delay(250);lcd.clear();}

      }
      
      else if( Estado == 45)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 6,tiempocorte4 ); }
    if (digitalRead(botAbajo) ==LOW && tiempocorte4>0){tiempocorte4= (tiempocorte4 -10);delay(150);} 
    if (digitalRead(botArriba) ==LOW && tiempocorte4<1000){tiempocorte4= (tiempocorte4 +10);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Duracion Corte");
       lcd.setCursor(0, 1);
       lcd.print("4>5 Gear");
       lcd.setCursor(9, 1);
       lcd.print(tiempocorte4);
       lcd.print("mS    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 46;EEPROM.update( 6,tiempocorte4 );delay(250);lcd.clear();}

      }

     else if( Estado == 46)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 9,offset ); }
         if (digitalRead(botAbajo) ==LOW && offset>0){offset= (offset -10);delay(150);} 
         if (digitalRead(botArriba) ==LOW && offset<2000){offset= (offset +10);delay(150);}
  
       lcd.setCursor(0, 0);
       lcd.print("Retardo Rearme ");
       lcd.setCursor(5, 1);
      lcd.print(offset);
      lcd.print("mS        ");
             if(digitalRead(botDerecha) ==LOW){Sig_Estado = 47;EEPROM.update( 9,offset );delay(250);lcd.clear();}

      }

     else if( Estado == 47)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 11,TriggerHi );}
    if (digitalRead(botAbajo) ==LOW && TriggerHi>0){TriggerHi= (TriggerHi -10);delay(150);} 
         if (digitalRead(botArriba) ==LOW && TriggerHi<1000){TriggerHi= (TriggerHi +10);delay(150);}
       lcd.setCursor(2, 0);
       lcd.print("Disparo Alto");
       lcd.setCursor(0, 1);
      lcd.print("Encima:   ");
      lcd.print(TriggerHi);
      lcd.print("     ");
             if(digitalRead(botDerecha) ==LOW){Sig_Estado = 48;EEPROM.update( 11,TriggerHi );delay(250);lcd.clear();}

      }

      else if( Estado == 48)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 13,rearmHI );}
    if (digitalRead(botAbajo) ==LOW && rearmHI>0){rearmHI= (rearmHI -10);delay(150);} 
         if (digitalRead(botArriba) ==LOW && rearmHI<1000){rearmHI= (rearmHI +10);delay(150);}
       lcd.setCursor(2, 0);
       lcd.print("Rearme Alto");
       lcd.setCursor(0, 1);
      lcd.print("Debajo:   ");
      lcd.print(rearmHI);
      lcd.print("     ");
             if(digitalRead(botDerecha) ==LOW){Sig_Estado = 49;EEPROM.update( 13,rearmHI );delay(250);lcd.clear();}

      }


      else if( Estado == 49)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 15,TriggerLOW ); }
    if (digitalRead(botAbajo) ==LOW && TriggerLOW>-1000){TriggerLOW= (TriggerLOW -10);delay(150);} 
         if (digitalRead(botArriba) ==LOW && TriggerLOW<0){TriggerLOW= (TriggerLOW +10);delay(150);}
       lcd.setCursor(2, 0);
       lcd.print("Disparo Bajo");
       lcd.setCursor(0, 1);
      lcd.print("Encima:   ");
      lcd.print(TriggerLOW);
      lcd.print("     ");
             if(digitalRead(botDerecha) ==LOW){Sig_Estado = 410;EEPROM.update( 15,TriggerLOW );delay(250);lcd.clear();}

      }

      else if( Estado == 410)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 17,rearmLOW ); }
    if (digitalRead(botAbajo) ==LOW && rearmLOW>-1000){rearmLOW= (rearmLOW -10);delay(150);} 
         if (digitalRead(botArriba) ==LOW && rearmLOW<0){rearmLOW= (rearmLOW +10);delay(150);}
       lcd.setCursor(2, 0);
       lcd.print("Rearme Bajo");
       lcd.setCursor(0, 1);
      lcd.print("Debajo:   ");
      lcd.print(rearmLOW);
      lcd.print("     ");
             if(digitalRead(botDerecha) ==LOW){Sig_Estado = 411;EEPROM.update( 17,rearmLOW ); delay(250);lcd.clear();}

      }

else if( Estado == 411)
      {
         if(digitalRead(botIzquierda) ==0){Sig_Estado = 4;EEPROM.update( 19,setTPS );}
    if (digitalRead(botAbajo) ==LOW && setTPS>-0){setTPS= (setTPS -5);delay(150);} 
         if (digitalRead(botArriba) ==LOW && setTPS<100){setTPS= (setTPS +5);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Acelerador MIN");
       lcd.setCursor(1, 1);
       lcd.print("Position: ");
       lcd.print(setTPS);
       lcd.print("%   ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 412;EEPROM.update( 19,setTPS );delay(250);lcd.clear();}

      }    

else if( Estado == 412)
      {
         if(digitalRead(botIzquierda) ==0 || digitalRead(botDerecha) ==LOW){
          Sig_Estado = 4; 
          EEPROM.update( 73,sensibilidadPalanca );
          lcd.setCursor (0,0);
          lcd.clear();
          lcd.print ("Calibrando...");
          delay(1000);
          calPalanca = analogRead(A9)/sensibilidadPalanca;
          }
    if (digitalRead(botAbajo) ==LOW && sensibilidadPalanca>1){sensibilidadPalanca= (sensibilidadPalanca -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && sensibilidadPalanca<20){sensibilidadPalanca= (sensibilidadPalanca +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Sensib Palanca");
       lcd.setCursor(1, 1);
       lcd.print(sensibilidadPalanca);
       lcd.print("     ");
              
      }    
      
    else if( Estado == 5)
  {
   int menu;
      String arrayMenu[] = {"TPS 0%","TPS 100%"};
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1;
      else if(menu == 1){Sig_Estado = 51;lcd.clear();}
      else if(menu == 2){Sig_Estado = 52;lcd.clear();}
  }

  else if( Estado == 51)
  {
  if(digitalRead(botIzquierda) ==0){Sig_Estado = 5;}
  if (digitalRead(botArriba) ==LOW){Lec0=  lecturaTPS;EEPROM.update( 21,Lec0 ); delay(200);}

        lcd.setCursor(5, 0);
        lcd.print("TPS=0% ");
        lcd.setCursor(3, 1);
        lcd.print(TPSporcentaje);
        lcd.print("%"  );
        lcd.setCursor(9, 1);
        lcd.print("Pres(^)");
  } 

  else if( Estado == 52)
  {
  if(digitalRead(botIzquierda) ==0){Sig_Estado = 5;}
  if (digitalRead(botArriba) ==LOW){Lec1024=  lecturaTPS; EEPROM.update(23,  Lec1024); delay(200);}

        lcd.setCursor(4, 0);
        lcd.print("TPS=100% ");
        lcd.setCursor(3, 1);
        lcd.print(TPSporcentaje);
        lcd.print("%  ");
        lcd.setCursor(9, 1);
        lcd.print("Pres(^)");
  } 

if(Estado == 6)
  {
     
      int menu;
      String arrayMenu[] = {"Diametro Del","Diametro Tras","Dientes Del","Dientes Tras","Calib. Del","Calib. Tras"};
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1;
      else if(menu == 1){Sig_Estado = 61;lcd.clear();}
      else if(menu == 2){Sig_Estado = 62;lcd.clear();}
      else if(menu == 3){Sig_Estado = 63;lcd.clear();}
      else if(menu == 4){Sig_Estado = 64;lcd.clear();}
      else if(menu == 5){Sig_Estado = 65;lcd.clear();}
      else if(menu == 6){Sig_Estado = 66;lcd.clear();}

       
  }
  
if(Estado == 61)
  {
if(digitalRead(botIzquierda) ==0){Sig_Estado = 6;EEPROM.update( 53,diametroruedaDel );}
    if (digitalRead(botAbajo) ==LOW && diametroruedaDel>0){diametroruedaDel= (diametroruedaDel -0.01);delay(150);} 
         if (digitalRead(botArriba) ==LOW && diametroruedaDel<1){diametroruedaDel= (diametroruedaDel +0.01);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Diametro Del");
       lcd.setCursor(5, 1);
       lcd.print(diametroruedaDel);
       lcd.print("m   ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 62;EEPROM.update( 53,diametroruedaDel );delay(250);lcd.clear();}

 }

 if(Estado == 62)
  {
if(digitalRead(botIzquierda) ==0){Sig_Estado = 6;EEPROM.update( 49,diametroruedaTras ); }
    if (digitalRead(botAbajo) ==LOW && diametroruedaTras>0){diametroruedaTras= (diametroruedaTras -0.01);delay(150);} 
         if (digitalRead(botArriba) ==LOW && diametroruedaTras<1){diametroruedaTras= (diametroruedaTras +0.01);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Diametro Tras");
       lcd.setCursor(5, 1);
       lcd.print(diametroruedaTras);
       lcd.print("m   ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 63;EEPROM.update( 49,diametroruedaTras );delay(250);lcd.clear();}

 }

 if(Estado == 63)
  {
if(digitalRead(botIzquierda) ==0){Sig_Estado = 6;EEPROM.update( 61,pprDel ); }
    if (digitalRead(botAbajo) ==LOW && pprDel>0){pprDel= (pprDel -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && pprDel<150){pprDel= (pprDel +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Dientes Del");
       lcd.setCursor(5, 1);
       lcd.print(pprDel);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 64;EEPROM.update( 61,pprDel ); delay(250);lcd.clear();}

       
 } 

  if(Estado == 64)
  {
if(digitalRead(botIzquierda) ==0){Sig_Estado = 6;EEPROM.update( 57,pprTras ); }
    if (digitalRead(botAbajo) ==LOW && pprTras>0){pprTras= (pprTras -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && pprTras<150){pprTras= (pprTras +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Dientes Tras");
       lcd.setCursor(5, 1);
       lcd.print(pprTras);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 65;EEPROM.update( 57,pprTras );delay(250);lcd.clear();}

       
 } 

 if(Estado == 65)
  {
if(digitalRead(botIzquierda) ==0){Sig_Estado = 6;EEPROM.update( 69,ajusteDel ); }
    if (digitalRead(botAbajo) ==LOW && ajusteDel>0){ajusteDel= (ajusteDel -0.1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && ajusteDel<100){ajusteDel= (ajusteDel +0.1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Ajuste Del");
       lcd.setCursor(5, 1);
       lcd.print(ajusteDel);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 66;EEPROM.update( 69,ajusteDel );delay(250);lcd.clear();}

       
 }

  if(Estado == 66)
  {
if(digitalRead(botIzquierda) ==0 || digitalRead(botDerecha) ==LOW){Sig_Estado = 6;EEPROM.update( 65,ajusteTras );}
    if (digitalRead(botAbajo) ==LOW && ajusteTras>0){ajusteTras= (ajusteTras -0.1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && ajusteTras<100){ajusteTras= (ajusteTras +0.1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Ajuste Tras");
       lcd.setCursor(5, 1);
       lcd.print(ajusteTras);lcd.print("    ");

       
 }

 if(Estado == 7)
  {
     
      int menu;
      String arrayMenu[] = {"Act/Des Cont","Destraccion 1","Destraccion 2","Destraccion 3","Destraccion 4","Destraccion 5","Agresividad","VelInicial"};
      
      
      int size = sizeof(arrayMenu) / sizeof(arrayMenu[0]);
      menu = menuANTIFALLOSLENTO(arrayMenu,size); 

      if(menu == -1)Sig_Estado = 1;
      else if(menu == 1){Sig_Estado = 71;lcd.clear();}
      else if(menu == 2){Sig_Estado = 72;lcd.clear();}
      else if(menu == 3){Sig_Estado = 73;lcd.clear();}
      else if(menu == 4){Sig_Estado = 74;lcd.clear();}
      else if(menu == 5){Sig_Estado = 75;lcd.clear();}
      else if(menu == 6){Sig_Estado = 76;lcd.clear();}
      else if(menu == 7){Sig_Estado = 77;lcd.clear();}
      else if(menu == 8){Sig_Estado = 78;lcd.clear();}
       
  }
  
if(Estado == 71)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 105,estadoControlT );}
    if (digitalRead(botAbajo) ==LOW){estadoControlT =0;delay(200);} 
    if (digitalRead(botArriba) ==LOW){estadoControlT =1;delay(200);}
      
        lcd.setCursor(4, 0);
      lcd.print("Control:");
      if (estadoControlT == 1){
        lcd.setCursor(4, 1);
        lcd.print("Enabled "); 
        }
      else {
        lcd.setCursor(4, 1);
        lcd.print("Disabled");
        }
               if(digitalRead(botDerecha) ==LOW){Sig_Estado = 72;EEPROM.update( 105,estadoControlT );delay(250);lcd.clear();}

 }
 if(Estado == 72)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 77,Destraccion1 );}
    if (digitalRead(botAbajo) ==LOW && Destraccion1>0){Destraccion1= (Destraccion1 -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Destraccion1<100){Destraccion1= (Destraccion1 +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Destraccion1");
       lcd.setCursor(5, 1);
       lcd.print(Destraccion1);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 73;EEPROM.update( 77,Destraccion1 );delay(250);lcd.clear();}

 }
if(Estado == 73)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 81,Destraccion2 );}
    if (digitalRead(botAbajo) ==LOW && Destraccion2>0){Destraccion2= (Destraccion2 -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Destraccion2<100){Destraccion2= (Destraccion2 +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Destraccion2");
       lcd.setCursor(5, 1);
       lcd.print(Destraccion2);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 74;EEPROM.update( 81,Destraccion2 );delay(250);lcd.clear();}

 }
if(Estado == 74)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 85,Destraccion3 );}
    if (digitalRead(botAbajo) ==LOW && Destraccion3>0){Destraccion3= (Destraccion3 -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Destraccion3<100){Destraccion3= (Destraccion3 +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Destraccion3");
       lcd.setCursor(5, 1);
       lcd.print(Destraccion3);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 75;EEPROM.update( 85,Destraccion3 );delay(250);lcd.clear();}

 }
if(Estado == 75)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 89,Destraccion4 );}
    if (digitalRead(botAbajo) ==LOW && Destraccion4>0){Destraccion4= (Destraccion4 -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Destraccion4<100){Destraccion4= (Destraccion4 +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Destraccion4");
       lcd.setCursor(5, 1);
       lcd.print(Destraccion4);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 76;EEPROM.update( 89,Destraccion4 );delay(250);lcd.clear();}

 }

if(Estado == 76)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 93,Destraccion5 );}
    if (digitalRead(botAbajo) ==LOW && Destraccion5>0){Destraccion5= (Destraccion5 -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Destraccion5<100){Destraccion5= (Destraccion5 +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Destraccion5");
       lcd.setCursor(5, 1);
       lcd.print(Destraccion5);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 77;EEPROM.update( 93,Destraccion5 );delay(250);lcd.clear();}

 }
if(Estado == 77)
  {
 if(digitalRead(botIzquierda) ==0){Sig_Estado = 7;EEPROM.update( 97,Agresividad );}
    if (digitalRead(botAbajo) ==LOW && Agresividad>0){Agresividad= (Agresividad -5);delay(150);} 
         if (digitalRead(botArriba) ==LOW && Agresividad<500){Agresividad= (Agresividad +5);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Agresividad");
       lcd.setCursor(5, 1);
       lcd.print(Agresividad);lcd.print("    ");
              if(digitalRead(botDerecha) ==LOW){Sig_Estado = 78;EEPROM.update( 97,Agresividad );delay(250);lcd.clear();}

 }
if(Estado == 78)
  {
 if(digitalRead(botIzquierda) ==0 || digitalRead(botDerecha) ==LOW){Sig_Estado = 7;EEPROM.update( 101,VelInicial );}
    if (digitalRead(botAbajo) ==LOW && VelInicial>0){VelInicial= (VelInicial -1);delay(150);} 
         if (digitalRead(botArriba) ==LOW && VelInicial<100){VelInicial= (VelInicial +1);delay(150);}
       lcd.setCursor(0, 0);
       lcd.print("Vel Inicial");
       lcd.setCursor(5, 1);
       lcd.print(VelInicial);lcd.print("    ");

 } 
  //--------------- Boost Control------------------

if (controlador == HIGH ){
if (burnOutset == 0){
if (velocidadTras <= boostVel1 && velocidadTras >1 ){
  PWMBoost = boost1;
  Destraccion = Destraccion1;
  }
  else if (velocidadTras <=boostVel2 && velocidadTras >boostVel1 ){
    PWMBoost = boost2;
    Destraccion = Destraccion2;
    }
else if (velocidadTras <=boostVel3 && velocidadTras >boostVel2 ){
    PWMBoost = boost3;
    Destraccion = Destraccion3;
    }
  else if (velocidadTras <=boostVel4 && velocidadTras >boostVel3 ){
    PWMBoost = boost4;
    Destraccion = Destraccion4;
    }

    else if ( velocidadTras >boostVel4){
    PWMBoost = boost5;
    Destraccion = Destraccion5;
    }
    else  {
    PWMBoost = 0;
    }
  }
  else { PWMBoost = boostBurnOut;}
}

else if (burnOutset == 0){PWMBoost = boostLargada;}
else { PWMBoost = boostBurnOut;}
 
  
    if (TPSporcentaje < setTPS && controlador == HIGH ){
  PWMBoost = 0;
  }

PWMBoostP = PWMBoost; //guardamos el valor en porcentaje para poder loogearlo posteriormente

if(PWMBoost != PWMBoostold){
 PWMBoost = map ( PWMBoost,0,100,0,255);
 analogWrite (salidaBoost,PWMBoost);
 
}
  PWMBoostold = PWMBoostP;
  //---------------- calculo cronometraje
if (Trastick >= Nticks18m && parcial18m == 0){
  parcial18m = millis();
  parcial18m =(parcial18m-tiempoLargada)/1000;
  
  Vparcial18m = velocidadTras;
  
}

if (Trastick >= Nticks100m && parcial100m == 0){
  parcial100m = millis();
  parcial100m =(parcial100m-tiempoLargada)/1000;
  
  Vparcial100m = velocidadTras;
  
}

if (Trastick >= Nticks201m && parcial201m == 0){
  parcial201m = millis();
  parcial201m =(parcial201m -tiempoLargada)/1000;
  
  Vparcial201m = velocidadTras;
  
}

  
 //---- lectura sensor de palanca-------------------
   
   voltageSensorPalanca = analogRead(A9)/sensibilidadPalanca - calPalanca;
   voltageSensorPalanca2 = voltageSensorPalanca;
   
 
 //---- sensa si la palanca fue soltada para rearmar el sistema
 if(voltageSensorPalanca2< rearmHI && voltageSensorPalanca2>rearmLOW ){
   estadoanteriorHI = 0;
      estadoanteriorLOW = 0;
  }

  
 //------------- tiempo muerto despues del cambio------
  if (Rele== true){T_Apagar2=millis()+offset;} 

   if (T_Apagar2>millis()){
    voltageSensorPalanca = 0;
    }
    
//------habilitar y desabilitar corte por cambio-----
if (contador2 == 0){voltageSensorPalanca=0;}


//-------------Lectura de TPS---
 lecturaTPS = analogRead(TPS);
 TPSporcentaje = map (lecturaTPS,Lec0,Lec1024,0,100);
if(TPSporcentaje < setTPS){
  voltageSensorPalanca = 0;
  PWMBoost = 0;
  }
  
//------------pulsador de controlador---------

   controlador = digitalRead (31);
if (controlador == LOW){
   digitalWrite (cortesalida, LOW);
    contadorShift = 0;
    //actSerial = true;
   Trastick =0;
   
    tiempoLargada = millis();
    parcial18m =0;
    parcial100m =0;
    parcial201m =0;
    Vparcial18m =0;
    Vparcial100m =0;
    Vparcial201m =0;

    tLogger = millis()+300000;
  }

 //-------tiempo y activacion de controlador para cambio------  
  else {

   
    if (voltageSensorPalanca > TriggerHi && estadoanteriorHI < rearmHI  || voltageSensorPalanca < TriggerLOW && estadoanteriorLOW > rearmLOW   ){
    T_Apagar=millis()+tiemposhift;
    contadorShift ++; 
    if (contadorShift>6){contadorShift =6;} // evita que el contador se pase de 6
    estadoanteriorHI = TriggerHi;
    estadoanteriorLOW= TriggerLOW;
    
    }
   
  if (T_Apagar>millis()){
   Rele=true;
   digitalWrite (cortesalida, LOW); 
   cortesalidalog =0;
    }

   else  {
    Rele=false;
    digitalWrite (cortesalida,HIGH);
    cortesalidalog =1;
    T_Apagar=0;
        } 
      
    }

 

//--------------------- Control de traccion----------------



  if (estadoControlT == 1){

  if(velocidadTras<VelInicial){          //arranca en el control inicial hasta superar la velocidad inicial
     if (velocidadDel>VelInicial+Destraccion){ // si la velocidad del es mayor a la velocidad inicial mas la aceleracion inicial
         ActivarControldeTraccion();    
    }
    else{digitalPotWrite(7);contadorEstadoCT =0;}
 }

  else if(velocidadDel > velocidadTras+Destraccion){                                       //una ves superada la velocidad inicial empieza el control de trac activo
          ActivarControldeTraccion();
}
  else {digitalPotWrite(7);contadorEstadoCT =0;}

  } 
  
  else {digitalPotWrite(7);}



//--------------Control barra de leds

//if (difdelp < (estrategiadifDelp-5)){tira.setPixelColor(1, 255, 233, 0); tira.show(); }
//else {tira.setPixelColor(1, 0, 0, 0); tira.show();}
//if (difdelp > (estrategiadifDelp-5)&& difdelp < (estrategiadifDelp+5) ){tira.setPixelColor(4, 0, 255, 0); tira.show(); }
//else {tira.setPixelColor(4, 0, 0, 0); tira.show();}
//if (difdelp > (estrategiadifDelp+5)){tira.setPixelColor(7, 255,0, 0); tira.show(); }
//else {tira.setPixelColor(7, 0, 0, 0); tira.show();}
 
 // idealmente la rueda delantera tiene q empezar patiando un 20% a vaja vel e ir incrementado hasta mas del 30% al final de la pista
 
 
 
 
    

  //--- etapas de cambios ----  
  switch (contadorShift){
    case 0:{
      tiemposhift = tiempocorte1;
       break;
      }
    case 1:{
      tiemposhift = tiempocorte2;
       break;
      }
      case 2:{
      tiemposhift = tiempocorte3;
       break;
      }
      case 3:{
      tiemposhift = tiempocorte4;
       break;
      }
      case 4:{
      tiemposhift = 0;
       break;
      }
      case 5:{
      tiemposhift = 0;
       break;
      }
      case 6:{
      tiemposhift = 0;
       break;
      }
    }

  
//----- datos que para data logger----

   if ((tiempoActual - tiempoAntLogger) >= 50){ 
    if(tLogger > millis()&& estadotarjeta == 1 ){
     tira.setPixelColor(0, 120, 40, 140);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();
      archivo = SD.open("Logger.msl", FILE_WRITE);
  archivo.print(millis());
  archivo.print("\t");
  archivo.print(controlador);
  archivo.print("\t"); 
  archivo.print(voltageSensorPalanca2);
  archivo.print("\t"); 
  archivo.print(contadorShift+1);
  archivo.print("\t");
  archivo.print(cortesalidalog);
  archivo.print("\t");
  archivo.print(PWMBoostP);
  archivo.print("\t");
  archivo.print(TPSporcentaje);
  archivo.print("\t");
  archivo.print(contadorEstadoCT);
  archivo.print("\t");
  archivo.print(parcial18m);
  archivo.print("\t");
  archivo.print(parcial100m);
  archivo.print("\t");
  archivo.print(parcial201m);
  archivo.print("\t");
  archivo.print(velocidadTras);
  archivo.print("\t");
  archivo.println(velocidadDel);
  
  archivo.close();
  
  //Serial.println("LOGGER");
  }


  if(tLogger < millis()&& estadotarjeta == 1){
    tira.setPixelColor(0, 0, 255, 0);   // cada pixel en color azul (posicion,R,G,B)
    tira.show();
  }
  
 tiempoAntLogger = tiempoActual;
   }


tiempodeciclo = (millis() - tiempoanteriorciclo);  
 //Serial.println(tiempodeciclo );
tiempoanteriorciclo  = millis();

  //--------------Calculo Velocidad Delantera-----

  
cicloTiempoAnteriorDel = tiempoAnteriorDel; 
  actualMicrosDel = micros();  

  if(actualMicrosDel < cicloTiempoAnteriorDel)
  {
    cicloTiempoAnteriorDel = actualMicrosDel;
  }

  frecSinProcesarDel = 10000000000 /periodoPromedioDel;  // Calculate the frequency using the period between pulses.

  if(periodoEntrePulsosDel > tiempoZeroDel - reboteAdiCeroDel|| actualMicrosDel - cicloTiempoAnteriorDel > tiempoZeroDel - reboteAdiCeroDel)
  {  
    frecSinProcesarDel = 0;  // Set frequency as 0.
    reboteAdiCeroDel = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    reboteAdiCeroDel = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }

  frecRealDel = frecSinProcesarDel / 10000;  


 rpsDel = ((2 * PI) /pprDel) * frecRealDel;                                                // frecuencia angular Rad/s
    velocidadDel = ajusteDel*rpsDel * (diametroruedaDel / 2);


// Suavizado Lectura Velocidad:
  totalDel = totalDel - readingsDel[readIndexDel];  // Advance to the next position in the array.
  readingsDel[readIndexDel] = velocidadDel;  // Takes the value that we are going to smooth.
  totalDel = totalDel + readingsDel[readIndexDel];  // Add the reading to the total.
  readIndexDel = readIndexDel + 1;  // Advance to the next position in the array.

  if (readIndexDel >= numReadingsDel)  // If we're at the end of the array:
  {
    readIndexDel = 0;  // Reset array index.
  }
  
  // Calculate the average:
  averageDel = totalDel / numReadingsDel;  // The average value it's the smoothed result.
velocidadDel = averageDel;


    

 //--------------Calculo Velocidad Tracera-----

 
cicloTiempoAnteriorTras = tiempoAnteriorTras; 
  actualMicrosTras = micros();  

  if(actualMicrosTras < cicloTiempoAnteriorTras)
  {
    cicloTiempoAnteriorTras = actualMicrosTras;
  }

  frecSinProcesarTras = 10000000000 /periodoPromedioTras;  // Calculate the frequency using the period between pulses.

  if(periodoEntrePulsosTras > tiempoZeroTras - reboteAdiCeroTras|| actualMicrosTras - cicloTiempoAnteriorTras > tiempoZeroTras - reboteAdiCeroTras)
  {  
    frecSinProcesarTras = 0;  // Set frequency as 0.
    reboteAdiCeroTras = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    reboteAdiCeroTras = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }

  frecRealTras = frecSinProcesarTras / 10000;  


 rpsTras = ((2 * PI) /pprTras) * frecRealTras;                                                // frecuencia angular Rad/s
    velocidadTras = ajusteTras*rpsTras * (diametroruedaTras / 2);

    // Suavizado Lectura Velocidad:
  totalTras = totalTras - readingsTras[readIndexTras];  // Advance to the next position in the array.
  readingsTras[readIndexTras] = velocidadTras;  // Takes the value that we are going to smooth.
  totalTras = totalTras + readingsTras[readIndexTras];  // Add the reading to the total.
  readIndexTras = readIndexTras + 1;  // Advance to the next position in the array.

  if (readIndexTras >= numReadingsTras)  // If we're at the end of the array:
  {
    readIndexTras = 0;  // Reset array index.
  }
  
  // Calculate the average:
  averageTras = totalTras / numReadingsTras;  // The average value it's the smoothed result.
velocidadTras = averageTras;
  
 if (calinicio == 0 ){
  lcd.setCursor (0,0);
  lcd.clear();
  lcd.print ("Calibrando...");
  delay(1000);
  calPalanca = analogRead(A9)/sensibilidadPalanca;
  calinicio ++;
  }

}

///----------------funciones secundarias-------

void Beep(){
  
    digitalWrite (salidaBeep, HIGH);
    delay (50);
    digitalWrite (salidaBeep, LOW);
  
}
   
void lecturaSdel(){  // funcion interrupciones rueda delantera

  periodoEntrePulsosDel = micros() - tiempoAnteriorDel;  
  tiempoAnteriorDel = micros(); 

  if(pulsosContadorDel >= cantidadLecturaDel)  
  {
   periodoPromedioDel = sumaPeriodoDel / cantidadLecturaDel;  
    pulsosContadorDel = 1;  
    sumaPeriodoDel = periodoEntrePulsosDel;  

    int cantReasigLecturasDel = map(periodoEntrePulsosDel, 40000, 5000, 1, 10);  
    cantReasigLecturasDel = constrain(cantReasigLecturasDel, 1, 10); 
    cantidadLecturaDel = cantReasigLecturasDel; 
  }
  else
  {
    pulsosContadorDel++;  // Increase the counter for amount of readings by 1.
    sumaPeriodoDel = sumaPeriodoDel + periodoEntrePulsosDel;  // Add the periods so later we can average.
  }

}

void lecturaStras(){

  
   Trastick++;
   
    periodoEntrePulsosTras = micros() - tiempoAnteriorTras;  
  tiempoAnteriorTras = micros(); 

  if(pulsosContadorTras >= cantidadLecturaTras)  
  {
   periodoPromedioTras = sumaPeriodoTras / cantidadLecturaTras;  
    pulsosContadorTras = 1;  
    sumaPeriodoTras = periodoEntrePulsosTras;  

    int cantReasigLecturasTras = map(periodoEntrePulsosTras, 40000, 5000, 1, 10);  
    cantReasigLecturasTras = constrain(cantReasigLecturasTras, 1, 10); 
    cantidadLecturaTras = cantReasigLecturasTras; 
  }
  else
  {
    pulsosContadorTras++;  // Increase the counter for amount of readings by 1.
    sumaPeriodoTras = sumaPeriodoTras + periodoEntrePulsosTras;  // Add the periods so later we can average.
  }
}

int menuANTIFALLOSLENTO(String arrayMenu[],int size)
{

  int opcion ;

  //determina en que nivel del menu se encuetra y le asigna a opcion el estado anterior correspondiente
  if (Estado!= 1)opcion =opcionAnteriorSub;
  else opcion =opcionAnterior;opcionAnteriorSub=1;

  int extraOpcion = opcion-1;
   
  //Pinta los 2 primeros del menu 
  lcd.clear();
 for(int x = extraOpcion; x < size && x <= (1+extraOpcion) ; x++) // 
      {
        lcd.setCursor(2,x - extraOpcion);
        lcd.print(arrayMenu[x]);
      }
      //Pintamos el cursor
      lcd.setCursor(0,opcion-1-extraOpcion);
      lcd.print(">");
  delay(500);

  //Si pulsamos el boton derecho sale del bucle
  while(digitalRead(botDerecha) != LOW)
  {
      if (digitalRead(botAbajo)==LOW)
      {
        if(opcion <size)
        {
          opcion ++;
          delay(200);
          //Si sobrepasamos el limite por debajo
      if(opcion < 1 + extraOpcion)
        extraOpcion--;
      //Si sobrepasamos el limite por encima
      if(opcion > 2 + extraOpcion)
        extraOpcion++;

      //Pintamos de nuevo el menu
      lcd.clear();
      for(int x = extraOpcion; x < size && x <= (1+extraOpcion) ; x++) // 
      {
        lcd.setCursor(2,x - extraOpcion);
        lcd.print(arrayMenu[x]);
      }
      //Pintamos el cursor
      lcd.setCursor(0,opcion-1-extraOpcion);
      lcd.print(">");

          
        }
      }
      
      if (digitalRead(botArriba)==LOW)
      {
        if(opcion>1)
        {
          opcion --;
          delay(200);
          //Si sobrepasamos el limite por debajo
      if(opcion < 1 + extraOpcion)
        extraOpcion--;
      //Si sobrepasamos el limite por encima
      if(opcion > 2 + extraOpcion)
        extraOpcion++;

      //Pintamos de nuevo el menu
      lcd.clear();
      for(int x = extraOpcion; x < size && x <= (1+extraOpcion) ; x++) // 
      {
        lcd.setCursor(2,x - extraOpcion);
        lcd.print(arrayMenu[x]);
      }
      //Pintamos el cursor
      lcd.setCursor(0,opcion-1-extraOpcion);
      lcd.print(">");

        }
      }

    //Si pulsamos el boton de atras, salimos del bucle y devolvemos -1
    if(digitalRead (botIzquierda)==LOW)
    {    
      //opcion = -1;
      return -1;  //break
    }    
  }
  if (Estado!= 1)opcionAnteriorSub=opcion;
  else opcionAnterior = opcion;
  delay(300);
  return opcion;
}

void digitalPotWrite(int value) {  //actualiza el valor en el potenciometro digital
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
  
}

void ActivarControldeTraccion(){
  if ((tiempoActual - tiempoAntTraccion) >= Agresividad){   // despues de acionar un control espera un tiempo establecido para dejar reaccionar al control(AGRESIVIDAD)

  switch (contadorEstadoCT){                            // si la accion de control no es suficiente pasa al case siguiente para aumentar la agrecion de control

  case 0:{
    digitalPotWrite(82);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
 case 1:{
    digitalPotWrite(104);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
 case 2:{
    digitalPotWrite(118);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
  case 3:{
    digitalPotWrite(123);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
  case 4:{
    digitalPotWrite(124);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
  case 5:{
    digitalPotWrite(126);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
  case 6:{
    digitalPotWrite(127);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      break;
  }
  case 7:{
    digitalPotWrite(128);
     contadorEstadoCT ++;
      tiempoAntTraccion = tiempoActual;
      
      break;
  }
  
   }
  }
 }
  
 


 //   7 -- 9000 -- 1º
 //  82 -- 3500 -- 20º
 // 104 -- 1873 -- 40º
 // 118 -- 837  -- 60º
 // 123 -- 465  -- 70º
 // 124 -- 388  -- 80º
 // 126 -- 239  -- 90º
 // 127 -- 165  -- 100º
 // 128 -- 91   -- 130º
