/*
 Copyright (C) 2017 Luis Hernan Espinosa Llanos 
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "DHT.h"
#include <OneWire.h>                
#include <DallasTemperature.h>

#define DHT_PIN 7
#define DHTTYPE DHT11 
#define JARDIN_POWER_READING_STATE_PIN 3


//DHT22 class
DHT dht(DHT_PIN, DHTTYPE);

OneWire ourWire(4);                //Se establece el pin 4  como bus OneWire - Temperatura Tierra 1
DallasTemperature sensors(&ourWire); //Se declara una variable  sensor

// Configuracion de Pines
const int electrovalvulaPin = 8; // PIN 8
const int lluviaPin = 6; // PIN DIGITAL 6
unsigned char flujoPin = 5; // PIN DIGITAL 5 
const int humedadTierra1Pin = A0; // PIN ANALOGO 0
const int humedadTierra2Pin = A1; // PIN ANALOGO 1

// Variables
short  temperature= 32767;
unsigned short  humidity  = 32767;
short rain = 32767;
unsigned short literPerMinute = 32767; 
short SoilMoisture1 = 32767;
short SoilMoisture2 = 32767;
float SoilTemperature = 32767;

float litros;

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9,10);

#define UNIT_ID 0xc3


const uint64_t pipes[2] = { 0xc2c2c2c2c3 , 0xe7e7e7e7e7 };              // Radio pipe addresses for the 2 nodes to communicate.


// Set up roles to simplify testing 
boolean role;                                    // The main role variable, holds the current role identifier
boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.
unsigned long Count=0;

volatile int pulsos; // Cantidad de pulsos del sensor. Como se usa dentro de una interrupcion debe ser volatile 
unsigned int litrosPorHora; // Calculated litres/hour
//unsigned int litrosPorMinuto; // Calculated litres/h 
unsigned long tiempoAnterior; // Para calcular el tiempo 
unsigned long pulsosAcumulados; // Pulsos acumulados


void flujo (){ // Funcion de interrupcion
  pulsos++; // Simplemente sumar el numero de pulsos
}


void setup() {

  dht.begin();

  // Set pin for Garden Reading state power
  pinMode(JARDIN_POWER_READING_STATE_PIN, OUTPUT);
  digitalWrite(JARDIN_POWER_READING_STATE_PIN, LOW);

  pinMode(electrovalvulaPin, OUTPUT);
  pinMode(lluviaPin, INPUT);
  pinMode(flujoPin, INPUT);

  attachInterrupt(0, flujo, RISING); // Setup Interrupt
  interrupts(); // Habilitar interrupciones
  tiempoAnterior = millis();
  sensors.begin();   //Se inicia el sensor de humedad en tierra
  
  Serial.begin(57600);
  printf_begin();
  printf("---- Nodo Jardin 1 Sistema de Domotica ----");
  
  // Setup and configure rf radio
  //radio.setChannel(0x60);
  radio.begin();                          // Start up the radio
  radio.setPayloadSize(32);
  radio.setChannel(0x4e);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(15,15);   // Max delay between retries & number of retries
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  //role = role_ping_out;                  // Become the primary transmitter (ping out)
  radio.openWritingPipe(pipes[1]);
   radio.openReadingPipe(1,pipes[0]);
    
  radio.startListening();                 // Start listening
  radio.stopListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();
}


enum cycleMode {ModeInit,ModeListen,ModeWriteData,ModeWait};

cycleMode cycle= ModeInit;


#define STRUCT_TYPE_GETDATA    0
#define STRUCT_TYPE_INIT_DATA  1
#define STRUCT_TYPE_GARDEN_DATA 2
#define STRUCT_TYPE_SENDCOMMAND    3


typedef struct
{
  char header;
  unsigned char structSize;
  unsigned char structType;
  unsigned char txmUnitId;
  unsigned long currentTime;
  unsigned short nextTimeReading;
  char Spare[22];
}RcvPacketStruct;


typedef struct
{
   char header;
   unsigned char structSize;
   unsigned char structType;
   unsigned char txmUnitId;
   unsigned long  stampTime;
   unsigned char  valid;
   unsigned short voltageA2D;
   unsigned short temperature;
   unsigned short humidity;
   unsigned short rain;
   unsigned short literPerMinute;
   unsigned short SoilMoisture1;
   unsigned short SoilMoisture2;
   unsigned short SoilTemperature;
}TxmGardenPacketStruct;


unsigned long currentDelay;
unsigned long targetDelay;

RcvPacketStruct RcvData;
TxmGardenPacketStruct Txmdata;

unsigned char * pt = (unsigned char *) &RcvData;


unsigned char rcvBuffer[32];


void PrintHex(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       char tmp[32];
       for (int i=0; i<length; i++)
       { 
         sprintf(tmp, "0x%.2X",data[i]); 
         Serial.print(tmp); Serial.print(" ");
       }
}

bool readSensor(void)
{
  /**
  // power Sensor UP
  digitalWrite(JARDIN_POWER_READING_STATE_PIN, HIGH);
  // Wait 2 sec
  delay(2000);
  
  // Now let's read the sensor twice
  // since the first one will be bad
  
  DHT.read(DHT_PIN);
  delay(1000);
  int rcode = DHT.read(DHT_PIN);
  
  // power off DHT22
  
  digitalWrite(JARDIN_POWER_READING_STATE_PIN,LOW);
  
  return (rcode == DHTLIB_OK);
  **/
}     



//From http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
unsigned short readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned short) result; // Vcc in millivolts
}


void loop(void){

  // Cada segundo calcular e imprimir Litros/seg
    if( millis() - tiempoAnterior > 1000) {
      tiempoAnterior = millis(); // Updates cloopTime
      // Pulse frequency (Hz) = 6.67 Q, Q is flow rate in L/min. (Results in +/- 3% range)
      // Q = frecuencia / 6.67 (L/min)
      // Q = (frecuencia * 60) / 6.67 (L/hora)
      pulsosAcumulados += pulsos;
      litrosPorHora = (pulsos * 60 / 6.67); // (Pulse frequency x 60 min) / 7.5Q = flow rate in L/hour
      literPerMinute = (pulsos / 6.67);
      pulsos = 0; // Reset Counter
      litros = pulsosAcumulados*1.0/400; //Cada 400 pulsos = 1 litro   
    }
  
  int loop;
  unsigned long deltaTime;  
  /****************** Ping Out Role ***************************/
    
    if(cycle==ModeInit)
     {
       Count++;
       Txmdata.header='*';
       Txmdata.structSize= sizeof(TxmGardenPacketStruct);
       Txmdata.structType=STRUCT_TYPE_INIT_DATA;
       Txmdata.txmUnitId = UNIT_ID;
       Txmdata.stampTime=0;
       Txmdata.valid=0;
       Txmdata.temperature=0;
       Txmdata.humidity=0;
       Txmdata.rain=0;
       Txmdata.literPerMinute=0;
       Txmdata.SoilMoisture1=0;
       Txmdata.SoilMoisture2=0;
       Txmdata.SoilTemperature=0;       
       Txmdata.voltageA2D=0;
       radio.writeAckPayload(1,&Txmdata,sizeof(TxmGardenPacketStruct));
       cycle=ModeListen;
     }
     
    if(cycle==ModeListen)
     {
        if(radio.available()) 
        {
          int rcv_size= radio.getDynamicPayloadSize();
          radio.read( &RcvData,rcv_size);

          unsigned char tipo = RcvData.structType;
          Serial.println(tipo);

          //if(strcmp(RcvData.structType, "STRUCT_TYPE_GETDATA") == 0){
          if(tipo == STRUCT_TYPE_GETDATA){  
            Serial.println("GETTING DATA");
            Serial.print("T:" );
            Serial.print(RcvData.currentTime);
            Serial.print(" Next reading in (1/10 sec): ");
            Serial.print(RcvData.nextTimeReading);
            Serial.print("\n");
            PrintHex(pt,rcv_size);
            Serial.print("\n");        
            currentDelay = millis();
          }
          
          //else if(strcmp(RcvData.structType, "STRUCT_TYPE_SENDCOMMAND") == 0){
          else if(tipo == STRUCT_TYPE_SENDCOMMAND){
            // Se revisa el comando recibido (Vacio o Existente)           
            Serial.println("SENDING DATA");
            if(strlen(RcvData.Spare) == 0){
              Serial.println("Sin Comando");
            }else{
              Serial.print(" Comando: ");
              Serial.println(RcvData.Spare);

              if(strcmp(RcvData.Spare, "ELVON") == 0){
                Serial.println("Encendiendo Electrovalvula!");
                digitalWrite(electrovalvulaPin, HIGH);
              } 

              if(strcmp(RcvData.Spare, "ELVOF") == 0){
                Serial.println("Apagando Electrovalvula!");
                digitalWrite(electrovalvulaPin, LOW);
              }              
            }
          }
          
                          
          if(RcvData.nextTimeReading > 50){
            targetDelay =  RcvData.nextTimeReading*100 - 5000UL;
            cycle = ModeWait;
          }else{
            cycle = ModeWriteData;
          }                     
        }
     }
     
     
     if(cycle== ModeWait)
       {
         
         deltaTime = millis() - currentDelay;
        
         if(deltaTime >= targetDelay)
            cycle = ModeWriteData;
         
       }
     
     
     if(cycle==ModeWriteData)
      {
       Txmdata.valid=1;
       Txmdata.stampTime=RcvData.currentTime;
       Txmdata.header='*';
       Txmdata.structSize= sizeof(TxmGardenPacketStruct);
       Txmdata.structType=STRUCT_TYPE_GARDEN_DATA;
       Txmdata.txmUnitId = UNIT_ID;
       Txmdata.stampTime=RcvData.currentTime + (deltaTime / 1000);
       Txmdata.valid=0;
       Txmdata.temperature=32767;
       Txmdata.humidity=32767;
       Txmdata.rain=32767;
       Txmdata.literPerMinute=32767;
       Txmdata.SoilMoisture1=32767;
       Txmdata.SoilMoisture2=32767;
       Txmdata.SoilTemperature=32767;       
       Txmdata.voltageA2D=readVcc();

        // length of the structure
        //int len_struct = sizeof(TxmGardenPacketStruct);
        //Serial.println("STRUCT SIZE 1");
        //Serial.print(len_struct);

        
                
       //if(readSensor())
       if(true)
         {
           //Txmdata.temperature = (short) floor(DHT.temperature * 10.0);
           //Txmdata.humidity = (short) floor(DHT.humidity);
           Txmdata.temperature = (short) floor(dht.readTemperature() * 10.0);
           Txmdata.humidity = (short) floor(dht.readHumidity());

           //Cambiar valor de lluvia (SI:1 - NO=0)
           int llvalue = 0;
           llvalue = digitalRead(lluviaPin);
           if(llvalue == LOW){
              // LLUVIA DETECTADA
              llvalue = 1;
           }else{
              // NO HAY LLUVIA 
              llvalue = 0;
              
           }

           Txmdata.rain = (short) floor(llvalue);
           //Txmdata.rain = (short) floor(digitalRead(lluviaPin));
           Txmdata.literPerMinute = (short) floor(literPerMinute);
           Txmdata.SoilMoisture1 = (short) floor(analogRead(humedadTierra1Pin));
           Txmdata.SoilMoisture2 = (short) floor(analogRead(humedadTierra2Pin));
           sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
           Txmdata.SoilTemperature = (short) floor(sensors.getTempCByIndex(0));
           Txmdata.valid = 1;
         }
       radio.writeAckPayload(1,&Txmdata,sizeof(TxmGardenPacketStruct));
       cycle=ModeListen;     
      }
      
}
