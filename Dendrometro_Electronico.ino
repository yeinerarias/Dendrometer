//Librerías para la comunicación con el módulo RFM69
#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_ATC.h>
#include <SPI.h>
//Librerías para la comunicación con el módulo de reloj
#include <Wire.h>
#include <ds3231.h>
//Librerías para activar el modo de bajo consumo
#include <avr/sleep.h>
#include <avr/power.h>

//Parámetros de la red de comunicación y protocolos SPI I2C
#define NETWORKID     55  // The same on all nodes that talk to each other
#define NODEID        9    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define SERIAL_BAUD   9600

#define DS3231_I2C_ADDRESS 0x68

#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9

//Variables para la medición del diámetro del tronco
float DiametroAnillo=203;
float Ajuste=-10.537;

//Variables para establecer la comunicación
bool RecibidoCorrectamente=false;
char radiopacket[20];
String lectura="QWERTY";
int16_t packetnum = 0;  // packet counter, we increment per xmission
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

//Pin para la interrupción
int pin3 = 3;

//Pines para el control de alimentación mediante power gating
int PWRRTC = 4; //Activar RTC
int PWRIR = 7;  //Activar Sensor infrarrojo
int PWRRFM = 8; //Activar RFM69
int PWRH = 6;   //Activar Humedad

//Variables para lectura del sensor infrarrojo y el Filtro de Kalman
float zk;
float Kk;
float xk;
float Pk;
float xkantes;
float Pkantes;
float R=0.06150844;
int Po=1;
int cont=0;
bool valorfinal=false;
bool finalizado=false;
bool terminado=false;
float distancia;
float i;
float val;
int pinA0=0;
int pinA1=1;
int n=0;
int m=0;
float prom=0;
float humedad=0;
float promedio=0;
bool dia=true;

//Variables para establecer la alarma
uint8_t Hora;
uint8_t Minuto;
uint8_t Segundo;

//Algoritmo para realizar el filtro de Kalman (Elimina el ruido de la señal)
float Kalman(){
  while(!valorfinal){
    i=analogRead(pinA0);
    zk=i*0.003852539;
    if (cont==0){
      xkantes=zk;
      Kk=(Po/(Po+R));
      xk=xkantes+(Kk*(zk-xkantes));
      //dato=MediaMovil(xk);
      Pk=(1-Kk)*Po;
      cont++;
    }
    else if(cont>0 && Pk>0.001){
      xkantes=xk;
      Pkantes=Pk;
      Kk=(Pkantes/(Pkantes+R));
      xk=xkantes+(Kk*(zk-xkantes));
      //dato=MediaMovil(xk);
      Pk=(1-Kk)*Pkantes;
      cont++;
    }
    else{
      //Serial.println(cont);
      cont=0;
      valorfinal=true;
    }
    delay(100);
  }
  valorfinal=false;
  return xk;
}

//Lectura del sensor infrarrojo
void SensorIR() {
  while (!finalizado){
    if (n<10){
      val=Kalman();
      distancia=-9.3065*(val*val*val)+82.686*(val*val)-279.37*val+419.09;//Ecuación del modelo obtenido
      prom+=distancia;
      n++;
    }
    else{
      prom=prom/10;
      n=0;
      finalizado= true;
      lectura="D: ";
      prom=(DiametroAnillo-prom)+Ajuste;//(anillo - medición) + ajuste
      lectura+=String(prom,3);//Concatena los datos en una variable
      prom=0;
    }
    delay(500);
  }
  finalizado=false;
}

//Lectura del sensor de humedad
void SensorHum() {
  while (!terminado){
    if (m<10){
      //val=Kalman();
      //distancia=lineal(val);
      humedad=analogRead(pinA1);
      humedad=humedad*0.003852539;
      promedio+=humedad;
      m++;
    }
    else{
      promedio=promedio/10;
      if(promedio>3.14){
        promedio=0;
      }
      else if(promedio<3.14 && promedio>1.282){
        promedio=12.691*(promedio*promedio)-104.76*promedio+207.26;
      }
      else{
        promedio=100;
      }
      m=0;
      terminado= true;
      lectura+="\tH: ";
      lectura+=String(promedio,3);//Concatena los datos en una variable
      //Serial.println(prom);//aquí añado el valor al radiopacket
      promedio=0;
    }
    delay(500);
  }
  terminado=false;
}

void AlarmaRTC(void)
{
    // flags para definir cuales elementos deben ser tomados en cuenta al realizar la comparación de la alarma
    // A1M1 (segundos) (0=enable, 1=disable)
    // A1M2 (minutos) (0=enable, 1=disable)
    // A1M3 (horas)    (0=enable, 1=disable) 
    // A1M4 (día)     (0=enable, 1=disable)
    // DY/DT          (díasemana == 1/díames == 0)
    uint8_t flags[5] = { 0, 0, 1, 1, 1 };

    //Configuración de la alarma
    DS3231_set_a1(Segundo, Minuto, Hora, 0, flags);

    //Activar Alarma 1
    DS3231_set_creg(DS3231_INTCN | DS3231_A1IE);
}

//Funciones utilizadas para escribir en el RTC
// Convertir decimal a BCD
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convertir BCD a decimal
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void EscribirRTC(byte seg, byte minuto, byte hora, byte diaSemana, byte
diaMes, byte mes, byte ano)
{
  // Establecer hora y fecha en el RTC
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // Establece el inicio en el registro de segundos para la próxima entrada
  Wire.write(decToBcd(seg)); // Establece los segundos
  Wire.write(decToBcd(minuto)); // Establece los minutos
  Wire.write(decToBcd(hora)); // Establece las horas
  Wire.write(decToBcd(diaSemana)); // Establece día de la semana (1=Domingo, 7=Sabado)
  Wire.write(decToBcd(diaMes)); // Establece la fecha (1 a 31)
  Wire.write(decToBcd(mes)); // Establece el mes
  Wire.write(decToBcd(ano)); // Establece el año (0 a 99)
  Wire.endTransmission();
}

void setup()
{
  //To reduce power, setup all pins as inputs with no pullups
  for(int x = 1 ; x < 7 ; x++){
    pinMode(x, INPUT);
    digitalWrite(x, LOW);
  }

  pinMode(PWRRFM, OUTPUT);
  //digitalWrite(PWRRFM,HIGH);
  delay(100);

  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_enable();
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
 
  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower();
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);

  pinMode(PWRIR, OUTPUT);
  pinMode(PWRH, OUTPUT);
  digitalWrite(PWRH, HIGH);
  delay(100);
  pinMode(PWRRTC, OUTPUT);
  digitalWrite(PWRRTC, LOW);
  delay(100);
  Wire.begin(); 
  pinMode(pin3, INPUT);
  //DS3231_init(DS3231_INTCN);
  //DS3231_clear_a1f();
  // Establecer la hora inicial aquí
  // DS3231 seg, min, hora, día, fecha, mes, año
  //EscribirRTC(40,59,12,3,23,11,16);
  delay(100);
  digitalWrite(PWRRTC, LOW);
  delay(100);
  digitalWrite(PWRRFM,HIGH);
  delay(100);
}

//Envío de datos
void enviar(){
  while (!(RecibidoCorrectamente)){
    delay(2000);
    lectura.toCharArray(radiopacket,20);
    if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket), 4, 2000)) { //target node Id, message as string or byte array, message length
      RecibidoCorrectamente=true;
    }
  }
  RecibidoCorrectamente=false;

}

//Interrupción creada mediante el RTC
void pin3Interrupt(void){
// Desconecta el interrupt para que no se vuelva a despertar
  detachInterrupt(1);
}

//Modo de bajo consumo
void ModoSleep(void){
  //Incluido para activar el RTC y reescribir alarma
  digitalWrite(PWRRTC, LOW);
  delay(100);
  DS3231_clear_a1f();

  //Trabajar con dos alarmas a diferentes horas
  //Alarma 1
  if (dia==true){
    // Hora para establecer la alarma
    Hora = 13;
    Minuto = 00;
    Segundo = 00;
    dia=false;
  }
  //Alarma 2
  else{
    Hora = 13;
    Minuto = 30;
    Segundo = 00;
    dia=true;
  }
  AlarmaRTC();
  delay(100);
  radio.sleep();
  digitalWrite(PWRRTC, HIGH);
  delay(100);
  
  noInterrupts ();

  //Pin 3 se asigna como el receptor del interrupt
  attachInterrupt(digitalPinToInterrupt(pin3), pin3Interrupt, LOW);
  interrupts();
  sleep_mode();
  // Aquí se duerme...y aquí continuará al recibir el interrupt
  delay(100);
}

void loop()
{
  delay(2000);
  //ModoSleep();
  //Activa sensor de distancia
  digitalWrite(PWRIR, LOW);
  delay(100);
  //SensorIR();                         
  delay(100);
  digitalWrite(PWRIR, HIGH);
  delay(100);
  //Activa sensor de humedad
  digitalWrite(PWRH, LOW);
  delay(100);
  //SensorHum();                         
  delay(100);
  digitalWrite(PWRH, HIGH);
  delay(100);
  //Envía los datos
  enviar();
  delay(1000);
}
