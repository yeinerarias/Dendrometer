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
//Biblioteca para el sensor infrarrojo
#include <SharpIR.h>

//Parámetros de la red de comunicación y protocolos SPI I2C
#define NETWORKID     50  // The same on all nodes that talk to each other
#define NODEID        10    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "IrazuDendrometer" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define SERIAL_BAUD   115200

#define DS3231_I2C_ADDRESS 0x68

#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0     // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define TRIES         3     // Numbers of intending to send
#define WAITING_TIME  400   // Request waiting time
#define BUFF_MAX 256

//Variables para la medición del diámetro del tronco
float DiametroAnillo=203;
float Ajuste=-10.537;

//Variables para establecer la comunicación
bool RecibidoCorrectamente=false;
String lectura;
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
int pinA2=2;
int n=0;
int m=0;
float prom=0;
int humedad=0;
float promedio=0;
bool dia=true;

SharpIR sharpR(pinA0, 1, 93, 430); 
SharpIR sharpL(pinA1, 1, 93, 430); 

//Variables para establecer la alarma
uint8_t wake_HOUR;
uint8_t wake_MINUTE;
uint8_t wake_SECOND;

uint8_t alarm_HOUR1 = 17;
uint8_t alarm_MINUTE1 = 30;
uint8_t alarm_SECOND1 = 00;

uint8_t alarm_HOUR2 = 04;
uint8_t alarm_MINUTE2 = 30;
uint8_t alarm_SECOND2 = 00;

bool enable_alarm = false;

bool type_alarm = true;

int sleep_period[10] = {0, 0, 0};

//Lectura del sensor infrarrojo
void SensorIR() {
  float dataR, media_fR = 0;
  int triesR;
  float sumR = 0, mediaR;
  float dataL, media_fL = 0;
  int triesL;
  float sumL = 0, mediaL;

  //lectura = "L=" + String(sharpL.distance());/////////////////////
  for(int j=0; j<70; j++){
    sumR += dataR = sharpR.distance();
    if(j > 24){
      triesR = 0;
      while(abs(mediaR-dataR) > 5 && triesR < 4) {
        delay(20);
        dataR = sharpR.distance();
        triesR ++;
      }
      media_fR += dataR;
    }
    else if(j == 24){
      mediaR = sumR/25;
    }

    sumL += dataL = sharpL.distance();
    if(j > 24){
      triesL = 0;
      while(abs(mediaL-dataL) > 5 && triesL < 4) {
        delay(20);
        dataL = sharpL.distance();
        triesL ++;
      }
      media_fL += dataL;
    }
    else if(j == 24){
      mediaL = sumL/25;
    }
    
    delay(20);
  }

  lectura = "R=";
  media_fR = media_fR/45;
  if(media_fR>679.3) lectura+=String(37039*pow(media_fR,-1.351),1);
  else if(media_fR>542.5) lectura+=String(5548*pow(media_fR,-1.06),1);
  else if(media_fR>409) lectura+=String(6000.4*pow(media_fR,-1.073),1);
  else if(media_fR>321) lectura+=String(2848.9*pow(media_fR,-0.948),1);
  else lectura+=String(6784.9*pow(media_fR,-1.097),1);

  lectura += " L=";
  media_fL = media_fL/45;
  if(media_fL>679.3) lectura+=String(37039*pow(media_fL,-1.351),1);
  else if(media_fL>542.5) lectura+=String(5548*pow(media_fL,-1.06),1);
  else if(media_fL>409) lectura+=String(6000.4*pow(media_fL,-1.073),1);
  else if(media_fL>321) lectura+=String(2848.9*pow(media_fL,-0.948),1);
  else lectura+=String(6784.9*pow(media_fL,-1.097),1);
}

//Lectura del sensor de humedad

//Lectura del sensor de humedad DOS
void SensorHum() {
  humedad=analogRead(pinA2);
  terminado= true;
  lectura+=" H=";
  lectura+=String(humedad);//Concatena los datos en una variable
}

void AlarmaRTC_Fija(void)
{
    uint8_t flags[5] = { sleep_period[0], sleep_period[1], sleep_period[2], 1, 1 };

    //Configuración de la alarma
    DS3231_set_a1(wake_SECOND, wake_MINUTE, wake_HOUR, 0, flags);

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
  analogReference(EXTERNAL);
  //Serial.begin(SERIAL_BAUD);
  //To reduce power, setup all pins as inputs with no pullups
  for(int x = 1 ; x < 7 ; x++){
    pinMode(x, INPUT);
    digitalWrite(x, LOW);
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  pinMode(PWRRFM, OUTPUT);
  digitalWrite(PWRRFM, LOW);
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

  DS3231_init(DS3231_INTCN);
  DS3231_clear_a1f();
  
  String sys_data;
  if(enviar("set_date"))
  {
    sys_data = recibir();
    // DS3231 seg, min, hora, día, fecha, mes, año
    EscribirRTC(sys_data.substring(0,2).toInt(),sys_data.substring(2,4).toInt(),
                sys_data.substring(4,6).toInt(),sys_data.substring(6,8).toInt(),
                sys_data.substring(8,10).toInt(),sys_data.substring(10,12).toInt(),
                sys_data.substring(12).toInt());
  }
  delay(500);
  if(enviar("set_alarm"))
  {
    sys_data = recibir();
    alarm_HOUR1 = sys_data.substring(0,2).toInt();
    alarm_MINUTE1 = sys_data.substring(2,4).toInt();
    alarm_SECOND1 = sys_data.substring(4,6).toInt();
    
    alarm_HOUR2 = sys_data.substring(6,8).toInt();
    alarm_MINUTE2 = sys_data.substring(8,10).toInt();
    alarm_SECOND2 = sys_data.substring(10,12).toInt();
    
    sleep_period[0] = sys_data.substring(12,13).toInt();
    sleep_period[1] = sys_data.substring(13,14).toInt();
    sleep_period[2] = sys_data.substring(14,15).toInt();
    enable_alarm = sys_data.substring(15).toInt();
  }
  digitalWrite(PWRRTC, HIGH);
}

//Envío de datos
bool enviar(String data){
  int str_len = data.length() + 1; 
  char radiopacket[str_len];
  String answer = "";
  data.toCharArray(radiopacket,str_len);
  uint32_t sentTime;
  for (uint8_t i = 0; i <= TRIES; i++)
  {
    radio.send(RECEIVER, radiopacket, strlen(radiopacket));
    sentTime = millis();
    while (millis() - sentTime < WAITING_TIME)
    {
      if (radio.receiveDone()) {
        answer = (char*)radio.DATA; 
        if(answer == "received") {return true;}
        else if(answer == "reset") {delay(1000); asm("jmp 0x0000");}
      }
    }
  }
  return false;
}

//Recepción de datos
String recibir(){
  uint32_t sentTime;
  sentTime = millis();
  while (millis() - sentTime < 2000)
  {
    if (radio.receiveDone())
    {
      radio.sendACK();
      return (char*)radio.DATA;
    }
  }
  return "";
}

String LeerRTC(){
  char in;
  char buff[BUFF_MAX];
  struct ts t;
  
  DS3231_get(&t);

  // there is a compile time option in the library to include unixtime support
  #ifdef CONFIG_UNIXTIME
          snprintf(buff, BUFF_MAX, "%d/%02d/%02d %02d:%02d:%02d %ld", t.year,
               t.mon, t.mday, t.hour, t.min, t.sec, t.unixtime);
  #else
          snprintf(buff, BUFF_MAX, "%d/%02d/%02d %02d:%02d:%02d", t.year,
               t.mon, t.mday, t.hour, t.min, t.sec);
  #endif
  return buff;
}


//Interrupción creada mediante el RTC
void pin3Interrupt(void){
// Desconecta el interrupt para que no se vuelva a despertar
  detachInterrupt(1);
}

//Modo de bajo consumo
void ModoSleep(void){

  DS3231_clear_a1f();
  //Incluido para activar el RTC y reescribir alarma
  //Trabajar con dos alarmas a diferentes horas
  //Alarma 1
  if (dia==true){
    // Hora para establecer la alarma
    wake_HOUR = alarm_HOUR1;
    wake_MINUTE = alarm_MINUTE1;
    wake_SECOND = alarm_SECOND1;
    dia=false;
  }
  //Alarma 2
  else{
    wake_HOUR = alarm_HOUR2;
    wake_MINUTE = alarm_MINUTE2;
    wake_SECOND = alarm_SECOND2;
    dia=true;
  }
  AlarmaRTC_Fija();  
  //radio.sleep();
  digitalWrite(PWRRTC, HIGH);
  Wire.end(); 
  digitalWrite(18,LOW);
  digitalWrite(19,LOW);
  noInterrupts ();

  //Pin 3 se asigna como el receptor del interrupt
  attachInterrupt(digitalPinToInterrupt(pin3), pin3Interrupt, LOW);
  interrupts();
  sleep_mode();
  // Aquí se duerme...y aquí continuará al recibir el interrupt
  delay(100);
}
//int transmissions = 0;
void loop()
{
  //transmissions++;
  //Activa sensor de distancia
  digitalWrite(PWRIR, LOW);
  delay(100);
  SensorIR();
  SensorHum();                       
  digitalWrite(PWRIR, HIGH);
  Wire.begin(); 
  digitalWrite(PWRRTC, LOW); // Cambiar
  digitalWrite(PWRRFM, LOW);
  delay(25);
  digitalWrite(RFM69_RST, HIGH);
  delay(25);
  digitalWrite(RFM69_RST, LOW);
  delay(25);
  String fecha = LeerRTC();

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower();
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
  
  enviar(lectura+" "+fecha);
 
  digitalWrite(PWRRFM, HIGH);
  //Serial.print(fecha+ " "+ transmissions);
  if(enable_alarm) ModoSleep();
}
