
#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>
#include "EmonLib.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  //
#include <SPI.h>

LiquidCrystal_I2C lcd(0x27,16,2);

//Modbus Registers Offsets
const int VAC1 = 100;
const int VAC2 = 104;
const int VAC3 = 108;
const int VAC4 = 112;
const int VAC5 = 116;

const int SENSOR_HREG1 = 200;
const int SENSOR_HREG2 = 204;
const int SENSOR_HREG3 = 208;
const int SENSOR_HREG4 = 212;
const int SENSOR_HREG5 = 216;

//Mapeamento hardware
const int V1 = 13;
const int V2 = 12;
const int V3 = 14;
const int V4 = 27;
const int V5 = 2;

//Pinos do teclado
//int SCL = 22;  //PINO DIGITAL UTILIZADO PELO TERMINAL SCL
int SDO = 21;  //PINO DIGITAL UTILIZADO PELO TERMINAL SDO

int I1 = 33; 
int I2 = 32; 
int I3 = 35; 
int I4 = 34; 
int I5 = 39; 

int Rele1 = 21;       //Emergencia M2
int Rele2 = 22;       //Emergencia M1
int Contat1 = 18;     //Contatora M2
int Contat2 = 19;     //Contatora M1
int Botao = 23;
int ledPin = 2;

int SensCont1 = 25;   //Sensor contagem de ciclos M2
int SensCont2 = 26;   //Sensor contagem de ciclos M1
int SensStop1 = 27;   //Sensor de perda de referencia de giro da maquina 2
int SensStop2 = 14;   //Sensor de perda de referencia de giro da maquina 1

//Variaveis globais para controle dos parametros das máquinas
int Tensao = 1;

double CorrenteLida1;
double CorrenteLida2;
double CorrenteLida3;
double CorrenteLida4;
double CorrenteLida5;

double CorrenteGravada1;
double CorrenteGravada2;
double CorrenteGravada3;
double CorrenteGravada4;
double CorrenteGravada5;

signed int ContT1;    //Contador da Tomada 1
signed int ContT2;    //Contador da Tomada 2
signed int ContT3;    //Contador da Tomada 3
signed int ContT4;    //Contador da Tomada 4
signed int ContT5;    //Contador da Tomada 5

int ControleContagem1 = 1;
int ControleContagem2 = 1;

//Variaveis de contole de status
boolean ledState = false;
boolean ComandoGrava;   //Comando para gravar a corrente inicial das tomadas e começar o monitoramento
boolean inicia;       //variavel para saber se o monitoramento já iniciou ou não
boolean Alarme1;      //Variavel para controle do alarme na maquina 1
boolean Alarme2;      //Variavel para controle do alarme na maquina 2
boolean Controle;

boolean Tomada1;    //Controle para saber se houve alarme na tomada 1
boolean Tomada2;    //Controle para saber se houve alarme na tomada 2
boolean Tomada3;    //Controle para saber se houve alarme na tomada 3
boolean Tomada4;    //Controle para saber se houve alarme na tomada 4
boolean Tomada5;    //Controle para saber se houve alarme na tomada 5

boolean IniciarMaquina;

byte Key;

//Variáveis para login na rede
const char* ssid     = "Taiff - Corp";
const char* password = "T@iff2018@";

/*
// Se for usar IP Fixo, preencha os valores abaixo
IPAddress local_IP(192, 168, 0, 120);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
*/

//ModbusIP object
ModbusIP mb;
EnergyMonitor emon1; 
EnergyMonitor emon2;
EnergyMonitor emon3;
EnergyMonitor emon4;
EnergyMonitor emon5;

long DoisSec;     //Variavel para valor do millis em dois segundos
long MeioSec;     //Variavel para valor do millis em meio segundo


//*******************************************interrupções**********************************************************************
void IRAM_ATTR IncrementaContadorM1_ISR(){      //interrupção do sensor do contador de ciclos da M1
  ControleContagem1 ++;

  if(ControleContagem1 > 2){    //Controle para contar 1 ciclo a cada duas passagens
    ControleContagem1 = 1;
  }
  
  if(ControleContagem1 == 2){   
    if(CorrenteLida1 > 4 && Alarme1 == LOW){    //Incrementa o contador se tiver corrente na tomada 1
      ContT1 ++;
    }
    if(ControleContagem1 == 2){
      if(CorrenteLida2 > 4 && Alarme1 == LOW)   //Incrementa o contador se tiver corrente na tomada 2
        ContT2 ++;
      }
    if(ControleContagem1 == 2){
      if(CorrenteLida3 > 4 && Alarme1 == LOW)   //Incrementa o contador se tiver corrente na tomada 3
        ContT3 ++;
      }
  }
}

void IRAM_ATTR IncrementaContadorM2_ISR(){    //interrupção do sensor do contador de ciclos da M2
  ControleContagem2 ++;
  if(ControleContagem2 > 2){
    ControleContagem2 = 1;
  }
  
  if(ControleContagem2 == 2){
    if(CorrenteLida4 > 4 && Alarme2 == LOW)     //Incrementa o contador se tiver corrente na tomada 4
      ContT4 ++;
    }
    if(ControleContagem2 == 2){
      if(CorrenteLida5 > 4 && Alarme2 == LOW)   //Incrementa o contador se tiver corrente na tomada 5
        ContT5 ++;
      }
}

//***************************************Setup********************************************************

void setup() {
    Serial.begin(115200);

    int count=0;
    //if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    //Serial.println("ERRO! STA Falha na configuração!");
    //}
  
  // Conecta a rede Wifi (se nao for descomentado as linhas acima, o IP será obtido por DHCP) 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); // Aguarda meio segundo
        Serial.print(".");
        count++;
        // Se ele imprimiu 25 pontos, salta a linha e comeca novamente.
        if (count >25) {count=0;Serial.println(".");}
    }

    Serial.println(". TERMINADO!");
    Serial.println(" ");
    Serial.println("Rede WiFi conectada!");  
    Serial.print("ssid: ");
    Serial.println(ssid);
    Serial.print ("password: ");
    Serial.println(password);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    WiFi.printDiag(Serial); // Essa linha joga info sobre a conexão e mostra diagnosticos do WIFI via porta serial.
    Serial.print("Iniciando ModbusIP  \nIP: " );
    Serial.print(WiFi.localIP());
    Serial.println("MAC");
    Serial.println(WiFi.macAddress());

    mb.server();		//Start Modbus IP

    pinMode(V1, INPUT);
    pinMode(V2, INPUT);
    pinMode(V3, INPUT);
    pinMode(V4, INPUT);
    pinMode(V5, INPUT);

    pinMode(SCL, OUTPUT); 
    pinMode(SDO, INPUT);

    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    //mb.addIreg(SENSOR_IREG);
    mb.addHreg(SENSOR_HREG1);
    mb.addHreg(SENSOR_HREG2);
    mb.addHreg(SENSOR_HREG3);
    mb.addHreg(SENSOR_HREG4);
    mb.addHreg(SENSOR_HREG5);

    mb.addIsts(VAC1);
    mb.addIsts(VAC2);
    mb.addIsts(VAC3);
    mb.addIsts(VAC4);
    mb.addIsts(VAC5);

    DoisSec = millis();
    MeioSec = millis();

    emon1.current(I1, 8.1);             // Current: input pin, calibration.
    emon2.current(I2, 8.1);             // Current: input pin, calibration.
    emon3.current(I3, 8.1);             // Current: input pin, calibration.
    emon4.current(I4, 8.1);             // Current: input pin, calibration.
    emon5.current(I5, 8.1);             // Current: input pin, calibration.

    pinMode(Botao, INPUT_PULLUP);
    pinMode(Rele1, OUTPUT);
    pinMode(Rele2, OUTPUT);
    pinMode(Contat1, OUTPUT);
    pinMode(Contat2, OUTPUT);
    pinMode(ledPin, OUTPUT);

    pinMode(SensCont1, INPUT_PULLUP);
    pinMode(SensCont2, INPUT_PULLUP);
    pinMode(SensStop1, INPUT_PULLUP);
    pinMode(SensStop2, INPUT_PULLUP);

    inicia = LOW; 
    Alarme1 = LOW;
    Alarme2 = LOW;
    Controle = LOW;

    Tomada1 = LOW;
    Tomada2 = LOW;
    Tomada3 = LOW;
    Tomada4 = LOW;
    Tomada5 = LOW;

    IniciarMaquina = LOW;

    lcd.init();

    //Interrupções dos sensores de posição das máquinas
    attachInterrupt(SensCont1, IncrementaContadorM1_ISR, RISING);
    attachInterrupt(SensCont2, IncrementaContadorM2_ISR, RISING);
    attachInterrupt(SensStop1, ParaM1, RISING);
    attachInterrupt(SensStop2, ParaM2, RISING);
}
//***************************************Fim do Setup*****************************************************************************


//*******************************************************************************************************************************

//Rotina chamada a cada 2 segundos onde faz a leitura das correntes e envia no modbus, é possível usar para enviar a contagem de ciclos também
void LeCorrente(){
       mb.Ists(VAC1, (Tensao));
       mb.Ists(VAC2, (Tensao));
       mb.Ists(VAC3, (Tensao));
       mb.Ists(VAC4, (Tensao));
       mb.Ists(VAC5, (Tensao));

              //Setting raw value (0-1024)
       /*
       mb.Ists(VAC1, digitalRead(V1));
       mb.Ists(VAC2, digitalRead(V2));
       mb.Ists(VAC3, digitalRead(V3));
       mb.Ists(VAC4, digitalRead(V4));
       mb.Ists(VAC5, digitalRead(V5));
       mb.Ists(VAC6, digitalRead(V6));
       */

       double Corrente1 = emon1.calcIrms(1480) * 10;
       CorrenteLida1 = Corrente1;
       double Corrente2 = emon2.calcIrms(1480) * 10;
       CorrenteLida2 = Corrente2;
       double Corrente3 = emon3.calcIrms(1480) * 10;
       CorrenteLida3 = Corrente3;
       double Corrente4 = emon4.calcIrms(1480) * 10;
       CorrenteLida4 = Corrente4;
       double Corrente5 = emon5.calcIrms(1480) * 10;
       CorrenteLida5 = Corrente5;

       //Corrente = (analogRead(34) * 20) / 4095;
       mb.Hreg(SENSOR_HREG1,Corrente1);
       mb.Hreg(SENSOR_HREG2,Corrente2);
       mb.Hreg(SENSOR_HREG3,Corrente3);
       mb.Hreg(SENSOR_HREG4,Corrente4);
       mb.Hreg(SENSOR_HREG5,Corrente5);
}

//*********************************************************************************************************************
//1° rotina ao energizar a máquina, as contatoras ficam desligadas esperando o usuário dar o Ok para ligar as contatoras.
//Esta rotina foi criada para garantir que na falta de energia os secadores não fiquem ligados apontando para o próprio cabo causando derretimento.

//Testado a parte de temporizador millis MeioSec
void AguardaInicio(){
      digitalWrite(Rele1, HIGH);     //liga relé emergencia M2
      digitalWrite(Rele2, HIGH);     //liga relé emergencia M1
      digitalWrite(Contat1, LOW);    //Desliga contatora M2
      digitalWrite(Contat2, LOW);    //Desliga contatora M1

       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print("Aperte OK p/");
       lcd.setCursor(1,0);
       lcd.print("ligar as tomadas");

       Key = Read_TTP229_Keypad();
       if(Key == 4){
        IniciarMaquina = HIGH;
       }
  
      //Rotina para inverter o estado do led a cada meio segundo.
      if (millis() > MeioSec + 500) {
       MeioSec = millis();

        ledState = !ledState;
        digitalWrite(ledPin, ledState);
      }
}

//*********************************************************************************************************************
//Esta rotina será chamada quando o usuário der Ok no teclado e vai ligar as contatoras e travar o eixo da máquina
//O usuário deve pressionar o botão de emergencia da máquina para posicionar o eixo no local correto.
//O usuário deve também verificar o contador de ciclos e resetar os valores se for conveniente
//Quando estiver na posição ideal o usuário deve pressionar o botão de start na máquina para iniciar o ensaio.

void PosicionaMaquinas(){
      digitalWrite(Rele1, LOW);       //Desliga relé emergencia M2
      digitalWrite(Rele2, LOW);       //Desliga relé emergencia M1
      digitalWrite(Contat1, HIGH);    //liga contatora M2
      digitalWrite(Contat2, HIGH);    //liga contatora M1

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ligar secadores");
      lcd.setCursor(1,0);
      lcd.print("ligar as tomadas");
  
      //Rotina para inverter o estado do led a cada meio segundo.
      if (millis() > MeioSec + 500) {
       MeioSec = millis();

        ledState = !ledState;
        digitalWrite(ledPin, ledState);
      }
}

//************************************************************************************************************************
//Após cinco ciclos em qualquer uma das máquinas essa função será chamada para fazer a gravação dos valores de corrente e calcular a corrente para alarme
void GravaCorrentes(){
      inicia = HIGH;
      CorrenteGravada1 = CorrenteLida1;
      CorrenteGravada1 = CorrenteGravada1 / 4;
      CorrenteGravada2 = CorrenteLida2;
      CorrenteGravada2 = CorrenteGravada2 / 4;
      CorrenteGravada3 = CorrenteLida3;
      CorrenteGravada3 = CorrenteGravada3 / 4;
      CorrenteGravada4 = CorrenteLida4;
      CorrenteGravada4 = CorrenteGravada4 / 4;
      CorrenteGravada5 = CorrenteLida5;
      CorrenteGravada5 = CorrenteGravada5 / 4;
      digitalWrite(ledPin, LOW);
  }

//*************************************************************************************************************************

  void ParaM1(){
    digitalWrite(Rele2, HIGH);  // Aciona emergencia M1
    digitalWrite(ledPin, HIGH); // Liga o led de aviso
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Alarme Maquina 1");

    if(Tomada1 == HIGH){
      lcd.setCursor(1,0);
      lcd.print("Tomada 1");
    }
    if(Tomada2 == HIGH){
      lcd.setCursor(1,0);
      lcd.print("Tomada 2");
    }
    if(Tomada3 == HIGH){
      lcd.setCursor(1,0);
      lcd.print("Tomada 3");
    }
    //delay(60000);
    digitalWrite(Contat2, LOW); // Desliga contatora M1
    //Serial.println("Alarme Acionado");
    WiFi.disconnect();
    delay(1000);
    Controle = HIGH;
  }

//*************************************************************************************************************************

  void ParaM2(){
    digitalWrite(Rele1, HIGH); //Aciona emergencia M2
    digitalWrite(ledPin, HIGH); // Liga o led de aviso
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Alarme Maquina 2");
    if(Tomada4 == HIGH){
      lcd.setCursor(1,0);
      lcd.print("Tomada 4");
    }
    if(Tomada5 == HIGH){
      lcd.setCursor(1,0);
      lcd.print("Tomada 5");
    }
    //delay(60000);
    digitalWrite(Contat1, LOW); //Desliga contatora M2
    //Serial.println("Alarme Acionado");
    WiFi.disconnect();
    delay(1000);
    Controle = HIGH; 
  }

//***********************************Leitura da tecla pressionada********************************************************************************
  
  byte Read_TTP229_Keypad(void)
{
  byte Num;
  byte Key_State = 0;
  for(Num = 1; Num <= 16; Num++)
  {
    digitalWrite(SCL, LOW);
    if (!digitalRead(SDO))
       Key_State = Num;
    digitalWrite(SCL, HIGH);
  } 
  return Key_State;
}

//*******************************************************LOOP*********************************************************

void loop() {

   if (WiFi.status() != WL_CONNECTED){ //Se não estiver conectado no wifi, faz a conexão
      WiFi.begin(ssid, password);
   }

  lcd.setBacklight(HIGH);   //Liga backlight do display

   //Call once inside loop() - all magic here
   mb.task();     //Manda as informações para o supervisório

   //Read each two seconds
   if (millis() > DoisSec + 2000) {   //Atualiza as correntes a cada 2 segundos
       DoisSec = millis();
       LeCorrente();    //Chama rotina para leitura das correntes
   }
   delay(10);

  //Controle de telas do display
   //key = ttp229.ReadKey16();

   if(inicia == LOW && ComandoGrava == LOW && IniciarMaquina == LOW && Alarme1 == LOW && Alarme2 == LOW){  //Aguarda para iniciar novo ciclo
    AguardaInicio();    //Incrementa tela
    }

  // Rotina de inicio para aguardar a leitura das correntes
  if(inicia == LOW && ComandoGrava == LOW && IniciarMaquina == HIGH && Alarme1 == LOW && Alarme2 == LOW){  //Aguarda para iniciar novo ciclo
      PosicionaMaquinas();    //Rotina de aguardar o inicio dos ensaios
    }

      //Rotina de gravação das correntes de referencia de alarme
  if(inicia == LOW && ComandoGrava == HIGH && Alarme1 == LOW && Alarme2 == LOW){ //Quando detectar comando para iniciar os ensaios
      GravaCorrentes();   //Grava a corrente atual e calcula a corrente limite para disparar o alarme.
    }

  if(inicia == HIGH && ComandoGrava == HIGH && Alarme1 == LOW && SensStop1 == LOW){   //Rotina normal durante ensaio
    //Criar rotina para contagem dos ciclos, salvar na memoria externa e mostrar no display

    if(CorrenteLida1 < CorrenteGravada1 && Alarme1 == LOW){
      Tomada1 = HIGH;
    }

    if(CorrenteLida2 < CorrenteGravada2 && Alarme1 == LOW){
      Tomada2 = HIGH;
    }

    if(CorrenteLida3 < CorrenteGravada3 && Alarme1 == LOW){
      Tomada3 = HIGH;
    }
  }

  if(inicia == HIGH && ComandoGrava == HIGH && Alarme2 == LOW && SensStop2 == LOW){   //Rotina normal durante ensaio
    //Criar rotina para contagem dos ciclos, salvar na memoria externa e mostrar no display

    if(CorrenteLida4 < CorrenteGravada4 && Alarme2 == LOW){
      Tomada4 = HIGH;
    }

    if(CorrenteLida5 < CorrenteGravada5 && Alarme2 == LOW){
      Tomada5 = HIGH;
    }
  }

    //Rotinas de verificação de correntes de alarme nas tomadas
  if(inicia == HIGH && Tomada1 == HIGH || Tomada2 == HIGH || Tomada3 == HIGH || SensStop1 == HIGH){   //Se acionar o alarme em qualquer tomada da M1
    Alarme1 = HIGH;   //Aciona o alarme da M1
    ParaM1();
  }

   if(inicia == HIGH && Tomada4 ==HIGH || Tomada5 == HIGH || SensStop2 == HIGH){    //Se acionar o alarme em qualquer tomada da M2
    Alarme2 = HIGH;
    ParaM2();
  } 
  
/*
    //Rotina de alarme na máquina 2
  if(inicia == HIGH && Alarme1 == HIGH && Controle == LOW){
    ParaM1();
  }

    //Rotina de alarme na máquina 1
  if(inicia == HIGH && Alarme2 == HIGH && Controle == LOW){
    ParaM2();
  }
  */
}