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
#include <Preferences.h>
#include "nvs_flash.h"

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
int SDO = 21;  //PINO DIGITAL UTILIZADO PELO TERMINAL SDO

const int I1 = 33; 
const int I2 = 32; 
const int I3 = 35; 
const int I4 = 34; 
const int I5 = 39; 

const int Rele1 = 21;       //Emergencia M2
const int Rele2 = 22;       //Emergencia M1
const int Contat1 = 18;     //Contatora M2
const int Contat2 = 19;     //Contatora M1

const int ledPin = 2;

const int SensCont1 = 26;   //Sensor contagem de ciclos M2
const int SensCont2 = 25;   //Sensor contagem de ciclos M1

const int SensStop1 = 14;   //Sensor de perda de referencia de giro da maquina 1
const int SensStop2 = 27;   //Sensor de perda de referencia de giro da maquina 2

const int StartM1 = 13;
const int StartM2 = 12;

const int Shutdown = 23;

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

int ContT1;    //Contador da Tomada 1
int ContT2;    //Contador da Tomada 2
int ContT3;    //Contador da Tomada 3
int ContT4;    //Contador da Tomada 4
int ContT5;    //Contador da Tomada 5

int Telas = 0;
int ContagemTela = 1;

int ControleContagem1 = 1; //Variável de controle de contagem da M1, cada duas passagens no sensor conta um ciclo
int ControleContagem2 = 1; //Variável de controle de contagem da M2, cada duas passagens no sensor conta um ciclo

//Variaveis de contole de status
boolean ledState = false;

volatile bool ComandoGravaM1; //Comando para gravar a corrente inicial das tomadas e começar o monitoramento
volatile bool ComandoGravaM2; //Comando para gravar a corrente inicial das tomadas e começar o monitoramento

boolean iniciaM1;       //variavel para saber se o monitoramento já iniciou ou não
boolean iniciaM2;       //variavel para saber se o monitoramento já iniciou ou não

volatile bool Alarme1;        //Variavel para controle do alarme na maquina 1
volatile bool Alarme2;        //Variavel para controle do alarme na maquina 2

boolean Tomada1;    //Controle para saber se houve alarme na tomada 1
boolean Tomada2;    //Controle para saber se houve alarme na tomada 2
boolean Tomada3;    //Controle para saber se houve alarme na tomada 3
boolean Tomada4;    //Controle para saber se houve alarme na tomada 4
boolean Tomada5;    //Controle para saber se houve alarme na tomada 5

boolean IniciarMaquina;

boolean SemLuz;

String Key;

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

//objeto gravação da flash
Preferences preferences;

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

//********************************************************************************************************************************************

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

//********************************************************************************************************************************************

void IRAM_ATTR StartM1_ISR(){      //interrupção do sensor do contador de ciclos da M1
  ComandoGravaM1 = HIGH;
}

//********************************************************************************************************************************************

void IRAM_ATTR StartM2_ISR(){      //interrupção do sensor do contador de ciclos da M1
  ComandoGravaM2 = HIGH;
}
//**********************************************************************************************************************************************

void IRAM_ATTR ParadaM1_ISR(){      //interrupção do sensor de perda de referencia de posição da M1
  Alarme1 = HIGH;
}
//**********************************************************************************************************************************************


void IRAM_ATTR ParadaM2_ISR(){      //interrupção do sensor de perda de referencia de posição da M2
  Alarme2 = HIGH;
}

//**********************************************************************************************************************************************
void IRAM_ATTR Grava_dados_ISR(){
  SemLuz = HIGH;
 }

//***********************************************Setup********************************************************

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

    mb.addHreg(VAC1);
    mb.addHreg(VAC2);
    mb.addHreg(VAC3);
    mb.addHreg(VAC4);
    mb.addHreg(VAC5);

    DoisSec = millis();
    MeioSec = millis();

    emon1.current(I1, 8.1);             // Current: input pin, calibration.
    emon2.current(I2, 8.1);             // Current: input pin, calibration.
    emon3.current(I3, 8.1);             // Current: input pin, calibration.
    emon4.current(I4, 8.1);             // Current: input pin, calibration.
    emon5.current(I5, 8.1);             // Current: input pin, calibration.

    //pinMode(Botao, INPUT_PULLUP);
    pinMode(Rele1, OUTPUT);
    pinMode(Rele2, OUTPUT);
    pinMode(Contat1, OUTPUT);
    pinMode(Contat2, OUTPUT);
    pinMode(ledPin, OUTPUT);

    pinMode(SensCont1, INPUT_PULLUP);
    pinMode(SensCont2, INPUT_PULLUP);

    pinMode(SensStop1, INPUT_PULLUP);
    pinMode(SensStop2, INPUT_PULLUP);

    pinMode(StartM1, INPUT_PULLUP);
    pinMode(StartM2, INPUT_PULLUP);

    pinMode(Shutdown, INPUT_PULLUP);    

    iniciaM1 = LOW; 
    iniciaM2 = LOW; 

    Alarme1 = LOW;
    Alarme2 = LOW;

    ComandoGravaM1 = LOW;
    ComandoGravaM2 = LOW;

    Tomada1 = LOW;
    Tomada2 = LOW;
    Tomada3 = LOW;
    Tomada4 = LOW;
    Tomada5 = LOW;

    IniciarMaquina = LOW;

    SemLuz = LOW;

    lcd.init();
    lcd.clear();

    preferences.begin("Ciclos", false);
    ContT1 = preferences.getInt ("Tomada1", 0);
    ContT2 = preferences.getInt ("Tomada2", 0);
    ContT3 = preferences.getInt ("Tomada3", 0);
    ContT4 = preferences.getInt ("Tomada4", 0);
    ContT5 = preferences.getInt ("Tomada5", 0);
    preferences.end();


    //Interrupções dos sensores de posição das máquinas
    attachInterrupt(digitalPinToInterrupt(SensCont1), IncrementaContadorM1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(SensCont2), IncrementaContadorM2_ISR, RISING);

    attachInterrupt(digitalPinToInterrupt(SensStop1), ParadaM1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(SensStop2), ParadaM2_ISR, RISING);
    
    attachInterrupt(digitalPinToInterrupt(StartM1), StartM1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(StartM2), StartM2_ISR, RISING);

    attachInterrupt(digitalPinToInterrupt(Shutdown), Grava_dados_ISR, FALLING);
}
//***************************************************Fim do Setup*****************************************************************************

void GravaDados(){
  preferences.begin("Ciclos", false);
  preferences.putInt("Tomada1", ContT1);
  preferences.putInt("Tomada2", ContT2);
  preferences.putInt("Tomada3", ContT3);
  preferences.putInt("Tomada4", ContT4);
  preferences.putInt("Tomada5", ContT5);
  preferences.end();
}

//***************************************Rotina de leitura das correntes nas tomadas**********************************************************

//Rotina chamada a cada 2 segundos onde faz a leitura das correntes e envia no modbus, é possível usar para enviar a contagem de ciclos também
void LeCorrente(){
       mb.Hreg(VAC1, (ContT1));
       mb.Hreg(VAC2, (ContT2));
       mb.Hreg(VAC3, (ContT3));
       mb.Hreg(VAC4, (ContT4));
       mb.Hreg(VAC5, (ContT5));

      //Setting raw value (0-1024)
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

       lcd.setCursor(0,0);
       lcd.print("Aperte OK p/    ");
       lcd.setCursor(0,1);
       lcd.print("ligar as tomadas");
       delay(100);

       //Key = Read_TTP229_Keypad();
       if(Serial.available()){
       Key = Serial.readStringUntil('\n');
       if(Key == "4"){
        IniciarMaquina = HIGH;
        }
       }
  
      //Rotina para inverter o estado do led a cada meio segundo.
      if (millis() > MeioSec + 100) {
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

   MostraTelas();

    //Rotina para determinar se usuário quer ver alguma tela especifica
    if(Serial.available()){
        //String Key;
        //boolean trava = LOW;
      Key = Serial.readStringUntil('\n');
      if(Key == "1" || Key == "2" || Key == "3"|| Key == "4" || Key == "5" ){
        Serial.println("Ajuste tela");
        AjustaContagem();
        }
    }


    //Rotina para inverter o estado do led a cada meio segundo.
    if (millis() > MeioSec + 500){
      MeioSec = millis();
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      ContagemTela = ContagemTela + 1;
    }

    if(ContagemTela >= 4){
      ContagemTela = 0;
      Telas = Telas + 1;
    }

    if(Telas >= 7){
      Telas = 1;
    }
}

//************************************************************************************************************************
//Após cinco ciclos em qualquer uma das máquinas essa função será chamada para fazer a gravação dos valores de corrente e calcular a corrente para alarme
void GravaCorrentesM1(){
    iniciaM1 = HIGH;
    CorrenteGravada1 = CorrenteLida1;
    CorrenteGravada1 = CorrenteGravada1 / 4;
    CorrenteGravada2 = CorrenteLida2;
    CorrenteGravada2 = CorrenteGravada2 / 4;
    CorrenteGravada3 = CorrenteLida3;
    CorrenteGravada3 = CorrenteGravada3 / 4;
    lcd.setCursor(0,0);
    lcd.print("Gravou M1!!!!!  ");
    digitalWrite(ledPin, LOW);
    }

//*************************************************************************************************************************

void GravaCorrentesM2(){
    iniciaM2 = HIGH;
    CorrenteGravada4 = CorrenteLida4;
    CorrenteGravada4 = CorrenteGravada4 / 4;
    CorrenteGravada5 = CorrenteLida5;
    CorrenteGravada5 = CorrenteGravada5 / 4;
    lcd.setCursor(0,1);
    lcd.print("Gravou M2!!!!!  ");
    digitalWrite(ledPin, LOW);
  }

//*************************************************************************************************************************

  void ParaM1(){
    digitalWrite(Rele2, HIGH);  // Aciona emergencia M1
    digitalWrite(ledPin, HIGH); // Liga o led de aviso
    digitalWrite(Contat1, LOW); // Desliga contatora M1
        
    //Serial.println("Alarme Acionado");
    //WiFi.disconnect();
    //delay(1000);

    MostraTelas();

    if (millis() > MeioSec + 500){
      MeioSec = millis();
      ContagemTela = ContagemTela + 1;
    }

    if(ContagemTela >= 4){
      ContagemTela = 0;
      Telas = Telas + 1;
    }

    if(Alarme2 == HIGH){
      Telas = 9;
    }else if(Telas >= 8 && Alarme2 == LOW){
      Telas = 5;
    }


    //Aguarda ok do teclado para liberar o alarme
    if(Serial.available()){
      Key = Serial.readStringUntil('\n');
      if(Key == "4"){
        lcd.setCursor(0,0);
        lcd.print("Apagar Alarme?  ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        }
        Key = Serial.readStringUntil('\n');
        if(Key == "4"){
          Alarme1 = LOW;
          iniciaM1 = LOW;
          Alarme2 = LOW;
          iniciaM2 = LOW;
          digitalWrite(Rele2, LOW);   // Desliga emergencia M1
          digitalWrite(Contat1, HIGH); // Liga contatora M1
        }
   
    } 
  }

//*************************************************************************************************************************

  void ParaM2(){
    digitalWrite(Rele1, HIGH); //Aciona emergencia M2
    digitalWrite(ledPin, HIGH); // Liga o led de aviso
    digitalWrite(Contat2, LOW); //Desliga contatora M2
    
    //WiFi.disconnect();
    //delay(1000);

    MostraTelas();

    if (millis() > MeioSec + 500){
      MeioSec = millis();
      ContagemTela = ContagemTela + 1;
    }

    if(ContagemTela >= 4){
      ContagemTela = 0;
      Telas = Telas + 1;
    }

    if(Alarme1 == HIGH){
      Telas == 9;
    }else if(Telas >= 9){
        Telas = 2;
      }

    if(Telas == 5){
      Telas = 8;
    }

    //Aguarda ok do teclado para liberar o alarme
    if(Serial.available()){
      Key = Serial.readStringUntil('\n');
      if(Key == "4"){
        lcd.setCursor(0,0);
        lcd.print("Apagar Alarme?  ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        }
        Key = Serial.readStringUntil('\n');
        if(Key == "4"){
          Alarme1 = LOW;
          iniciaM1 = LOW;
          Alarme2 = LOW;
          iniciaM2 = LOW;
          digitalWrite(Rele1, LOW);   // Desliga emergencia M1
          digitalWrite(Contat2, HIGH); // Liga contatora M1
        }
    }
  }

//***********************************Leitura da tecla pressionada********************************************************************************
  
  byte Read_TTP229_Keypad(void){
  byte Num;
  byte Key_State = 0;
  for(Num = 1; Num <= 16; Num++){
    digitalWrite(SCL, LOW);
    if (!digitalRead(SDO))
       Key_State = Num;
      digitalWrite(SCL, HIGH);
    } 
  return Key_State;
}

//************************************Ajuste Contagem **************************************************
void AjustaContagem(){
  boolean trava = LOW;
  int Estado;
  Estado = Key.toInt();
  switch (Estado){
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ciclos T1");
      lcd.setCursor(0,1);
      lcd.print(ContT1);
      delay(100);
      trava = HIGH;

      while(trava == HIGH){
        if(Serial.available()){
          String Testando;
          Testando = Serial.readStringUntil('\n');
          ContT1 = Testando.toInt();
          trava = LOW;
        }
      }
    break;
  
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ciclos T2");
      lcd.setCursor(0,1);
      lcd.print(ContT2);
      delay(100);
      trava = HIGH;

      while(trava == HIGH){
        if(Serial.available()){
          String Testando;
          Testando = Serial.readStringUntil('\n');
          Serial.println("Testando");
          Serial.println(Testando);
          ContT2 = Testando.toInt();
          trava = LOW;
          }
      }
    break;
                
    case 3:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ciclos T3");
      lcd.setCursor(0,1);
      lcd.print(ContT3);
      delay(100);
      trava = HIGH;

      while(trava == HIGH){
        if(Serial.available()){
          String Testando;
          Testando = Serial.readStringUntil('\n');
          Serial.println("Testando");
          Serial.println(Testando);
          ContT3 = Testando.toInt();
          trava = LOW;
        }
      }
    break;
        
    case 4:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ciclos T4");
      lcd.setCursor(0,1);
      lcd.print(ContT4);
      delay(100);
      trava = HIGH;

      while(trava == HIGH){
        if(Serial.available()){
          String Testando;
          Testando = Serial.readStringUntil('\n');
          Serial.println("Testando");
          Serial.println(Testando);
          ContT4 = Testando.toInt();
          trava = LOW;
        }
      }
    break;
                
    case 5:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ciclos T5");
      lcd.setCursor(0,1);
      lcd.print(ContT5);
      delay(100);
      trava = HIGH;

      while(trava == HIGH){
        if(Serial.available()){
          String Testando;
          Testando = Serial.readStringUntil('\n');
          Serial.println("Testando");
          Serial.println(Testando);
          ContT5 = Testando.toInt();
          trava = LOW;
        }
      }
    break;
    }
 } 

//**************************************Mostra Telas********************************************************************
 void MostraTelas(){
  switch (Telas){

      case 1:
        lcd.setCursor(0,0);
        lcd.print("Posicionar maq. ");
        lcd.setCursor(0,1);
        lcd.print("Ligar secadores ");
        delay(100);
        break;

      case 2:
        lcd.setCursor(0,0);
        lcd.print("Ciclos T1.      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        lcd.print(ContT1);
        delay(100);
        break;

      case 3:
        lcd.setCursor(0,0);
        lcd.print("Ciclos T2.      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        lcd.print(ContT2);
        delay(100);
        break;

      case 4:
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ciclos T3.      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        lcd.print(ContT3);
        delay(100);
        break;

      case 5:
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ciclos T4.      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        lcd.print(ContT4);
        delay(100);
        break;

      case 6:
        //lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ciclos T5.      ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
        lcd.print(ContT5);
        delay(100);
        break;

      case 7:
       if(Tomada1 == HIGH){
          lcd.setCursor(0,0);
          lcd.print("Alarme          ");
          lcd.setCursor(0,1);
          lcd.print("Tomada 1        ");
        }else if(Tomada2 == HIGH){
          lcd.setCursor(0,0);
          lcd.print("Alarme          ");
          lcd.setCursor(0,1);
          lcd.print("Tomada 2        ");
        }else if(Tomada3 == HIGH){
          lcd.setCursor(0,0);
          lcd.print("Alarme          ");
          lcd.setCursor(0,1);
          lcd.print("Tomada 3        ");
        }else{
          lcd.setCursor(0,0);
          lcd.print("Maquina 1 fora  ");
          lcd.setCursor(0,1);
          lcd.print("de controle     ");
        }

      case 8:
        if(Tomada4 == HIGH){
          lcd.setCursor(0,0);
          lcd.print("Alarme          ");
          lcd.setCursor(0,1);
          lcd.print("Tomada 4        ");
        }else if(Tomada5 == HIGH){
          lcd.setCursor(0,0);
          lcd.print("Alarme          ");
          lcd.setCursor(0,1);
          lcd.print("Tomada 5        ");
        }else{
          lcd.setCursor(0,0);
          lcd.print("Maquina 2 fora  ");
          lcd.setCursor(0,1);
          lcd.print("de controle     ");
        }

      case 9:
          lcd.setCursor(0,0);
          lcd.print("Maquinas 1 e 2  ");
          lcd.setCursor(0,1);
          lcd.print("fora de controle");
      break;    
    } 
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

  if(iniciaM1 == LOW && iniciaM2 == LOW && ComandoGravaM1 == LOW && IniciarMaquina == LOW && Alarme1 == LOW && Alarme2 == LOW){  //Aguarda para iniciar novo ciclo
    AguardaInicio();    //Incrementa tela
    Serial.println("Aguardando inicio");
    }

  // Rotina de inicio para aguardar a leitura das correntes
  if(iniciaM1 == LOW && iniciaM2 == LOW && ComandoGravaM1 == LOW && IniciarMaquina == HIGH && Alarme1 == LOW && Alarme2 == LOW){  //Aguarda para iniciar novo ciclo
      PosicionaMaquinas();    //Rotina de aguardar o inicio dos ensaios
      Serial.println("Posiciona Maquina");
    }

  //Rotina de gravação das correntes de referencia de alarme
  if(iniciaM1 == LOW && ComandoGravaM1 == HIGH && Alarme1 == LOW && Alarme2 == LOW){ //Quando detectar comando para iniciar os ensaios
          GravaCorrentesM1();   //Grava a corrente atual e calcula a corrente limite para disparar o alarme.
          Serial.println("Gravou Correntes M1");
          Serial.println(iniciaM1);
          Serial.println(ComandoGravaM1);
          Serial.println(Alarme1);
          Serial.println(Alarme2);
  }

    //Rotina de gravação das correntes de referencia de alarme
  if(iniciaM2 == LOW && ComandoGravaM2 == HIGH && Alarme1 == LOW && Alarme2 == LOW){ //Quando detectar comando para iniciar os ensaios
          GravaCorrentesM2();   //Grava a corrente atual e calcula a corrente limite para disparar o alarme.
          Serial.println("Gravou Correntes M2");
          Serial.println(iniciaM2);
  }

//Rotina para determinação de qual tomada está em alarme na maquina 1
  if(iniciaM1 == HIGH && ComandoGravaM1 == HIGH && Alarme1 == LOW){   //Rotina normal durante ensaio  
    //Criar rotina para contagem dos ciclos, salvar na memoria externa e mostrar no display

    Serial.println("Testou aqui!!!!");
     Serial.println(Alarme1);

    if(CorrenteLida1 < CorrenteGravada1 && Alarme1 == LOW){
      Tomada1 = HIGH;
    }

    if(CorrenteLida2 < CorrenteGravada2 && Alarme1 == LOW){
      Tomada2 = HIGH;
    }

    if(CorrenteLida3 < CorrenteGravada3 && Alarme1 == LOW){
      Tomada3 = HIGH;
    }

    //Rotina para inverter o estado do led a cada meio segundo.
    if (millis() > MeioSec + 500){
      MeioSec = millis();
      ContagemTela = ContagemTela + 1;
    }

    if(ContagemTela >= 4){
      ContagemTela = 0;
      Telas = Telas + 1;
    }

    if(Telas >= 7){
      Telas = 2;
    }

    MostraTelas();

    if(Serial.available()){
        //String Key;
        //boolean trava = LOW;
      Key = Serial.readStringUntil('\n');
      if(Key == "1" || Key == "2" || Key == "3"|| Key == "4" || Key == "5" ){
        Serial.println("Ajuste tela");
        AjustaContagem();
        }
    }

    Serial.println("Lendo correntes M1");
  }

//Rotina para determinação de qual tomada está em alarme na maquina 2
  if(iniciaM2 == HIGH && ComandoGravaM2 == HIGH && Alarme2 == LOW ){   //Rotina normal durante ensaio  && SensStop2 == LOW
    //Criar rotina para contagem dos ciclos, salvar na memoria externa e mostrar no display

    if(CorrenteLida4 < CorrenteGravada4 && Alarme2 == LOW){
      Tomada4 = HIGH;
    }

    if(CorrenteLida5 < CorrenteGravada5 && Alarme2 == LOW){
      Tomada5 = HIGH;
    }

    if(Serial.available()){
        //String Key;
        //boolean trava = LOW;
      Key = Serial.readStringUntil('\n');
      if(Key == "1" || Key == "2" || Key == "3"|| Key == "4" || Key == "5" ){
        Serial.println("Ajuste tela");
        AjustaContagem();
        }
    }
    Serial.println("Lendo Correntes M2");
    MostraTelas();
  }

//Rotinas de parada da maquina  1
  if(iniciaM1 == HIGH && Tomada1 == HIGH || Tomada2 == HIGH || Tomada3 == HIGH || Alarme1 == HIGH){   //Se acionar o alarme em qualquer tomada da M1
    ParaM1();
    Serial.println("Emergencia M1");
  }

//Rotinas de parada da maquina  2
   if(iniciaM2 == HIGH && Tomada4 == HIGH || Tomada5 == HIGH || Alarme2 == HIGH){    //Se acionar o alarme em qualquer tomada da M2
      ParaM2();
    Serial.println("Emergencia M2");
  } 

    if(SemLuz == HIGH){
      GravaDados();
    }

}