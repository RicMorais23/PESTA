//Bibliotecas
#include <Arduino.h>
#include "NimBLEDevice.h"
#include <String>
#include "WiFi.h"         // Biblioteca WI-FI
#include "PubSubClient.h" // Biblioteca MQTT
#include "DHT.h"          //Biblioteca  DHT11
#include <DHT_U.h>


//WI-FI Parâmetros
const char *WIFI_Network = "ZON-D2E0";
const char *WIFI_Password = "fc1ce0717d08";
// Tempo definido para voltar a tentar ligar ao WI-Fi
#define WIFI_Timeout_seg 60 

//MQTT Parâmetros
const char *mqtt_server = "192.168.1.6";
//Parâmetros - User, PASS, ID
const char *mqtt_user = "morais23";
const char *mqtt_pass = "Morais23";
const char *ID = "ESP32.1";

//Iniciar objetos WIFI e o MQTT Client
WiFiClient wifiClient;
PubSubClient client (mqtt_server, 1883, wifiClient);
//PubSubClient client(wifiClient);

//Topics
#define topic1 "Quarto/iluminação/cor"
#define topic2 "Quarto/iluminação/intensidade"
#define topic3 "Quarto/temp"
#define topic4 "Quarto/temp/ac"
#define topic5 "Garagem/porta"
#define topic6 "Garagem/porta/estado"
#define topic7 "Garagem/carro/estado"
#define topic8 "Quarto/AC/estado"


//------------------ Configurações Dispositivo BLE------------------------------------------------------------------------//
#define SCAN_INTERVALO 1000   // Tempo (s) do intervalo entre cada Scan                                                   //
#define MAX_MISSING_Time 5000 // Tempo limite para o programa perceber que perdeu a conectividade ao dispositivo BLE      //
#define Tamanho 1             // Numero de dispositivos BLE                                                               //
                                                                                                                          //
typedef struct                                                                                                            //
{                                                                                                                         //
String addressMAC;                 // Endereço MAC dos dispositivos BLE                                                   //
boolean dispositivoFound = false;  // Varialvel para guardar se o dispositivo foi ou não encontrado                       //
uint32_t lastFoundTime = 0;        // Último tempo desde que foi encontrado um dispositivo                                //
uint32_t arrivalTime = -1;	       // Momento em que o dispositivo foi reconhecido pela primeira vez                      //
uint32_t exitTime = -1;                                                                                                   //
uint32_t workTimer = -1;                                                                                                  //
}Device;                                                                                                                  //
                                                                                                                          //
NimBLEScan *pBLEScan;                                                                                                     //
uint32_t lastScanTime = 0;                                                                                                //
Device devices[Tamanho]; // Guarda as informações de cada dispositivo registado                                           //
String addressMACList[Tamanho] = {"fa:c2:a7:1c:ed:9c"}; // Endereço MAC do dispositivo autorizado                         //
                                                                                                                          //                          
//Estrutura para flags                                                                                                    //  
typedef struct                                                                                                            //
{                                                                                                                         //  
  unsigned char dispositivoBLE; // flag qie indica a presença do dispositivo autorizado                                   //  
}flags_st;                                                                                                                //
                                                                                                                          //
volatile flags_st flags = {0};                                                                                            //
//------------------------------------------------------------------------------------------------------------------------//  

//---------------------------- Configurações Sonar ----------------------//
#define t_envio_seg 1                                                    //
#define trig1 34                                                         //
#define echo1 35                                                         //
                                                                         //
long duration;               // Tempo de eco                             //
unsigned long t0 = millis(); // Tempo inicial                            //
unsigned long t = 0;         // Tempo passado                            //
float distance;              // Variavel para guardar a distancia        //
float v_som = 343.31;        // Velocidade do som                        //
//-----------------------------------------------------------------------//

//---------------------- Configurações LED RGB ----------------------------------//
// Pinos LED RGB 1                                                               //
#define red1  32                                                                 //
#define green1 33                                                                //
#define blue1  25                                                                //
                                                                                 //                                                                 
//Configurações LED_RGB_1 PWM                                                    //
const int freq1 = 3000;       // configurar uma freq de 3 Khz                    //
const int pwmChannel_r1 = 2;  // utiliza o canal 2 pwm para o red1               //
const int pwmChannel_g1 = 3;  // utiliza o canal 3 pwm para o green1             //
const int pwmChannel_b1 = 4;  // utiliza o canal 4 pwm para o blue1              //
const int resolution1 = 8;    // utiliza uma resolução de 8 bits                 //
int dutyCycle_r1 = 0;                                                            //
int dutyCycle_g1 = 0;                                                            //
int dutyCycle_b1 = 0;                                                            //
                                                                                 //
//Struct para falgs controlo do LED1_RGB                                         //
typedef struct                                                                   //
{                                                                                //
  unsigned char r1; //flag para verificar se a cor vermelha esta ligada          //
  unsigned char g1; //flag para verificar se a cor verde esta ligada             //
  unsigned char b1; //flag para verificar se a cor azul esta ligada              //
  unsigned char y1; //flag para verificar se a cor amarela esta ligada           //
  unsigned char w1; //flag para verificar se a cor branca esta ligada            // 
  unsigned char pwm1_r1; //flag para + pwm cor vermelha                          //
  unsigned char pwm1_g1; //flag para + pwm cor verde                             //
  unsigned char pwm1_b1; //flag para + pwm cor azul                              //
  unsigned char pwm1_y1; //flag para + pwm cor amarela                           //
  unsigned char pwm1_w1; //flag para + pwm cor branca                            //
  unsigned char pwm0_r1; //flag para - pwm cor vermelha                          //
  unsigned char pwm0_g1; //flag para - pwm cor verde                             //
  unsigned char pwm0_b1; //flag para - pwm cor azul                              //
  unsigned char pwm0_y1; //flag para - pwm cor amarela                           //
  unsigned char pwm0_w1; //flag para - pwm cor branca                            //
  unsigned char turn_off; // flag para desligar iluminação                       //
}flagsrgb_st;                                                                    //
//iniciar flags a 0                                                              //
volatile flagsrgb_st flagsrgb = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//
//-------------------------------------------------------------------------------//

//--------------------------- Configurações Sensores de Temperatura---------------------------------------------//
                                                                                                                //
#define DHTTYPE DHT11             // definir o tipo de sensor a usar                                            //
#define DHTPIN 0                  // pino onde está conectado o output                                          //
#define cont_sg 2                 // tempo definido para o sensor fazer a leitura e imprimir                    //
unsigned long cont = millis();                                                                                  //
unsigned long tempoAnterior = 0;  // cria uma variavel que guarda o tempo anterior                              //
DHT dht(DHTPIN, DHTTYPE);         // cria um objeto para o sensor DHT, com o pino de saída e o tipo de sensor   //
float temperatura;                // variavel para guardar o valor da Temperatura                               //
//--------------------------------------------------------------------------------------------------------------//

//---------------------------- Configurações Motores -----------------------------------------------//
                                                                                                    //
// Pinos MOTOR Garagem - Controlo de Rotação                                                        //
#define IN1_A 14                                                                                    //
#define IN2_A 12                                                                                    //
#define EN_A 13                                                                                     // 
                                                                                                    //
// Configurações PWM - Ventoinha                                                                    //
const int freqA = 30000;     // definir uma frequência de 30KHz                                     //
const int pwmChannelA = 0;   // canal PWM - 0                                                       //
const int resolutionA = 8;   // resolução de ( 8 bits)                                              //
int dutyCycleA = 255;        // dutycycle inicia - 255                                              //
                                                                                                    //  
//Pinos Motor ventoinha - Controlo de Rotação                                                       //
#define IN1_B 21                                                                                    // 
#define IN2_B 22                                                                                    //    
#define EN_B  23                                                                                    //
                                                                                                    //
// Configurações PWM - Motor Garagem                                                                //
const int freqB = 30000;   // definir uma frequência de 30KHz                                       //
const int pwmChannelB = 1; // canal PWM - 1                                                         //
const int resolutionB = 8; // resolução de ( 8 bits)                                                //
int dutyCycleB = 150;      // dutycycle inicia - 150                                                //
                                                                                                    //
//--------------------------------------------------------------------------------------------------//

//-------------Configurações Sensores Infra-vermelho Garagem ---------//
#define infraGaragem_1 19                                             //
#define infraGaragem_2 18                                             //
#define infraGaragem_deteta 17                                        //
//--------------------------------------------------------------------//

//----- Pinos para os Leds------//
#define led 26
#define led1 27
#define ledWC 15

typedef struct 
{
  unsigned char dispositivoBLE;  // flag que verifica a presença do dispositivo BLE
  unsigned char fechar_garagem;  // flag que verifica se foi enviada a instrução de fechar garagem
  unsigned char abrir_garagem;   // flag que verifica se foi enviada a instrução de abrir garagem
  unsigned char detetaCarro;     // flag que verifica a presença do carro na garagem
  unsigned char modoAuto;        // flag que verifica se o modo Auto. está ligado 
}flagsGaragem_st;

volatile flagsGaragem_st flagsGaragem = {0 ,0 ,0 ,0, 1};

typedef struct
{
  unsigned char desligarAC;
  unsigned char ligarAC;
}flagsAC_st;

volatile flagsAC_st flagsAc = {0 ,0};

char estadoPortaGaragem[10]; // variável para guardar estado da Porta Garagem
char estadoGaragemCarro[10]; // variável para guardar estado carro
char estadoAC[10];           // variável que guarda o estado da ventoinha


void conectarWIFI() // função que estabelece a conexão WIFI
{
  //Fazer um print para mostrar que se está a iniciar a conexão
  Serial.print("Conectando ao WIFI...");
  // Existem 2 modos de conexão ao wifi:
  // -> STA
  // -> AP
  // Quando me quero conectar a um WIFI já existente tem que ser usado o modo STA
  WiFi.mode(WIFI_STA);                     // função que permite definir o modo de conexão e assim usar uma rede já existente
  WiFi.begin(WIFI_Network, WIFI_Password); //inicializar a conexão WIFI, com o nome da Rede e a password da mesma

  unsigned long startconect = millis(); //guarda o tempo que passou desde o inicio da tentativa de conexão ao WIFI

  while (WiFi.status() != WL_CONNECTED && (millis() - startconect) < WIFI_Timeout_seg * 1000) // Enquanto a conexão ainda n estiver estabelecida e o tempo que passou desde a tentativa de conexão for inferior à estabelecida (60s)
  {
    Serial.print("\n");
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\n");
    Serial.println(" Falhou!");
  }
  else
  {
    Serial.print("\n");
    Serial.print("Conectado!!");
    Serial.print("\n");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP()); // Dá print ao IP local
  }
}

void callback(char *topic, byte *mensagem, unsigned int length) // função que analise as mensagens que chegaram nos tópicos correspondentes
{
  Serial.print("Messagem chegou do tópico: ");
  Serial.print(topic);
  Serial.print(" . Mensagem: ");
  String resposta; //defenir uma classe para a string

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)mensagem[i]);
    resposta += (char)mensagem[i];
  }

  if (String(topic) == topic1)
  {
    if(resposta == "red" && flagsrgb.r1 == 0) // se o utilizador escolher a cor vermelha
   {
     flagsrgb.r1 = 1; // colocar a flag red a 1
     // colocar as restantes a 0
     flagsrgb.g1 = 0;
     flagsrgb.b1 = 0;
     flagsrgb.w1 = 0;
     flagsrgb.y1 = 0;
   }
   else if (resposta == "blue" && flagsrgb.b1 == 0)// se o utilizador escolher a cor azul
   {
     flagsrgb.b1 = 1; // colocar a flag blue a 1
      // colocar as restantes a 0
     flagsrgb.r1 = 0;
     flagsrgb.g1 = 0;
     flagsrgb.w1 = 0;
     flagsrgb.y1 = 0;
   }
   else if (resposta == "green" && flagsrgb.g1 == 0)// se o utilizador escolher a cor verde
   {
     flagsrgb.g1 = 1; // colocar a flag green a 1
      // colocar as restantes a 0
     flagsrgb.r1 = 0;
     flagsrgb.b1 = 0;
     flagsrgb.w1 = 0;
     flagsrgb.y1 = 0;
   }
   else if (resposta == "white" && flagsrgb.w1 == 0)// se o utilizador escolher a cor branca
   {
     flagsrgb.w1 = 1; // colocar a flag white a 1
      // colocar as restantes a 0
     flagsrgb.r1 = 0;
     flagsrgb.g1 = 0;
     flagsrgb.b1 = 0;
     flagsrgb.y1 = 0;
   }
   else if (resposta == "yellow" && flagsrgb.y1 == 0)// se o utilizador escolher a cor amarela
   {
     flagsrgb.y1 = 1; // colocar a flag blue a 1
      // colocar as restantes a 0
     flagsrgb.r1 = 0;
     flagsrgb.g1 = 0;
     flagsrgb.b1 = 0;
     flagsrgb.w1 = 0;
   }
   else if (resposta == "desligar" && flagsrgb.turn_off == 0) //se o utilizador pretende desligar a iluminação
   {
     flagsrgb.turn_off = 1; // coloca flag a 1
   }
   else if (resposta == "ligar wc")
   {
     digitalWrite(ledWC, HIGH);
   }
   else if (resposta == "desligar wc")
   {
     digitalWrite(ledWC, LOW);
   }
 }
 else if(String(topic) == topic2) 
 {
   if(resposta == "mais" && flagsrgb.r1 == 1) // se o utilizador pretende aumentar a luminosidade da cor vermelha 
   {
     flagsrgb.pwm1_r1 = 1; // colocar a flag de aumentar pwm da cor vermelha a 1
     //colocar a flag de dimninuir pwm  a 0
     flagsrgb.pwm0_r1 = 0;
     
   }
   else if (resposta == "mais" && flagsrgb.b1 == 1) // se o utilizador pretende aumentar a luminosidade da cor azul
   {
     flagsrgb.pwm1_b1 = 1; // colocar a flag de aumentar pwm da cor azul a 1
     //colocar a flag de dimninuir pwm  a 0
    flagsrgb.pwm0_b1 = 0;
   } 
   else if (resposta == "mais" && flagsrgb.g1 == 1) // se o utilizador pretende aumentar a luminosidade da cor verde
   {
     flagsrgb.pwm1_b1 = 1; // colocar a flag de aumentar pwm da cor verde a 1
     //colocar a flag de dimninuir pwm  a 0
    flagsrgb.pwm0_b1 = 0;
   } 
   else if (resposta == "mais" && flagsrgb.y1 == 1) // se o utilizador pretende aumentar a luminosidade da cor amarela
   {
     flagsrgb.pwm1_y1 = 1; // colocar a flag de aumentar pwm da cor amarela a 1
     //colocar a flag de dimninuir pwm  a 0
    flagsrgb.pwm0_y1 = 0;
   } 
   else if (resposta == "mais" && flagsrgb.w1 == 1) // se o utilizador pretende aumentar a luminosidade da cor branca
   {
     flagsrgb.pwm1_w1 = 1; // colocar a flag de aumentar pwm da cor branca a 1
     //colocar a flag de dimninuir pwm  a 0
    flagsrgb.pwm0_w1 = 0;
   } 
   else if(resposta == "menos" && flagsrgb.r1 == 1) // se o utilizador pretende diminuir a luminosidade da cor vermelha 
   {
     flagsrgb.pwm0_r1 = 1; // colocar a flag de diminuir pwm da cor vermelha a 1
     //colocar a flag de aumentar pwm  a 0
     flagsrgb.pwm1_r1 = 0;
     
   }
   else if (resposta == "menos" && flagsrgb.b1 == 1) // se o utilizador pretende diminuir a luminosidade da cor azul
   {
     flagsrgb.pwm0_b1 = 1; // colocar a flag de diminuir pwm da cor azul a 1
     //colocar a flag de aumentar pwm  a 0
    flagsrgb.pwm1_b1 = 0;
   } 
   else if (resposta == "menos" && flagsrgb.g1 == 1) // se o utilizador pretende diminuir a luminosidade da cor verde
   {
     flagsrgb.pwm0_b1 = 1; // colocar a flag de diminuir pwm da cor verde a 1
     //colocar a flag de aumentar pwm  a 0
    flagsrgb.pwm1_b1 = 0;
   } 
   else if (resposta == "menos" && flagsrgb.y1 == 1) // se o utilizador pretende diminuir a luminosidade da cor amarela
   {
     flagsrgb.pwm0_y1 = 1; // colocar a flag de diminuir pwm da cor amarela a 1
     //colocar a flag de aumentar pwm  a 0
    flagsrgb.pwm1_y1 = 0;
   } 
   else if (resposta == "menos" && flagsrgb.w1 == 1) // se o utilizador pretende diminuir a luminosidade da cor branca
   {
     flagsrgb.pwm0_w1 = 1; // colocar a flag de diminuir pwm da cor branca a 1
     //colocar a flag de aumentar pwm  a 0
    flagsrgb.pwm1_w1 = 0;
   } 
 }
 else if (String(topic) == topic4)
 {
   if(resposta == "ativar AC") // ligar ventoinha
   {
     flagsAc.ligarAC = 1;
      // colocar flag de desligar a 0
    flagsAc.desligarAC = 0;
   }
   else if (resposta == "desligar AC") //desligar ventoinha
   {
     flagsAc.desligarAC = 1;
    // colocar flag de ligar a 0
    flagsAc.ligarAC = 0;
  } 
 }
 else if (String(topic) == topic5)
 {
   if(resposta == "abrir garagem")//  abrir porta Garagem
   {
     flagsGaragem.abrir_garagem = 1;
   }
   else if (resposta == "fechar garagem")// fechar porta Garagem
   {
     flagsGaragem.fechar_garagem = 1;
   }
 } 
}

void conectar_mqtt() //função para fazer a ligação  MQTT
{
  // Enquanto não se conecta
  while (!client.connected())
  {
    Serial.print("Esperando conexão ao MQTT...");
    // Se a conexão ocorrer
    if (client.connect(ID, mqtt_user, mqtt_pass))
    {
      Serial.println("Conectado");
      //listar todos os topicos usados para subscrever ações
      client.subscribe(topic1);
      client.subscribe(topic2);
      client.subscribe(topic4);
      client.subscribe(topic5);
    }
    else
    {
      Serial.println(" Tentar de novo em 5 segundos");
      // Espera 5 segundos e tenta de novo
      delay(5000);
    }
  }
}

class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice)
  {
    uint32_t now = millis();
    for(int i = 0; i < Tamanho; i++)
    {
      if(strcmp(advertisedDevice -> getAddress().toString().c_str(), addressMACList[i].c_str()) == 0) 
      // se a comparação entre o endereço MAC que foi encontrado no Scan corresponder ao do dispositivo autorizado
      {
        if(!devices[i].dispositivoFound) devices[i].arrivalTime = now; // se ainda n foi encontrado nenhum dispositivo
         devices[i].dispositivoFound = true; // coloca o valor em verdadeiro, pq foi encontrado
         devices[i].lastFoundTime = now;    // guarda o tempo que passou 
         digitalWrite(led, HIGH);
         flags.dispositivoBLE = 1;
         break; 
      }
    }
  }
};

void clearNoMoreFound()
{
  uint32_t now = millis();
  for (int i = 0; i<Tamanho; i++){
    if((devices[i].dispositivoFound)&&((now - devices[i].lastFoundTime) > MAX_MISSING_Time))
    {
      devices[i].dispositivoFound = false;
      flags.dispositivoBLE = 0;
    }
  }
}

void fillDevices()
{
  for (int i = 0; i < Tamanho; i++)
  {
    devices[i].addressMAC = addressMACList[i];
    // lista os MAC ADRESS dos dispositivos 
  }
}

void scanDispositivo()
{
  uint32_t now = millis();
  // Inicia o Sacn se o passou o tempo definido
  if(now - lastScanTime > SCAN_INTERVALO)
  {
    lastScanTime = now;
    pBLEScan -> start(1, false);
    clearNoMoreFound();
  }
}

void processar_dist()
{
  digitalWrite(trig1,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1,LOW);
  // espera pelo pulse do echo
  duration = pulseIn(echo1, HIGH);
  // Calcular o valor da distanci
  distance = 0.5 * v_som * (float)duration/10000.0 ;
}  

void veiculoGaragem()
{
  if((flags.dispositivoBLE == 1) && (distance > 6 && distance < 13) )
  {
    int estado = 1;
    dtostrf(estado, 3 ,1, estadoGaragemCarro);
    client.publish(topic7, estadoGaragemCarro);
    flagsGaragem.dispositivoBLE = 1; //coloca a flag a 1 -> indica presença de carro autorizado
  }
  else{
    flagsGaragem.dispositivoBLE = 0; //coloca a flag a 0 -> carro não autorizado
  }
}

void detetaCarroGaragem()
{
  if (digitalRead(infraGaragem_deteta) == HIGH) // Se o sensor não deteta nenhum carro na garagem
  {
    // Colocar a flag a 0
    flagsGaragem.detetaCarro = 0;
  }
  else if (digitalRead(infraGaragem_deteta) == LOW) // Se o sensor detetar um carro estacionado na garagem
  {
    // Colocar a flag a 1
    flagsGaragem.detetaCarro = 1;
  }
}

void controlarPortaGaragem_manual()
{
  //Fechar a Porta da Garagem modo manual
  if(digitalRead(infraGaragem_1) == LOW && digitalRead(infraGaragem_2) == HIGH && flagsGaragem.fechar_garagem == 1)
  {
    //colocar o motor a rodar no sentido de fechar a garagem
    digitalWrite(IN1_B, HIGH); 
    digitalWrite(IN2_B, LOW);
    //definir velocidade
    ledcWrite(pwmChannelA, dutyCycleA); 
    int estado = 1; // porta a fechar
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
  else if(digitalRead(infraGaragem_2) == LOW && flagsGaragem.fechar_garagem == 1) 
  {
    // parar o motor
    digitalWrite(IN1_B, LOW);
    digitalWrite(IN2_B, LOW);
    //colocar a flag de fechar porta a 0
    flagsGaragem.fechar_garagem = 0;
    
    int estado = 0; // porta fechada
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
  //Abrir Porta da Garagem Manual
  else if (digitalRead(infraGaragem_2) == LOW && flagsGaragem.abrir_garagem == 1)
  {
    // colocar o motor a rodar no sentido de abrir a Porta
    digitalWrite(IN1_B, LOW); 
    digitalWrite(IN2_B, HIGH);
    //definir velocidade
    ledcWrite(pwmChannelA, dutyCycleA); 
    int estado = 2; // porta a abrir
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
  else if (digitalRead(infraGaragem_1) == LOW  && flagsGaragem.abrir_garagem == 1)
  {
    // o motor vai parar pq a porta está totalmente aberta
    digitalWrite(IN1_B, LOW); 
    digitalWrite(IN2_B, LOW);
    // colocar a flag de abrir a 0
    flagsGaragem.abrir_garagem = 0;
    
    int estado = 3; // porta aberta
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
}

void controlarPortaGaragem_aut()
{
  //Abrir Porta da Garagem automaticamente
  if (digitalRead(infraGaragem_2) == LOW && flagsGaragem.dispositivoBLE == 1 && flagsGaragem.detetaCarro == 0 && flagsGaragem.modoAuto == 1)
  {
    // colocar o motor a rodar no sentido de abrir a Porta
    digitalWrite(IN1_B, LOW); 
    digitalWrite(IN2_B, HIGH);
    //definir velocidade
    ledcWrite(pwmChannelA, dutyCycleA); 
    
    int estado = 2; // porta a abrir
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
  else if (digitalRead(infraGaragem_1) == LOW  && flagsGaragem.dispositivoBLE == 1 && flagsGaragem.detetaCarro == 0 && flagsGaragem.modoAuto == 1)
  {
    // o motor vai parar pq a porta está totalmente aberta
    digitalWrite(IN1_B, LOW); 
    digitalWrite(IN2_B, LOW);
    int estado = 3; // porta aberta
    dtostrf(estado, 3 ,1, estadoPortaGaragem);
    client.publish(topic6, estadoPortaGaragem);
  }
}

void colorLED1()
{
  if (flagsrgb.r1 == 1) // caso seja escolhida a cor vermelha
  {
    dutyCycle_r1 = 255; // colocar duty. da cor vermelha no máximo
    // colocar restantes a 0
    dutyCycle_g1 = 0;
    dutyCycle_b1 = 0;  
    ledcWrite(pwmChannel_g1,dutyCycle_g1);
    ledcWrite(pwmChannel_b1,dutyCycle_b1); 
    ledcWrite(pwmChannel_r1,dutyCycle_r1);
  }
  else if (flagsrgb.g1 == 1) // caso seja escolhida a cor verde
  {
    dutyCycle_g1 = 255; // colocar duty. da cor verde no máximo
    // colocar restantes a 0
    dutyCycle_r1 = 0;
    dutyCycle_b1 = 0;  
    ledcWrite(pwmChannel_g1,dutyCycle_g1);
    ledcWrite(pwmChannel_b1,dutyCycle_b1); 
    ledcWrite(pwmChannel_r1,dutyCycle_r1);
  }
  else if (flagsrgb.b1 == 1) // caso seja escolhida a cor azul
  {
    dutyCycle_b1 = 255; // colocar duty. da cor azul no máximo
    // colocar restantes a 0
    dutyCycle_r1 = 0;
    dutyCycle_g1 = 0;  
    ledcWrite(pwmChannel_g1,dutyCycle_g1);
    ledcWrite(pwmChannel_b1,dutyCycle_b1); 
    ledcWrite(pwmChannel_r1,dutyCycle_r1);
  }
  else if (flagsrgb.y1 == 1) // caso seja escolhida a cor amarela
  {
    dutyCycle_b1 = 255; // colocar duty. da cor azul no máximo
    dutyCycle_g1 = 255; // colocar duty. da cor verde no máximo
    // colocar restantes a 0
    dutyCycle_r1 = 0;
    ledcWrite(pwmChannel_g1,dutyCycle_g1);
    ledcWrite(pwmChannel_b1,dutyCycle_b1); 
    ledcWrite(pwmChannel_r1,dutyCycle_r1);
  }
  else if (flagsrgb.w1 == 1) // caso seja escolhida a cor branca
  { 
    // colocar todas a 1
    dutyCycle_b1 = 255;
    dutyCycle_r1 = 255;
    dutyCycle_g1 = 255;
    ledcWrite(pwmChannel_g1,dutyCycle_g1);
    ledcWrite(pwmChannel_b1,dutyCycle_b1); 
    ledcWrite(pwmChannel_r1,dutyCycle_r1);
  }
}

void pwmLED1()
{
  if (flagsrgb.pwm1_r1 == 1 ) // caso seja escolhido aumentar a cor vermelha
  {

    if(dutyCycle_r1 <=255 && dutyCycle_r1 >=0)
      {
        flagsrgb.r1;
         ledcWrite(pwmChannel_r1, dutyCycle_r1);  
         dutyCycle_r1 = dutyCycle_r1 + 15 ;
      }
  }
  else if (flagsrgb.pwm0_r1 == 1 ) //caso seja escolhido diminuir a cor vermelha
  {
    flagsrgb.r1;
    if(dutyCycle_r1 <=255 && dutyCycle_r1 >=0)
      {
         ledcWrite(pwmChannel_r1, dutyCycle_r1);  
         dutyCycle_r1 = dutyCycle_r1 - 15 ;
       }
  }
  else if (flagsrgb.pwm1_g1 == 1) // caso seja escolhido aumentar a cor verde
  {
    if(dutyCycle_g1 <=255 && dutyCycle_g1 >=0)
      {
         ledcWrite(pwmChannel_g1, dutyCycle_g1);  
         dutyCycle_g1 = dutyCycle_g1 + 15 ;
      }
  }
  else if (flagsrgb.pwm0_g1 == 1 ) //caso seja escolhido diminuir a cor verde
  {
    if(dutyCycle_g1 <=255 && dutyCycle_g1 >=0)
      {
         ledcWrite(pwmChannel_g1, dutyCycle_g1);  
         dutyCycle_g1 = dutyCycle_g1 - 15 ;
       }
  }
  else if (flagsrgb.pwm1_b1 == 1 ) // caso seja escolhido aumentar a cor azul
  {
    if(dutyCycle_b1 <=255 && dutyCycle_b1 >=0)
      {
         ledcWrite(pwmChannel_b1, dutyCycle_b1);  
         dutyCycle_b1 = dutyCycle_b1 + 15 ;
      }
  }
  else if (flagsrgb.pwm0_b1 == 1 ) //caso seja escolhido diminuir a cor azul
  {
    if(dutyCycle_b1 <=255 && dutyCycle_b1 >=0)
      {
         ledcWrite(pwmChannel_b1, dutyCycle_b1);  
         dutyCycle_b1 = dutyCycle_b1 - 15 ;
       }
  }
  else if (flagsrgb.pwm1_w1 == 1 ) // caso seja escolhido aumentar a cor branca
  {
    if((dutyCycle_r1 <=255 && dutyCycle_r1 >=0) &&(dutyCycle_g1 <=255 && dutyCycle_g1 >=0) && (dutyCycle_b1 <=255 && dutyCycle_b1 >=0))
      {
         ledcWrite(pwmChannel_r1, dutyCycle_r1);
         ledcWrite(pwmChannel_g1,dutyCycle_g1);  
         ledcWrite(pwmChannel_b1, dutyCycle_b1); 
         dutyCycle_r1 = dutyCycle_r1 + 15 ;
         dutyCycle_g1 = dutyCycle_g1 + 15 ;
         dutyCycle_b1 = dutyCycle_b1 + 15 ;
      }
  }
  else if (flagsrgb.pwm0_w1 == 1) //caso seja escolhido diminuir a cor branca
  {
    if((dutyCycle_r1 <=255 && dutyCycle_r1 >=0) &&(dutyCycle_g1 <=255 && dutyCycle_g1 >=0) && (dutyCycle_b1 <=255 && dutyCycle_b1 >=0))
      {
         ledcWrite(pwmChannel_r1, dutyCycle_r1);
         ledcWrite(pwmChannel_g1,dutyCycle_g1);  
         ledcWrite(pwmChannel_b1, dutyCycle_b1); 
         dutyCycle_r1 = dutyCycle_r1 - 15 ;
         dutyCycle_g1 = dutyCycle_g1 - 15 ;
         dutyCycle_b1 = dutyCycle_b1 - 15 ;
       }
  }
  else if (flagsrgb.pwm1_y1 == 1) // caso seja escolhido aumentar a cor amarela
  {
    if((dutyCycle_g1 <=255 && dutyCycle_g1 >=0) && (dutyCycle_b1 <=255 && dutyCycle_b1 >=0))
      {
         ledcWrite(pwmChannel_g1,dutyCycle_g1);  
         ledcWrite(pwmChannel_b1, dutyCycle_b1); 
         dutyCycle_g1 = dutyCycle_g1 + 15 ;
         dutyCycle_b1 = dutyCycle_b1 + 15 ;
      }
  }
  else if (flagsrgb.pwm0_y1 == 1) //caso seja escolhido diminuir a cor amarela
  {
    if((dutyCycle_g1 <=255 && dutyCycle_g1 >=0) && (dutyCycle_b1 <=255 && dutyCycle_b1 >=0))
      {
         ledcWrite(pwmChannel_g1,dutyCycle_g1);  
         ledcWrite(pwmChannel_b1, dutyCycle_b1); 
         dutyCycle_g1 = dutyCycle_g1 - 15 ;
         dutyCycle_b1 = dutyCycle_b1 - 15 ;
       }
  } 
}

void LED1OFF()
{
  if(flagsrgb.r1 == 1 && flagsrgb.turn_off == 1)
  {
     dutyCycle_r1 = 0; // colocar duty. da cor vermelha a 0
     ledcWrite(pwmChannel_r1,dutyCycle_r1);
     flagsrgb.turn_off = 0;
     flagsrgb.r1 = 0;
  }
  else if(flagsrgb.b1 == 1 && flagsrgb.turn_off == 1)
  {
     dutyCycle_b1 = 0; // colocar duty. da cor azul a 0
     ledcWrite(pwmChannel_b1,dutyCycle_b1);
     flagsrgb.turn_off = 0;
     flagsrgb.b1 = 0;
  }
  else if(flagsrgb.g1 == 1 && flagsrgb.turn_off == 1)
  {
     dutyCycle_g1 = 0; // colocar duty. da cor verde a 0
     ledcWrite(pwmChannel_g1,dutyCycle_g1);
     flagsrgb.turn_off = 0;
     flagsrgb.g1 = 0;
  }
  else if(flagsrgb.w1 == 1 && flagsrgb.turn_off == 1)
  {
     dutyCycle_r1 = 0; // colocar duty. da cor vermelha a 0
     dutyCycle_g1 = 0; // colocar duty. da cor verde a 0
     dutyCycle_b1 = 0; // colocar duty. da cor azul a 0
     ledcWrite(pwmChannel_r1,dutyCycle_r1);
     ledcWrite(pwmChannel_g1,dutyCycle_g1);
     ledcWrite(pwmChannel_b1,dutyCycle_b1);
     flagsrgb.turn_off = 0;
     flagsrgb.w1 = 0;
  }
  else if(flagsrgb.y1 == 1 && flagsrgb.turn_off == 1)
  {
     dutyCycle_g1 = 0; // colocar duty. da cor verde a 0
     dutyCycle_b1 = 0; // colocar duty. da cor azul a 0
     ledcWrite(pwmChannel_g1,dutyCycle_g1);
     ledcWrite(pwmChannel_b1,dutyCycle_b1);
     flagsrgb.turn_off = 0;
     flagsrgb.y1 = 0;
  }
}

void medir_temp_quarto() //função para medir a temperatura
{
  cont = millis();                             // contador para guardar o tempo que passa desde o inicio da medição de temperatura
  if (cont - tempoAnterior > (cont_sg * 1000)) // se o tempo que passou desde o inico da medição for inferior ao estipulado (2 segundos)
  {
    tempoAnterior = millis();                  // Atualiza a variável para a nova contagem de tempo, ara que a condição acima seja sempre sincronizada
    temperatura = dht.readTemperature();       // variável para guardar o valor da temperatura
    char tempString[8];                        // variável para guardar o valor da temperatura
    dtostrf(temperatura, 3, 4, tempString);    //converter o valor da temperatura em string
    client.publish(topic3, tempString);        // enviar a string que contem a temperatura através do topic3
  }
}

void controloVentoinha()
{
  if (flagsAc.ligarAC == 1)
  {
    digitalWrite(IN1_A, HIGH); 
    digitalWrite(IN2_A, LOW);
    ledcWrite(pwmChannelA, dutyCycleA); 
    int estado = 1;
    dtostrf(estado, 3 ,1, estadoAC);
    client.publish(topic8, estadoAC);
  }
  else if (flagsAc.desligarAC == 1)
  {
    digitalWrite(IN1_A, LOW); 
    digitalWrite(IN2_A, LOW);
    int estado = 0;
    dtostrf(estado, 3 ,1, estadoAC);
    client.publish(topic8, estadoAC);
  }
}

void setup()
{
  //Inicialização UART
  Serial.begin(115200);
  
  //iniciar o Sensor de Temperatura
  dht.begin();

  //Inicializar conexão WIFI
  conectarWIFI();
  
  //Inicializar MQTT
  client.setServer(mqtt_server, 1883); //Inicializar o server com o IP do raspberry no porto 1883
  client.setCallback(callback);        // inicializar a rotina do callback

  //Inicializações BLE
  fillDevices();
  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan -> setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(),false);
  pBLEScan -> setActiveScan(false);

  //Inicializações Sonar
  pinMode(trig1,OUTPUT); // definir trigger como saída
  pinMode(echo1,INPUT);  // definir Echo como entrada
 
  //Inicialização Leds
  pinMode(led, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(ledWC, OUTPUT);

  //inicializações para o LED_RGB_1 PWM
  ledcSetup(pwmChannel_r1, freq1, resolution1); // pino cor vermelha
  ledcAttachPin(red1, pwmChannel_r1);
  ledcSetup(pwmChannel_g1, freq1, resolution1); // pino cor verde
  ledcAttachPin(green1, pwmChannel_g1);
  ledcSetup(pwmChannel_b1, freq1, resolution1); // pino cor azul
  ledcAttachPin(blue1, pwmChannel_b1);

  //inicializações para o pwm - Ventoinha
  ledcSetup(pwmChannelA, freqA, resolutionA); // configura o canal 0 com uma frequência de 30KHz e uma resolução de 8 bits
  ledcAttachPin(EN_A, pwmChannelA);           // configura o pino EN_A (controlo da velociade de rotação), com as caracteristicas do canal 0
  ledcWrite(pwmChannelA, dutyCycleA);         // configurar velocidade de rotação
  
  //inicializaçoes pinos MotorGaragem - como saídas
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);

  //inicializações para o pwm - Motor Garagem
  ledcSetup(pwmChannelB, freqB, resolutionB); // configura o canal 1 com uma frequência de 30KHz e uma resolução de 8 bits
  ledcAttachPin(EN_B, pwmChannelB);           // configura o pino EN_B (controlo da velociade de rotação), com as caracteristicas do canal 1
  ledcWrite(pwmChannelB, dutyCycleB);         // configurar velocidade de rotação
  
  //inicializaçoes pinos MotorGaragem - como saídas
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  
  //inicializações Sensor infra-vermelho
  pinMode(infraGaragem_1, INPUT_PULLUP);
  pinMode(infraGaragem_2, INPUT_PULLUP);
  pinMode(infraGaragem_deteta, INPUT_PULLUP);

}

void loop()
{
  if (!client.connected()) //reconecta se a conexão se perdeu 
  {
   conectar_mqtt();
  }
  client.loop();
  scanDispositivo();
  processar_dist();
  veiculoGaragem();
  pwmLED1();
  colorLED1();
  LED1OFF();
  medir_temp_quarto();
  detetaCarroGaragem();
  controlarPortaGaragem_manual();
  controlarPortaGaragem_aut();
  controloVentoinha();
}
