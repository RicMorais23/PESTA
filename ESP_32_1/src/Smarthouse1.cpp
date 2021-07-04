// Bibliotecas
#include <Arduino.h>
#include "WiFi.h"             // Biblioteca  WI-Fi
#include "PubSubClient.h"
#include "DHT.h"              // Biblioteca  DHT11
#include <DHT_U.h>
#include <Stepper.h>         //  Biblioteca  Motor Passo

//--------------------------- Configurações Sensores de Temperatura---------------------------------------------//
                                                                                                                //
#define DHTTYPE DHT11       // definir o tipo de sensor a usar                                                  //
#define DHTPIN 0            // pino onde está conectado o output                                                //
#define cont_sg 60          // tempo definido para o sensor fazer a leitura e imprimir                          //
unsigned long cont = millis();                                                                                  //
unsigned long tempoAnterior = 0; // cria uma variavel que guarda o tempo anterior                               //
DHT dht(DHTPIN, DHTTYPE);        // cria um objeto para o sensor DHT, com o pino de saída e o tipo de sensor    //
                                                                                                                //
//--------------------------------------------------------------------------------------------------------------//

//---------------------------- Configurações Motores -----------------------------------------------//
                                                                                                    //
// Pinos MOTOR A - Controlo de Rotação                                                              //
#define IN1_A 14                                                                                    //
#define IN2_A 15                                                                                    //
#define EN_A 16                                                                                     // 
                                                                                                    //
// Configurações PWM - Motor A                                                                      //
const int freqA = 30000;   // definir uma frequência de 30KHz                                       //
const int pwmChannelA = 1; // canal PWM - 1                                                         //
const int resolutionA = 8; // resolução de ( 8 bits)                                                //
int dutyCycleA = 220;       // dutycycle inicia - 220                                               //
//--------------------------------------------------------------------------------------------------//                                                                                                    

//-------------------- Configurações Sensores Infra-vermelhos---------//
#define infra_1 2                                                     //
#define infra_2 4                                                     //
const int infra_porta = 5 ;                                           //
//--------------------------------------------------------------------//

//------------------------------------- Configurações Motor Passo_Porta_Entrada ------------------------------------------------------------ //  
#define IN1_mpA 17                                                                                                                           //
#define IN2_mpA 18                                                                                                                           //
#define IN3_mpA 19                                                                                                                           //
#define IN4_mpA 21                                                                                                                           //
                                                                                                                                             //
const int num_passos = 2038; //defenir o número de passos numa volta                                                                         //
//Configurações sentido de rotação do motor                                                                                                  //
Stepper motorA_passo_R = Stepper(num_passos, IN1_mpA, IN2_mpA, IN3_mpA, IN4_mpA);//comfiguração para o motor A ,sentido horário              //
Stepper motorA_passo_L = Stepper(num_passos, IN4_mpA, IN2_mpA, IN3_mpA, IN1_mpA);//comfiguração para o motor A ,sentido anti-horário         //
//-------------------------------------------------------------------------------------------------------------------------------------------//

//---------------------------------- Configurações Sensor de Movimento_Entrada -----------------------------//
#define temp_on_seg 15                                                                                      //
                                                                                                            //
const int sensor_movimento_entrada = 35; // definir GPIO35 para saída do sensor de Movimento                //
unsigned long temp_atual = millis(); // variavel usada para retornar o numero de milissegundos passados     //
unsigned long temp_antigo = 0;       // iniciar o tempo_antigo a 0                                          //
boolean cont_moviment = false;                                                                              //
//----------------------------------------------------------------------------------------------------------//

//--- Configurações Sensores LDR ----//
#define LDR1 34 // definir o pino LDR
int ldr1_valor; // variável onde se vai guardar o valor da leitura analogica

//----- Buzzer para alarme ------------------------------------//
#define buzzer 23 //Pino de ligação do buzzer                                                                       
//------- Pinos LEDS------------------//                                                                                                                    
#define ledSala 22
#define ledHall 12
#define ledEntrada 13
#define ledCozinha 25
#define ledGaragem 26

//-------------------- Configurações Timer 1 ------------------------------------------------------------------------------------//
volatile int counterInt; // definir o contador para a interrupção                                                                //
                                                                                                                                 //
hw_timer_t *timer = NULL; // estruta usada para definir a variavel timer usada mais à frente                                     //   
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // variável usada para tomar conta da sincronização entre a main e a ISR,  //
//quando se modifica uma variável partilhada                                                                                     //
                                                                                                                                 //
//-------------------------------------------------------------------------------------------------------------------------------//


//WIFI
const char *WIFI_Network = "ZON-D2E0";
const char *WIFI_Password = "fc1ce0717d08";

#define WIFI_Timeout_seg 60 // tempo definido para voltar a tentar ligar ao wifi

//MQTT
const char *mqtt_server = "192.168.1.6";
//Parâmetros USER, PASS, ID
const char *mqtt_user = "morais23";
const char *mqtt_pass = "Morais23";
const char *ID = "ESP32";

//Topics
#define topic1  "home/dados/network"
#define topic2  "home/iluminação"
#define topic3  "home/temp"
#define topic4  "home/humid"
#define topic5  "home/estore"
#define topic6  "home/porta/abrir"
#define topic7  "home/porta/fechar"
#define topic8  "home/alarme/desligar"
#define topic9  "home/alarme/ativar"
#define topic10 "home/porta/estado"
#define topic11 "home/estore/estado"
#define topic12 "home/Sensor/movimento"
#define topic13 "home/alarme/estado"
#define topic14 "home/luminosidade"

char estadoPortaPrincipal[10]; // variável para guardar o estado da Porta Principal
char estadoEstore[10];         // variável para guardar o estado do Estore
char sensorMovimento[10];      // variável para guardar estado do Sensor Movimento
char estadoAlarme[10];         // variável para guaradar o estado do alarme
char estadoLuminosidade[10];   // variável para guardar o estado da luminosidade

// STRUCT para flags
typedef struct
{
  unsigned char fechar_estA;   // flag que verifica se foi pressionado o botão para fechar o estore
  unsigned char abrir_estA;    // flag que verifica se foi pressionado o botão para abrir o estore
  unsigned char parar_estA;    // flag que verifica se foi pressionado o botão para parar o estore
  unsigned char abrir_porta;   // flag que verifica se foi pressionado o botão para abrir a Porta principal
  unsigned char porta_aberta;  // flag que verifica se a porta está aberta
  unsigned char fechar_porta;  // flag que verifica se foi pressionado o botão para fechar a Porta principal
  unsigned char porta_fechada; // flag que verifica se a porta está fechada
  unsigned char alarme;        // flag que verifica se o alarme está ou não ligado
  unsigned char interrpt;      // flag que indica se a interruoção pode ou nao acontecer

}flags_st;

volatile flags_st flags = {0, 0, 0, 0, 0, 0, 0, 1, 0}; // flags 

// Struct para o sensor LDR

typedef struct 
{
  unsigned char lumi_mt_alta; // flag que indica que a luminosidade está muito alta
  unsigned char lumi_alta;    // flag que indica que a luminosidade está alta
  unsigned char lumi_med;     // flag que indica que a luminosidade está média
  unsigned char lumi_baixa;   // flag que indica que a luminosidade está baixa
  unsigned char lumi_mt_baixa;// flag que indica que a luminosidade está muito baixa
  unsigned char lumi_nula;    // flag que indica que a luminosidade é nula
}flagsldr_st;

volatile flagsldr_st flagsldr = {0, 0, 0, 0, 0, 0}; // flags iniciam todas a 0
 

//Iniciar objetos WIFI e o MQTT Client
WiFiClient wifiClient;
//PubSubClient client (mqtt_server, 1883, wifiClient);
PubSubClient client(wifiClient);

void conectarWIFI() // Função que estabelece a conexão WIFI
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

  while (WiFi.status() != WL_CONNECTED && (millis() - startconect) < WIFI_Timeout_seg * 1000) // Enquanto a conexão ainda n estiver estabelecida e o tempo que passou desde a tentativa de conexão for inferior à estabelecida (2s)
  {
    Serial.print("\n");
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("\n");
    Serial.println(" Falhou!");
    // Executar mais uma ação
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


void conectar_mqtt() //função para fazer a ligação ao mqtt
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
      client.subscribe(topic5);
      client.subscribe(topic6);
      client.subscribe(topic7);
      client.subscribe(topic8);
      client.subscribe(topic9);
    }
    else
    {
      Serial.println(" Tentar de novo em 5 segundos");
      // Espera 5 segundos e tenta de novo
      delay(5000);
    }
  }
}

void callback(char *topic, byte *mensagem, unsigned int length) // função que recebe as mensagens de cada tópico correspondente
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

   if(String(topic) == topic1)
   {
    Serial.println(resposta);

   } 

   else if (String(topic) == topic2)
  {
    if (resposta == "ligar sala") 
    {
      digitalWrite(ledSala, HIGH);
    }
    else if (resposta == "desligar sala") 
    {
      digitalWrite(ledSala, LOW);
    }
    else if (resposta == "ligar hall")
    {
      digitalWrite(ledHall, HIGH);
    }
    else if (resposta == "desligar hall")
    {
      digitalWrite(ledHall, LOW);
    }
    else if (resposta == "ligar entrada")
    {
      digitalWrite(ledEntrada, HIGH);
    }
    else if (resposta == "desligar entrada")
    {
      digitalWrite(ledEntrada, LOW);
    }
    else if (resposta == "ligar cozinha")
    {
      digitalWrite(ledCozinha, HIGH);
    }
    else if (resposta == "desligar cozinha")
    {
      digitalWrite(ledCozinha, LOW);
    }
     else if (resposta == "ligar garagem")
    {
      digitalWrite(ledGaragem, HIGH);
    }
    else if (resposta == "desligar garagem")
    {
      digitalWrite(ledGaragem, LOW);
    }
    else if (resposta == "desligar tudo")
    {
      digitalWrite(ledSala, LOW);
      digitalWrite(ledHall, LOW);
      digitalWrite(ledEntrada, LOW);
      digitalWrite(ledCozinha, LOW);
      digitalWrite(ledGaragem, LOW); 
    }
  }
 else if (String(topic) == topic5)
  {
      if (resposta == "fechar estore A" && flags.fechar_estA == 0)
      {
        flags.fechar_estA = 1;
      }
      else if (resposta == "parar estore A")
      {
        flags.parar_estA = 1;
      }
      else if (resposta == "abrir estore A" && flags.abrir_estA == 0)
      {
        flags.abrir_estA = 1;
      }
  }
 else if (String(topic) == topic6)
  {
    if (resposta == "abrir porta" && flags.abrir_porta == 0)
    {
      flags.abrir_porta = 1;
    }
  } 
 else if (String(topic) == topic7)
  {
    if(resposta == "fechar porta" && flags.fechar_porta == 0)
    {
      flags.fechar_porta = 1;
    }
  }  
 else if(String(topic) == topic8)
  {
    if(resposta == "desligar alarme" && flags.alarme == 1)
    {
      flags.alarme = 0;
    }
  }
 else if(String(topic) == topic9)
   {
     if (resposta == "ativar alarme" && flags.alarme == 0)
    {
      flags.alarme = 1; // coloca a flag do alarme a 1
      //O contador volta ao valor inicial, pronto a ser decrementado
      counterInt = 15;
      flags.interrpt = 0;
      int estado = 1;
      dtostrf(estado, 3 ,1, estadoAlarme);
      client.publish(topic13, estadoAlarme);
    }
  }
 
}  

void IRAM_ATTR interrupt0() // função onde ocorre a interrupção
{
  if(flags.interrpt == 1) // verifica se pode decerementar o contador
  {
  // Principal função decrementar o contador
  portENTER_CRITICAL_ISR (&timerMux);
  counterInt--;
  portEXIT_CRITICAL_ISR (&timerMux);
  }
}

void medir_temp() //função para medir a temperatura
{
  cont = millis();                             // contador para guardar o tempo que passa desde o inicio da medição de temperatura
  if (cont - tempoAnterior > (cont_sg * 1000)) // se o tempo que passou desde o inico da medição for inferior ao estipulado (2 segundos)
  {
    tempoAnterior = millis();                  // Atualiza a variável para a nova contagem de tempo, ara que a condição acima seja sempre sincronizada
    float humidade = dht.readHumidity();       // variável para guardar o valor da humidade
    float temperatura = dht.readTemperature(); // variável para guardar o valor da temperatura
    char tempString[8];                        // variável para guardar o valor da temperatura
    char humidString[8];                       // variável para guardar o valor da humidade

    dtostrf(temperatura, 3, 4, tempString);  // converter o valor da temperatura em string
    dtostrf(humidade, 3, 4, humidString);    // converter o valor da Humidade em string

    client.publish(topic3, tempString);  // enviar a string que contem a temperatura através do topic3
    client.publish(topic4, humidString); // enviar a string que contem a humidade através do topic4
  }
}

void controlarEstore() //função para controlar o estore
{
    if(digitalRead(infra_1) == LOW && digitalRead(infra_2) == HIGH && flags.fechar_estA == 1) 
        {  
          // colocar o motor a rodar no sentido de fechar o estore
          digitalWrite(IN1_A, HIGH); 
          digitalWrite(IN2_A, LOW);
          //definir velocidade
          ledcWrite(pwmChannelA, dutyCycleA); 
          //Enviar Estado
          int estado = 0;
          dtostrf(estado, 3 ,1, estadoEstore);
          client.publish(topic11, estadoEstore);
        }
        else if (digitalRead(infra_2) == LOW && flags.fechar_estA == 1)  
        {
          // parar o motor
          digitalWrite(IN1_A, LOW);
          digitalWrite(IN2_A, LOW);
          flags.fechar_estA = 0; // colocar a flag de fechar o estore a 0
          flags.parar_estA = 0; // colocar a flag de parar o estore a 0 
          //Enviar Estado
          int estado = 1;
          dtostrf(estado, 3 ,1, estadoEstore);
          client.publish(topic11, estadoEstore);
        }
        else if (digitalRead(infra_2) == LOW && flags.abrir_estA == 1) 
        {
          // colocar o motor a rodar no sentido de abrir o estore
          digitalWrite(IN1_A, LOW); 
          digitalWrite(IN2_A, HIGH);
          //definir velocidade
          ledcWrite(pwmChannelA, dutyCycleA); 
          //Enviar Estado
          int estado = 2;
          dtostrf(estado, 3 ,1, estadoEstore);
          client.publish(topic11, estadoEstore);
        }
       
        else if (digitalRead(infra_1) == LOW && flags.abrir_estA == 1) 
       {
         // o motor vai parar, pq o estore está completamente aberto
          digitalWrite(IN1_A, LOW); 
          digitalWrite(IN2_A, LOW);
          flags.abrir_estA = 0; //colocar a flag de abrir o estore a 0
          //Enviar Estado
          int estado = 3;
          dtostrf(estado, 3 ,1, estadoEstore);
          client.publish(topic11, estadoEstore);
       }   
}

void pararEstore()
{ 
  if (flags.parar_estA == 1) //pressionado o botão de parar o estore, a flag vai a 1
  {
    flags.abrir_estA = 0; // flag de abrir o estore fica a 0
    flags.fechar_estA = 0;// flag de fechar o estore fica a 0
    // Para o motor
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);
    flags.parar_estA = 0; //flag de parar o estore fica a 0
    //Enviar Estado
      int estado = 4;
      dtostrf(estado, 3 ,1, estadoEstore);
      client.publish(topic11, estadoEstore);
  }
}

void detetaPorta() // função que deteta se a porta está ou não aberta
{
  if(digitalRead(infra_porta) == LOW) // se o sensor  estiver interrumpido 
  {
    flags.porta_aberta = 1;  // A porta está aberta
    flags.porta_fechada = 0; // Flag porta fechada vai a 0
    // Envio do estado
    int estado = 1;
    dtostrf(estado, 3 ,1, estadoPortaPrincipal);
    client.publish(topic10, estadoPortaPrincipal);
  }
  else if(digitalRead(infra_porta) == HIGH) // se o sensor n estiver interrumpido
  {
    flags.porta_fechada = 1; // A porta está fechada
    flags.porta_aberta = 0; // Flag da porta aberta vai a 0
    // Envio do estado
    int estado = 0;
    dtostrf(estado, 3 ,1, estadoPortaPrincipal);
    client.publish(topic10, estadoPortaPrincipal);
  }
}

void abrir_fechar_porta() // função para abrir ou fechar a porta
{
  if (flags.abrir_porta == 1 && flags.porta_fechada == 1) // se  for acionado a abertura da porta
  {
    //definir velocidade de rotação [1-10]
    motorA_passo_L.setSpeed(5);
     //definir numero de passos a dar para a abertura da porta, no sentido de abertura
    motorA_passo_L.step(-num_passos/3);
    //colocar flag abrir porta a 0
    flags.abrir_porta = 0;
  }
  else if (flags.fechar_porta == 1 && flags.porta_aberta == 1) // se for acionado fechar a porta
  {
    //definir velocidade de rotação [1-10]
    motorA_passo_R.setSpeed(5);
    //definir numero de passos a dar par fechar a porta, no sentido contrário à abertura
    motorA_passo_R.step(num_passos/3);
    //colocar flag abrir porta a 0
    flags.fechar_porta = 0;
  }
  else if (flags.fechar_porta == 1 && flags.porta_fechada == 1) // se a porta estiver fechada e for pressionado o botão para fechar
  {
  //colocar a flag de fechar a porta a 0, para não acontecer nada
    flags.fechar_porta = 0;
  }
  else if(flags.abrir_porta ==1 && flags.porta_aberta == 1) // se a porta estiver aberta e for pressionado o botão para abrir
  {
    // colocar a flag de abrir a porta a 0, para não acontecer nada
    flags.abrir_porta = 0;
  }
}

void movimento_detetado_entrada() // função que vai desenpenhar todo o processo depois de ser detetado movimento pelo sensor
{
  Serial.println("Foi detetado movimento");
  //Enviar Estado
    int estado = 1;
    dtostrf(estado, 3 ,1, sensorMovimento);
    client.publish(topic12, sensorMovimento);
  if(flagsldr.lumi_mt_alta == 1 || flagsldr.lumi_alta == 1 || flagsldr.lumi_med == 1)
  {
    digitalWrite(ledEntrada, LOW);
    cont_moviment = true; //inicia o contador quando é detetado movimento
    temp_antigo = millis(); // guarda o tempo em millissegundos que já passaram 
  }
  
  else if(flagsldr.lumi_baixa == 1 || flagsldr.lumi_mt_baixa == 1 || flagsldr.lumi_nula == 1)
  {
    digitalWrite(ledEntrada, HIGH);
    cont_moviment = true; //inicia o contador quando é detetado movimento
    temp_antigo = millis(); // guarda o tempo em millissegundos que já passaram
  }
  
}

void sensor_mov_entrada() 
{
  temp_atual = millis(); //guardar o valor do contador
  //Desligar as luzes depois de passarem o número de segundos definidos
  if(cont_moviment == true && (temp_atual - temp_antigo > (temp_on_seg*1000))) 
  {
    Serial.println("Motion stopped...");
    cont_moviment = false;
  }
}

void controlo_luminosidade()
{
  ldr1_valor = analogRead(LDR1); // guarda o valor da leitra na variável

  // Sequência de condições para ativar as flags consoante a luminosidade
  if (ldr1_valor <= 270) 
  {
    // coloca a flag de luminosidade mt alta a 1
    flagsldr.lumi_mt_alta = 1 ;

    // Envio da luminosidade
    int estado = 5;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
    
    //coloca as restantes a 0
    flagsldr.lumi_alta = 0;
    flagsldr.lumi_med  = 0;
    flagsldr.lumi_baixa = 0;
    flagsldr.lumi_mt_baixa = 0;
    flagsldr.lumi_nula = 0;

  }
  else if (ldr1_valor > 270 && ldr1_valor <= 570)
  {
    // coloca a flag de luminosidade alta a 1
    flagsldr.lumi_alta = 1;
    
    // Envio da luminosidade
    int estado = 4;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
    
    // coloca as restantes a 0
    flagsldr.lumi_mt_alta = 0; 
    flagsldr.lumi_med   = 0;
    flagsldr.lumi_baixa = 0;
    flagsldr.lumi_mt_baixa = 0;
    flagsldr.lumi_nula = 0;
  }
  else if (ldr1_valor > 570 && ldr1_valor <= 960)
  {
    // dá reset a todas as outras
    flagsldr.lumi_mt_alta = 0;
    flagsldr.lumi_alta  = 0;
    flagsldr.lumi_baixa = 0;
    flagsldr.lumi_mt_baixa = 0;
    flagsldr.lumi_nula = 0;

    // coloca a flag de luminosidade média a 1
    flagsldr.lumi_med = 1;
    
    // Envio da luminosidade
    int estado = 3;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
  }
  else if (ldr1_valor > 960 && ldr1_valor <= 1670)
  {
    // dá reset a todas as outras
    flagsldr.lumi_mt_alta = 0;
    flagsldr.lumi_alta  = 0;
    flagsldr.lumi_med   = 0;
    flagsldr.lumi_mt_baixa = 0;
    flagsldr.lumi_nula = 0;
    
    // coloca a flag de luminosidade baixa a 1
    flagsldr.lumi_baixa = 1;
    
    // Envio da luminosidade
    int estado = 2;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
  }
  else if (ldr1_valor > 1670 && ldr1_valor <= 3500)
  {
    // dá reset a todas as outras
    flagsldr.lumi_mt_alta = 0;
    flagsldr.lumi_alta  = 0;
    flagsldr.lumi_med   = 0;
    flagsldr.lumi_baixa = 0;
    flagsldr.lumi_nula = 0;

     // coloca a flag de luminosidade mt baixa a 1
     flagsldr.lumi_mt_baixa = 1;
     
    // Envio da luminosidade
    int estado = 1;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
  }
  else if (ldr1_valor > 3500 && ldr1_valor <= 4100)
  {
     // dá reset a todas as outras
    flagsldr.lumi_mt_alta = 0;
    flagsldr.lumi_alta  = 0;
    flagsldr.lumi_med   = 0;
    flagsldr.lumi_baixa = 0;
    flagsldr.lumi_mt_baixa = 0;

    // coloca a flag de luminosidade nula a 1
    flagsldr.lumi_nula = 1;
    
    // Envio da luminosidade
    int estado = 0;
    dtostrf(estado, 3 ,1, estadoLuminosidade);
    client.publish(topic14, estadoLuminosidade);
  }

}

void sistema_alarme()
{
  if(flags.porta_aberta == 1 && flags.alarme == 1) // verifica se o alarme está ativo e a porta está aberta
  {
    timerAlarmEnable(timer);  // inicia o timer.
    flags.interrpt = 1;       // coloca a flag de interrupção a 1
   if (counterInt > 0)        // se o valor do contador for maior q 0
    {
      digitalWrite(buzzer, LOW); // o buzzer continua desligado

    }
    else if (counterInt == 0) // se o contador chegar a 0
    {
       timerAlarmDisable(timer); // para o Contador
      flags.interrpt = 0; // dá reset na flag
      //colocar buzzer atuado, para simular alarme
      digitalWrite(buzzer, HIGH);
      int estado = 2;
      dtostrf(estado, 3 ,1, estadoAlarme);
      client.publish(topic13, estadoAlarme);
    } 
  }  
  else if (flags.alarme == 0) // se a flag do alarmes está a 0 -> foi pressionado o botão desligar alarme
    {
      // desligar o buzzer
      digitalWrite(buzzer, LOW);
      int estado = 0;
      dtostrf(estado, 3 ,1, estadoAlarme);
      client.publish(topic13, estadoAlarme);
    }  
} 

void setup()
{
  Serial.begin(115200);
  //Inicializar conexão WIFI
  conectarWIFI();
  //Inicializar MQTT
  client.setServer(mqtt_server, 1883); //Inicializar o server com o IP do raspberry no porto 1883
  client.setCallback(callback);        // inicializar a rotina do callback
  
  //iniciar o Sensor de Temperatura
  dht.begin();

  //inicializações para o pwm - MotorA
  ledcSetup(pwmChannelA, freqA, resolutionA); // configura o canal 0 com uma frequência de 30KHz e uma resolução de 8 bits
  ledcAttachPin(EN_A, pwmChannelA);           // configura o pino EN_A (controlo da velociade de rotação), com as caracteristicas do canal 0
  ledcWrite(pwmChannelA, dutyCycleA);         // configurar velocidade de rotação
  
  //inicializaçoes pinos MotorA- como saídas
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);

  //inicializações Sensor infra-vermelho
  pinMode(infra_1, INPUT_PULLUP);
  pinMode(infra_2, INPUT_PULLUP);
  pinMode(infra_porta, INPUT_PULLUP);

  //inicializações Sensor de movimento - Entrada
  pinMode(sensor_movimento_entrada, INPUT_PULLUP); //definir sensor de movimento como uma entrada com as resitências PULLUP ligadas
  attachInterrupt(digitalPinToInterrupt(sensor_movimento_entrada), movimento_detetado_entrada, RISING); //função usada para que ocorra uma interrupção no pino do sensor de movimento, no modo "RISING", ou seja ocorre o trigger quando o pino passa de low para high

  //inicializar buzzer
   pinMode(buzzer, OUTPUT);

  //inicializações Timer1
  counterInt = 15;// iniciar o contador do Timer para tempo de espera de 15s
  timer = timerBegin(0, 80, true);// 0-> timer0, 80->Prescaler, true-> count up
  timerAttachInterrupt(timer, &interrupt0, true);// função recebe um apontador para inicializar o timer, que guardamos na variavel global(timer),
  //o endereço na qual queremos que ocorra a interrupção(interrupt0) e uma flag(true)
  timerAlarmWrite(timer, 1000000, true);// função usada para especificar o valor do contador, em q a interrupção vai ocorrer
  //timer-> apontador para a variável, 1000000->valor do contador, para dar uma interupt de 1 s, true-> flag para q a interrup ocorra ciclicamente
  
  //Inicializações leds 
  pinMode(ledSala, OUTPUT);
  pinMode(ledHall, OUTPUT);
  pinMode(ledEntrada, OUTPUT);
  pinMode(ledCozinha, OUTPUT);
  pinMode(ledGaragem, OUTPUT);
}

void loop() //função main
{
  if (!client.connected()) //reconecta se a conexão se perdeu 
  {
   conectar_mqtt();
  }
  client.loop();
  medir_temp();
  controlarEstore();
  pararEstore();
  detetaPorta();
  abrir_fechar_porta();
  sensor_mov_entrada();
  controlo_luminosidade();
  sistema_alarme();
  
}