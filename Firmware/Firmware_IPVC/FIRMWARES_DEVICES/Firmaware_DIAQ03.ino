//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//                            Firmware - IAQ4CLASSROOM                                  //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
// -> Sensores usados:                                                                  //
//                                                                                      //
//    * DHT22 - Sensor com o objetivo de recolher os dados referentes à                 //
//              temperatura e humidade na sala de aula.                                 //
//    * SPS30 - Sensor com o objetivo de recolher os dados referentes às                //
//              particulas de diversos diâmetros presentes no ar.                       //
//    * SGP30 - Sensor que é capaz de medir os componentes orgánicos                    //
//              voláteis (TVOC) sendo também capaz de medir os valores de eCO2          //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
// -> Tecnologias usadas:                                                               //
//                                                                                      //
//    * LoRaWAN - Comunicação dos dados recolhidos pelos sensores.                      //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
// -> Descrição/Objetivo:                                                               //
//                                                                                      //
//    * Projeto com o intuito de medir a qualidade do ar no interior                    //
//      das saulas de aulo com recurso a diversos sensores                              //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
// -> Autor:                                                                            //
//                                                                                      //
//    * Jorge Manuel Silva                                                              //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
// -> Last Firmware Update:                                                             //
//                                                                                      //
//    * 02/05/2022                                                                      //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "sps30.h"


// create constructor
Adafruit_SGP30 sgp;
// create constructor
SPS30 sps30;

/*  DEFINE ID ESP32 */
#define MY_ID   "DIAQ03"

/*  SPS30 CONFS */
#define SPS30_COMMS I2C_COMMS
#define TX_PIN 0
#define RX_PIN 0
/////////////////////////////////////////////////////////////
/* define driver debug
 * 0 : no messages
 * 1 : request sending and receiving
 * 2 : request sending and receiving + show protocol errors */
 //////////////////////////////////////////////////////////////
#define DEBUG 2 // Código original estava a 0 
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfoSps30();
bool spsFunc();
//void SetAutoClean();


/*  DHT22 CONFS */
#define DHTPIN 27            // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22       // DHT 22
DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT sensor.



/*  LED RGB CONFS   */
#define LEDR 4        // PINOUT NO PROJ ->    4
#define LEDG 5        // PINOUT NO PROJ ->    5
#define LEDB 18       // PINOUT NO PROJ ->    18
int ledcolor = 0;
int a = 1000; //this sets how long the stays one color for
String ColorToText = "";


/*  Variaveis Globais   */

    // Data send
    String dados;

    // For DHT22
    float temp=0, hum=0;
    int cntdht=0;

    // For SGP 30
    String tvoc, eco2;
    int cntsgp=0;

    // For SPS 30
    String ValuePM1, ValuePM2, ValuePM10;  
    // int cntSPS30 = 0;

    // Errors Part | If value = 1 -> Error in Sensor
    int 
        // DHT22
        error_dht = 0,
          // Variaveis subjacentes
          ErrorDataRead = 0,
          ErrorCheck = 0;
          // Mensagens de Erro
          String MessageErrorDHT = "",
          MessageErrorDHT1 = "",
          MessageErrorDHT2 = "";

        // SGP30
        int error_sgp = 0, 
          // Variaveis subjacentes
          ErrorNotFound = 0,
          ErrorMeasurementFailed = 0,
          ErrorFailedRead = 0;
          // Mensagens de Erro
          String MessageErrorSGP = "",
          MessageErrorSGP1 = "",
          MessageErrorSGP2 = "",
          MessageErrorSGP3 = "";

        // SPS30 - Variavel Principal
        int error_sps = 0,
          // Variaveis subjacentes
          ErrorCodeCommChann = 0,
          ErrorCodeConnect = 0,
          ErrorCodeReset = 0,
          ErrorCodeMeasurement = 0,
          ErrorCodeGetSerialNumber = 0,
          ErrorCodeGetProductName = 0,
          ErrorCodeReadValues= 0;
          // Mensagens de Erro
          String MessageError = "",
          MessageError1 = "",
          MessageError2 = "",
          MessageError3 = "",
          MessageError4 = "",
          MessageError5 = "",
          MessageError6 = "",
          MessageError7 = "";

    // Cor do Led
        /*
        0 : Azul - Default
        1 : Amarelo - MODERATE
        2 : Laranja - UNHEALTHY FOR SENTIVE GROUP
        3 : Vermelho - UNHEALTHY
        4 : Verde - GOOD
        */
    int LedColour = 0;
    

/*  LoRaWAN */

    #define SEND_BY_TIMER 2 // Send a message every TX_INTERVAL seconds
    
    // Chaves referentes ao Device - DIAQ03

    static const PROGMEM u1_t NWKSKEY[16] = { 0x46, 0x13, 0xe5, 0x40, 0xbd, 0x5c, 0xe0, 0x91, 0x8b, 0x73, 0x0a, 0xc0, 0x97, 0x2c, 0x88, 0x38 };
    static const u1_t PROGMEM APPSKEY[16] = { 0x6d, 0xf1, 0xa7, 0xab, 0xc9, 0xd6, 0x4d, 0x4b, 0xb4, 0x30, 0x69, 0xd1, 0x5a, 0x27, 0xf6, 0xae };
    static const u4_t DEVADDR = 0x04bf80aa;

    void os_getArtEui (u1_t* buf) { }
    void os_getDevEui (u1_t* buf) { }
    void os_getDevKey (u1_t* buf) { }

    static uint8_t mydata[100];
    static osjob_t sendjob;

    // Pin mapping for the SparkX ESP32 LoRa 1-CH Gateway
    const lmic_pinmap lmic_pins = {
      .nss = 16,
      .rxtx = LMIC_UNUSED_PIN,
      .rst = 5,
      .dio = {26, 33, 32},
    };

    // If send-by-timer is enabled, define a tx interval
    #ifdef SEND_BY_TIMER
    #define TX_INTERVAL 60 // Message send interval in seconds
    #endif

    // State machine event handler
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      // digitalWrite(LED_BUILTIN, LOW); // Turn off LED after send is complete
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
#ifdef SEND_BY_TIMER
      // Schedule the next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      INIT DHT22 PROCESS                                                                  //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ErrorDHT(float Temperature, float Humidity)
{
    // Verificar o contador
    if (cntdht < 2)
    {
        // Check if any reads failed and exit early (to try again).
        if (isnan(Temperature) || isnan(Humidity)) {
            // Incrementa o contador e tenta ler os valores outra vez
            cntdht++;
            //Serial.println(F("Failed to read from DHT sensor!"));
            Serial.println(F("Error data read..."));
            error_dht = 1; ErrorDataRead = 1;
            dht22();
            //return;
        }
        else
        {
            // Se ler corretamente os valores mete o contador a ZERO 
            cntdht=0;
            error_dht = 0;
        }
    }
    else
    {
        // Sensor já excedeu as tentivas de erro | Enviar mensagem de erro para Dash 
            // Printar na console 
            Serial.println(F("Failed to read from DHT sensor! | Possible error, check sensor!"));
            // Adicionar report do erro à string de dados
            error_dht = 2; ErrorCheck = 1;
            
        // Voltar a meter a variavel do contador a ZERO
        cntdht = 0;
    }
}


/*  DHT22 FUNCTION  */
void dht22()
{
    // Read Temperature
    temp = dht.readTemperature();

    // Read Humidity
    hum = dht.readHumidity();

    // Check Erros
    ErrorDHT(temp, hum);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      END DHT22 PROCESS                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      INIT SGP30 PROCESS                                                                  //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ErrorSGP()
{
    // Verificar o contador
    if (cntsgp < 2)
    {
        // Check if any reads failed and exit early (to try again).
        if (!sgp.IAQmeasure()) {
            // Incrementa o contador e tenta ler os valores outra vez
            cntsgp++;
            Serial.println(F("Measurement failed..."));
            error_sps = 2; ErrorMeasurementFailed = 1;
            sgpFunc();
        }
        else
        {
            // Se ler corretamente os valores mete o contador a ZERO 
            cntsgp = 0;
            error_sgp = 0;
        }
    }
    else
    {
        // Sensor já excedeu as tentivas de erro | Avisar USER (PISCAR LED) | Enviar mensagem de erro para Dash 

            // Printar na console 
            Serial.println(F("Failed to read from SGP30 sensor!"));
            error_sps = 3; ErrorFailedRead = 1;
            // Adicionar report do erro à string de dados
            error_sgp = 1;
            
        // Voltar a meter a variavel do contador a ZERO
        cntsgp = 0;
    }
}

/*  SGP30 FUNCTION  */
void sgpFunc()
{
    // Check Erros
    ErrorSGP();

    // Copia para as variaveis globais o valor lido
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      END SGP30 PROCESS                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       INIT SPS30 PROCESS                                                                 //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GetDeviceInfoSps30()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    Serial.print(F("Serial number : "));
    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
  {
    ErrtoMess((char *) "could not get serial number", ret);
    error_sps = 5; ErrorCodeGetSerialNumber = 1;
  }
  
  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("Product name  : "));

    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
  {
    ErrtoMess((char *) "could not get product name.", ret);
    error_sps = 6; ErrorCodeGetProductName = 1;
  }
    

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    Serial.println(F("Can not read version info"));
    return;
  }

  Serial.print(F("Firmware level: "));  Serial.print(v.major);
  Serial.print("."); Serial.println(v.minor);

  if (SPS30_COMMS != I2C_COMMS) {
    Serial.print(F("Hardware level: ")); Serial.println(v.HW_version);

    Serial.print(F("SHDLC protocol: ")); Serial.print(v.SHDLC_major);
    Serial.print("."); Serial.println(v.SHDLC_minor);
  }

  Serial.print(F("Library level : "));  Serial.print(v.DRV_major);
  Serial.print(".");  Serial.println(v.DRV_minor);
}

/**
 * @brief : read and display all values
 */
bool spsFunc()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          error_sps = 7; ErrorCodeReadValues= 1; 
          return(false);
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      error_sps = 7; ErrorCodeReadValues= 1; 
      return(false);
    }

  } while (ret != ERR_OK);

  // only print header first time
  if (header) {
    Serial.println(F("-------------Mass -----------    ------------- Number --------------   -Average-"));
    Serial.println(F("     Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
    Serial.println(F("P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
    header = false;
  } 

  Serial.print(val.MassPM1);
  ValuePM1 = val.MassPM1;
  //dados += ("&a="); dados+= (val.MassPM1);
  Serial.print(F("\t"));
  Serial.print(val.MassPM2);
  ValuePM2 = val.MassPM2;
  //dados += ("&b="); dados+= (val.MassPM2);
  Serial.print(F("\t"));
  Serial.print(val.MassPM4);
  Serial.print(F("\t"));
  Serial.print(val.MassPM10);
  ValuePM10 = val.MassPM10;
  //dados += ("&c="); dados+= (val.MassPM10);
  Serial.print(F("\t"));
  Serial.print(val.NumPM0);
  Serial.print(F("\t"));
  Serial.print(val.NumPM1);
  Serial.print(F("\t"));
  Serial.print(val.NumPM2);
  Serial.print(F("\t"));
  Serial.print(val.NumPM4);
  Serial.print(F("\t"));
  Serial.print(val.NumPM10);
  Serial.print(F("\t"));
  Serial.print(val.PartSize);
  Serial.print(F("\n"));

  // Incrementa o contador das vezes que já leu consequentemente
  // cntSPS30++;

  return(true);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  // Comentar os dois comandos abaixo
  //Serial.println(F("Program on hold"));
  //for(;;) delay(100000);                        // Este é o problema de quando não há SPS30, o firmware deixa de funcionar !!!!  
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */

void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       END SPS30 PROCESS                                                                  //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      INIT CREATE DATA PROCESS                                                            //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CreateDados()
{
    // Inicio da String dados
    dados += ("ID="); dados += (MY_ID);

    // Add Temperature
    dados += ("&T="); dados += (temp); 

    // Add Humidity
    dados += ("&H="); dados += (hum); 

    // Add TVOC
    dados += ("&t="); dados += (tvoc); 

    // Add eCO2
    dados += ("&CO2="); dados += (eco2); 
    
    // Add PM 1.0
    dados += ("&a="); dados+= (ValuePM1);

    // Add PM 2.5
    dados += ("&b="); dados+= (ValuePM2);

    // Add PM 10.0
    dados += ("&c="); dados+= (ValuePM10);

    // Add Colour LED
    dados += ("&l="); dados+= (LedColour);
    
    /*
        Errors Part
    */
   
    // DHT22 Part - Caso o valor da váriavel seja diferente de zero é enviado o valor da mesma
    if(error_dht != 0)
    {
      dados += ("&ErDht="); dados += (error_dht);   // Erro do dht22 
    }
    // SPS30 Part - Caso o valor da váriavel seja diferente de zero é enviado o valor da mesma
    if(error_sps != 0)
    {
      dados += ("&ErSps="); dados += (error_sps);   // Erro do SPS30
    } 

    // SGP30 Part - Caso o valor da váriavel seja diferente de zero é enviado o valor da mesma
    if(error_sgp != 0)
    {
      dados += ("&ErSgp="); dados += (error_sgp);   // Erro do SGP30 
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      END CREATE DATA PROCESS                                                             //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      INIT OUTPUT USER PROCESS                                                            //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////
    //                                        //
    //   Função para atribuir a cor ao LED    //
    //                                        //
    ////////////////////////////////////////////

void setColor(int R, int G, int B) {
  digitalWrite(LEDR, R);
  digitalWrite(LEDG, G);
  digitalWrite(LEDB, B);
}

void AirQual()
{
    //  Compare the values to good conditions and show to user 
        /*
                Se a humidade for a baixo dos 20% e qualquer temp -> Mau
                Se a humidade estiver entre os 20 e os 85%:
                    - Mas a temp abaixo dos 19ºC -> Mau
                    - Mas se a temp for entre os 19 e os 27ºC -> Bom
                    - Mas se a temp for acima dos 27ºC -> Mau
                Se a humidade for acima dos 85% e qualquer temp -> Mau
            Nós lemos as PM1 , PM2.5 e PM10 e TVOC
            Se  0.0 >= PM2.5 <= 12.0 && 0.0 >= PM10 <= 54 && 0.0 >= TVOC <= 0.5 -> GOOD                         (GREEN)
            Se 12.1 >= PM2.5 <= 35.4 && 55 >= PM10 <= 154 && TVOC > 0.5 -> MODERATE                             (YELLOW)
            Se 35.5 >= PM2.5 <= 55.4 && 155 >= PM10 <= 254 && TVOC > 0.5 -> UNHEALTHY FOR SENTIVE GROUP         (ORANGE)
            Se PM2.5 >= 55.5 && PM10 >= 255 && TVOC > 0.5 -> UNHEALTHY                                          (RED)
                ATENÇÃO !!! FALTA A PARTE DO CO2 (https://www.ebay.com/itm/112691626960?mkevt=1&siteid=1&mkcid=2&mkrid=711-153320-877651-5&source_name=google&mktype=pla_ssc&campaignid=11826484955&groupid=114180021839&targeted=pla-293946777986&MT_ID=&adpos=&device=c&googleloc=1011776&itemid=112691626960&merchantid=116792603&geo_id=164&gclid=EAIaIQobChMIte_IhKba9QIViQUGAB0nfQFtEAYYAyABEgIPf_D_BwE)
        */

       /*
        0 : Azul - Default
          // color code #0000FF (R = 0,  G = 0, B = 255)
          // setColor(0, 0, 255);
        1 : Amarelo - MODERATE
          // color code #FBFF00 (R = 251,  G = 255, B = 0)
          // setColor(251, 255, 0);
        2 : Laranja - UNHEALTHY FOR SENTIVE GROUP
          // color code #FFAA00 (R = 255,  G = 170, B = 0)
          // setColor(255, 170, 0);
        3 : Vermelho - UNHEALTHY
          // color code #FFAA00 (R = 255,  G = 0, B = 0)
          // setColor(255, 0, 0);
        4 : Verde - GOOD
          // color code #FFAA00 (R = 0,  G = 255, B = 0)
          // setColor(0, 255, 0);
        */

       // String to float 
       float tvocF = 0.0, pm2 = 0.0, pm10 = 0.0;
       tvocF = tvoc.toFloat();
       pm2 = ValuePM2.toFloat();
       pm10 = ValuePM10.toFloat();

    if (hum < 20.0)
    {
        // Acender luz vermelha
        setColor(255, 0, 0);

        // Alterar a variavel LedColour para a cor vermelha
        LedColour = 3;
    }

    if (hum > 85.0)
    {
        // Acender luz vermelha
        setColor(255, 0, 0);

        // Alterar a variavel LedColour para a cor vermelha
        LedColour = 3;
    }

    if (hum >= 20.0 && hum <= 85.0)
    {
        if (temp < 19.0)
        {
            // Acender luz amarela
            setColor(251, 255, 0);

            // Alterar a variavel LedColour para a cor amarela
            LedColour = 1;
        }
        if (temp > 27.0)
        {
            // Acender luz amarela
            setColor(251, 255, 0);

            // Alterar a variavel LedColour para a cor amarela
            LedColour = 1;
        }
        if (temp >= 19.0 && temp <= 27.0)
        {
            // CASO 1 - GOOD
            // Se  0.0 >= PM2.5 <= 12.0 && 0.0 >= PM10 <= 54 && 0.0 >= TVOC <= 0.5 -> GOOD                         (GREEN)
            if ((pm2 >= 0.0 && pm2 <= 12.0) && (pm10 >= 0 && pm10 <= 54) && (tvocF <= 0.5))
            {
                // Acender led com cor verde
                setColor(0, 255, 0);

                // Alterar a variavel LedColour para a cor verde
                LedColour = 4;
            }

            // CASO 2 - MODERATE
            // Se 12.1 >= PM2.5 <= 35.4 && 55 >= PM10 <= 154 && TVOC > 0.5 -> MODERATE                             (YELLOW)
            if ((pm2 >= 12.1 && pm2 <= 35.4) && (pm10 >= 55 && pm10 <= 154) && (tvocF > 0.5))
            {
                // Acender led com cor amarela
                setColor(251, 255, 0);

                // Alterar a variavel LedColour para a cor amarela
                LedColour = 1;
            }

            // CASO 3 - UNHEALTHY FOR SENTIVE GROUP
            // Se 35.5 >= PM2.5 <= 55.4 && 155 >= PM10 <= 254 && TVOC > 0.5 -> UNHEALTHY FOR SENTIVE GROUP         (ORANGE)
            if ((pm2 >= 35.5 && pm2 <= 55.4) && (pm10 >= 155 && pm10 <= 254) && (tvocF > 0.5))
            {
                // Acender led com cor laranja
                setColor(255, 170, 0);

                // Alterar a variavel LedColour para a cor laranja
                LedColour = 2;
            }

            // CASO 4 - UNHEALTHY
            // Se PM2.5 >= 55.5 && PM10 >= 255 && TVOC > 0.5 -> UNHEALTHY                                          (RED)
            if ((pm2 >= 55.5) && (pm10 >= 255) && (tvocF > 0.5))
            {
                // Acender led com cor vermelha
                setColor(255, 0, 0);

                // Alterar a variavel LedColour para a cor vermelha
                LedColour = 3;
            }

            // Acender led com cor amarela
            setColor(251, 255, 0);
            
            // Alterar a variavel LedColour para a cor amarela
            LedColour = 1;
            
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      END OUTPUT USER PROCESS                                                             //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      INIT LORA PROCESS                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Transmit data from mydata
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // digitalWrite(LED_BUILTIN, HIGH); // Turn on LED while sending
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       END LORA PROCESS                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
        0 : Azul - Default
          // color code #0000FF (R = 0,  G = 0, B = 255)
          // setColor(0, 0, 255);
        1 : Amarelo - MODERATE
          // color code #FBFF00 (R = 251,  G = 255, B = 0)
          // setColor(251, 255, 0);
        2 : Laranja - UNHEALTHY FOR SENTIVE GROUP
          // color code #FFAA00 (R = 255,  G = 170, B = 0)
          // setColor(255, 170, 0);
        3 : Vermelho - UNHEALTHY
          // color code #FFAA00 (R = 255,  G = 0, B = 0)
          // setColor(255, 0, 0);
        4 : Verde - GOOD
          // color code #FFAA00 (R = 0,  G = 255, B = 0)
          // setColor(0, 255, 0);
        */

void LedColorToText(int valueColor)
{
  if(valueColor == 0)
  {
    ColorToText = "Default State";
  }

  else if(valueColor == 1)
  {
    ColorToText = "Amarelo - MODERATE";
  }

  else if(valueColor == 2)
  {
    ColorToText = "Laranja - UNHEALTHY FOR SENTIVE GROUP";
  }

  else if(valueColor == 3)
  {
    ColorToText = "Vermelho - UNHEALTHY";
  }

  else if(valueColor == 4)
  {
    ColorToText = "Verde - GOOD";
  }

  else
  {
    ColorToText = "Unkown State";
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       INIT PRETTYPRINT                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////
    //                                        //
    //   Funções para ajudar no PrettyPrint   //
    //                                        //
    ////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////

void ErrorSPSFuncPP(int error_sps)
{

  if (error_sps != 0)
  {
    /*  Local: Setup()
        Motivo: Erro no canal de comunicação
        Mensagem: Could not initialize communication channel.
        Variavel Subjacente: ErrorCodeCommChann                 
    */
    if (ErrorCodeCommChann != 0)
    {
      //
      MessageError1 = "Could not initialize communication channel";
    }
    else
    {
      //
      MessageError1 = "";
    }
    /*  Local: Setup()
        Motivo: Erro na conexão ao SPS30
        Mensagem: Could not probe / connect with SPS30.
        Variavel Subjacente: ErrorCodeConnect             
    */ 
    if (ErrorCodeConnect != 0)
    {
      //
      MessageError2 = "Could not probe / connect with SPS30";
    }
    else
    {
      //
      MessageError2 = "";
    }
    /*  Local: Setup()
        Motivo: Erro ao fazer reset ao SPS30
        Mensagem: Could not reset.
        Variavel Subjacente: ErrorCodeReset       
    */
    if (ErrorCodeReset != 0)
    {
      MessageError3 = "Could not reset";
    }
    else
    {
      //
      MessageError3 = "";
    }
    /*  Local: Setup()
        Motivo: Erro ao fazer a leitura dos dados do SPS30
        Mensagem: Could NOT start measurement.
        Variavel Subjacente: ErrorCodeMeasurement           
    */
    if (ErrorCodeMeasurement != 0)
    {
      MessageError4 = " Could NOT start measurement";
    }
    else
    {
      //
      MessageError4 = "";
    }
    /*
        Local: GetDeviceInfoSps30()
        Motivo: Erro ao tentar obter o Nome do Produto do SPS30
        Mensagem: Could not get product name.
        Variavel Subjacente: ErrorCodeGetProductName
    */
    if (ErrorCodeGetProductName != 0)
    {
      MessageError5 = "Could not get product name";
    }
    else
    {
      //
      MessageError5 = "";
    }
    /*
        Local: GetDeviceInfoSps30()
        Motivo: Erro ao tentar obter o numero série do SPS30
        Mensagem: could not get serial number.
        Variavel Subjacente: ErrorCodeGetSerialNumber
    */
    if (ErrorCodeGetSerialNumber != 0)
    {
      MessageError6 = "Could not get serial number";
    }
    else
    {
      //
      MessageError6 = "";
    }
    /*
        Local: SpsFunc()
        Motivo: Erro durante a leitura dos valores do SPS30
        Mensagem: Error during reading values.
        Variavel Subjacente: ErrorCodeReadValues
    */
    if (ErrorCodeReadValues != 0)
    {
      MessageError7 = "Error during reading values";
    }
    else
    {
      //
      MessageError7 = "";
    }
  }
  else
  {
    // Não há erros
    MessageError = "Não há erros no sensor :)";
    //Clean errors & strings SPS30
    ErrorCodeCommChann = 0,
    ErrorCodeConnect = 0,
    ErrorCodeReset = 0,
    ErrorCodeMeasurement = 0,
    ErrorCodeGetSerialNumber = 0,
    ErrorCodeGetProductName = 0,
    ErrorCodeReadValues= 0;
    MessageError1 = "",
    MessageError2 = "",
    MessageError3 = "",
    MessageError4 = "",
    MessageError5 = "",
    MessageError6 = "",
    MessageError7 = "";
  }
}

void ErrorSGPFuncPP(int error_sgp)
{
  if (error_sgp != 0)
  {
    /*
        Local: SetupSGP()
        Motivo: Erro na conexão ao SGP30
        Mensagem: Sensor SGP30 Not Found.
        Variavel Subjacente: ErrorNotFound
    */
    if (ErrorNotFound != 0)
    {
      MessageErrorSGP1 = "Sensor SGP30 Not Found";
    }
    else
    {
      //
      MessageErrorSGP1 = "";
    }
    /*
        Local: ErrorSGP()
        Motivo: Erro ao fazer a leitura dos dados do SGP30
        Mensagem: Measurement Failed.
        Variavel Subjacente: ErrorMeasurementFailed
    */
    if (ErrorMeasurementFailed != 0)
    {
      MessageErrorSGP2 = "Measurement Failed";
    }
    else
    {
      //
      MessageErrorSGP2 = "";
      // Clean errors & strings SGP30
      ErrorNotFound = 0,
      ErrorMeasurementFailed = 0,
      ErrorFailedRead = 0;
      MessageErrorSGP = "",
      MessageErrorSGP1 = "",
      MessageErrorSGP3 = "";
    }
    /*
        Local: ErrorSGP()
        Motivo: Erro na leitura dos dados do SGP30
        Mensagem: Failed to read from SGP30 sensor.
        Variavel Subjacente: ErrorFailedRead
    */
    if (ErrorFailedRead != 0)
    {
      MessageErrorSGP3 = "Failed to read from SGP30 sensor";
    }
    else
    {
      //
      MessageErrorSGP3 = "";
    }
  }
  else
  {
    // Não há erros
    MessageErrorSGP = "Não há erros no sensor :)";
  }
}

void ErrorDHTFuncPP(int error_dht)
{
  if (error_dht != 0)
  {
    /*
        Local: ErrorDHT()
        Motivo: Erro na leitura 
        Mensagem: Error data read.
        Variavel Subjacente: ErrorDataRead
    */
    if (ErrorFailedRead != 0)
    {
      MessageErrorDHT1 = "Error data read";
    }
    else
    {
      //
      MessageErrorDHT1 = "";
    }
    /*
        Local: ErrorDHT()
        Motivo: Falha na obtençao de dados do sensor DHT22
        Mensagem: Failed to read from DHT sensor! | Possible error, check sensor!
        Variavel Subjacente: ErrorCheck
    */
    if (ErrorFailedRead != 0)
    {
      MessageErrorDHT2 = "Failed to read from DHT sensor | Possible error, check sensor";
    }
    else
    {
      //
      MessageErrorDHT2 = "";
    }
  }
  else
  {
    // Não há erros
    MessageErrorDHT = "Não há erros no sensor :)";
    // Clean errors & strings
    ErrorDataRead = 0,
    ErrorCheck = 0;
    MessageErrorDHT1 = "",
    MessageErrorDHT2 = "";
  }
}
//////////////////////////////////////////////////////////////////////////


void PrettyPrint()
{
  // Init
  Serial.print("\n###############################################################################################################");
  Serial.print("\n################################################# ");
  Serial.print(" INIT LOOP ");
  Serial.print(" #################################################");
  Serial.print("\n###############################################################################################################");

  // Parte do DHT22
  Serial.print("\n\n###############################################################################################################");
  Serial.print("\n################################################## ");
  Serial.print(MY_ID);
  Serial.print(" ####################################################");
  Serial.print("\n###############################################################################################################\n");
  Serial.print("\nSensor DTH22 Values:");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Temperature: ");
  Serial.print(temp); Serial.print(" Celsius");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Humidity: ");
  Serial.print(hum);  Serial.print(" %");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n##############################################################################################################");

  // Parte do SGP30
  Serial.print("\n##############################################################################################################\n");
  Serial.print("\nSensor SGP30 Values:");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> TVOC: ");
  Serial.print(tvoc);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> eCO2: ");
  Serial.print(eco2);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n##############################################################################################################");

  // Parte do SPS30
  Serial.print("\n##############################################################################################################\n");
  Serial.print("\nSensor SPS30 Values:");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> PM 1.0: ");
  Serial.print(ValuePM1);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> PM 2.5: ");
  Serial.print(ValuePM2);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> PM 10.0: ");
  Serial.print(ValuePM10);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n##############################################################################################################");

  // Parte do AirQual
  Serial.print("\n##############################################################################################################\n");
  Serial.print("\nData sended:");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> String Data: ");
  Serial.print(dados);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n##############################################################################################################");

  // Parte do AirQual
  LedColorToText(LedColour);
  Serial.print("\n##############################################################################################################\n");
  Serial.print("\nEstado do LED:");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Modo: ");
  Serial.print(ColorToText);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n##############################################################################################################");

  // Parte dos Erros - Chamar a Função ErrorTestFunc
  ErrorSPSFuncPP(error_sps);
  ErrorSGPFuncPP(error_sgp);
  ErrorDHTFuncPP(error_dht);
    // Imprimir a mensagem do erro
  Serial.print("\n##############################################################################################################\n");
  Serial.print("\n Erros no Sistema :");
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Mensagens de erros no SPS30: ");
  Serial.print("\n * "); Serial.print(MessageError);
  Serial.print("\n * "); Serial.print(MessageError1);
  Serial.print("\n * "); Serial.print(MessageError2);
  Serial.print("\n * "); Serial.print(MessageError3);
  Serial.print("\n * "); Serial.print(MessageError4);
  Serial.print("\n * "); Serial.print(MessageError5);
  Serial.print("\n * "); Serial.print(MessageError6);
  Serial.print("\n * "); Serial.print(MessageError7);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Mensagens de erros no SGP30: ");
  Serial.print("\n * "); Serial.print(MessageErrorSGP);
  Serial.print("\n * "); Serial.print(MessageErrorSGP1);
  Serial.print("\n * "); Serial.print(MessageErrorSGP2);
  Serial.print("\n * "); Serial.print(MessageErrorSGP3);
  Serial.print("\n____________________________________________________________________________________________________________\n");
  Serial.print("\n\t-> Mensagens de erros no DHT22: ");
  Serial.print("\n * "); Serial.print(MessageErrorDHT);
  Serial.print("\n * "); Serial.print(MessageErrorDHT1);
  Serial.print("\n * "); Serial.print(MessageErrorDHT2);
  Serial.print("\n____________________________________________________________________________________________________________\n");

  // End
  Serial.print("\n\n##############################################################################################################");
  Serial.print("\n############################################### ");
  Serial.print(" END LOOP ");
  Serial.print(" ###################################################");
  Serial.print("\n##############################################################################################################\n\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       END PRETTYPRINT                                                                    //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       INIT SETUP                                                                         //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SetupSPS()
{
  sps30.EnableDebugging(DEBUG);

  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);

  // Begin communication channel;
  if (! sps30.begin(SPS30_COMMS))
  {
    Errorloop((char *) "could not initialize communication channel.", 0);
    // Para este erro a variavel toma o valor de 1
    error_sps = 1; ErrorCodeCommChann = 1;
  }
    

  // check for SPS30 connection
  if (! sps30.probe())
  {
    Errorloop((char *) "could not probe / connect with SPS30.", 0);
    // Para este erro a variavel toma o valor de 2
    error_sps = 2; ErrorCodeConnect = 1;
  } 
  else  Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (! sps30.reset())
  {
    Errorloop((char *) "could not reset.", 0);
    // Para este erro a variavel toma o valor de 3
    error_sps = 3; ErrorCodeReset = 1;
  } 

  // read device info
  GetDeviceInfoSps30();

  // start measurement
  if (sps30.start()) Serial.println(F("Measurement started"));
  else
  {
    Errorloop((char *) "Could NOT start measurement", 0);
    // Para este erro a variavel toma o valor de 4
    error_sps = 4; ErrorCodeMeasurement = 1;
  } 

  //serialTrigger((char *) "Hit <enter> to continue reading");

  if (SPS30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }

  // Initialize SPS 30
  Serial.println(F("Trying to connect ..."));
  // set driver debug level
  sps30.EnableDebugging(DEBUG);
  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);
}

void SetupSGP()
{
  // Initialize SGP 30
  if (!sgp.begin())
  {
    Serial.println("Sensor SGP30 not found ...");
    error_sgp = 1; ErrorNotFound = 1;
    return;
  }
  Serial.print("Found SGP30 //Serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

void SetupDHT()
{
  // Initialize dht 
  dht.begin();
}

void SetupLoRa()
{
  // Initialize LoRa Communication
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  #if defined(CFG_eu868) // EU channel setup
   LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  #elif defined(CFG_us915) // US channel setup
   for (int b = 0; b < 8; ++b) {
     LMIC_disableSubBand(b);
   }
   LMIC_enableChannel(17);
  #endif
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Start job -- Transmit a message on begin
  do_send(&sendjob);
}



void setup() 
{
  Serial.begin(9600);

  // Call Setup's
  SetupLoRa();
  SetupSPS();
  SetupSGP();
  SetupDHT();

  // Initialize the digital pin as an output 
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       END SETUP                                                                          //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       INIT LOOP                                                                          //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // LoRa Communication
    os_runloop_once();
    #ifdef SEND_BY_BUTTON
        if (digitalRead(0) == LOW) {
            while (digitalRead(0) == LOW) ;
            do_send(&sendjob);
        }
    #endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Call dht22 function
    dht22();

        // Dealy between function ( 1sec = 1 000ms )
        delay(1000);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Call sgp30 function
    sgpFunc();

        // Dealy between function ( 1sec = 1 000ms )
       delay(1000);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Call sps30 function
    spsFunc();  

        // Dealy between function ( 1sec = 1 000ms )
       delay(1000);

       // Auto Clean Function - Após 10 leituras p.e
       //SetAutoClean();


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Function to show the Air Quality (LED)
    AirQual();

      // Dealy between function ( 1sec = 1 000ms )
       delay(1000);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Function to create/prepare DADOS
    CreateDados();

      // Dealy between function ( 1sec = 1 000ms )
       delay(1000);

    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Chamar o PrettyPrint
PrettyPrint();

// Clear all Datas
    dados.getBytes(mydata, sizeof(mydata)-1);
    // Clean Data
    dados = "";
    // Clean Led atribution color
    LedColour = 0;
    
    // Delay to new Read    ( 5min = 300 000 ms )
    delay(300000);
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       END LOOP                                                                           //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*

void SetAutoClean()
{
  uint32_t interval;
  uint8_t ret;

  // try to get interval
  ret = sps30.GetAutoCleanInt(&interval);
  if (ret == ERR_OK) {
    Serial.print(F("Current Auto Clean interval: "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
    // Meter contador a zero
    cntSPS30 = 0;
  }
  else
    ErrtoMess((char *) "could not get clean interval.", ret);


 Não preciso daqui até ....

  // only if requested
  if (AUTOCLEANINTERVAL == -1) {
    Serial.println(F("No Auto Clean interval change requested."));
    return;
  }

  // try to set interval
  interval = AUTOCLEANINTERVAL;
  ret = sps30.SetAutoCleanInt(interval);
  if (ret == ERR_OK) {
    Serial.print(F("Auto Clean interval now set : "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
  }
  else
      ErrtoMess((char *) "could not set clean interval.", ret);

  // try to get interval
  ret = sps30.GetAutoCleanInt(&interval);
  if (ret == ERR_OK) {
    Serial.print(F("Current Auto Clean interval: "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
  }
  else
    ErrtoMess((char *) "could not get clean interval.", ret);
}

..... aqui

*/