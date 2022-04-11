//
//  Firmware
//

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
//#include <DHT_U.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "sps30.h"



// create constructor
Adafruit_SGP30 sgp;
// create constructor
SPS30 sps30;

/*  DEFINE ID ESP32 */
#define MY_ID   "DIAQ002"

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


/*  DHT22 CONFS */
#define DHTPIN 27            // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22       // DHT 22
DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT sensor.



/*  LED RGB CONFS   */
#define LEDR 25             // PINOUT NO PROJ ->    4
#define LEDG 26             // PINOUT NO PROJ ->    5
#define LEDB 27             // PINOUT NO PROJ ->    18
int ledcolor = 0;
int a = 1000; //this sets how long the stays one color for


/*  Variaveis Globais   */

    // Data send
    String dados;

    // For DHT22
    float temp=0, hum=0;
    int cntdht=0;

    // For SGP 30
    String tvoc, eco2;
    int cntsgp=0;

    // Errors Part | If value = 1 -> Error in Sensor
    int error_dht = 0,
        error_sgp = 0, 
        error_sps = 0;




/*  LoRaWAN */

    #define SEND_BY_TIMER 2 // Send a message every TX_INTERVAL seconds
    
    static const PROGMEM u1_t NWKSKEY[16] = { 0xd9, 0x75, 0x24, 0xc0, 0xf2, 0x09, 0x52, 0x65, 0xa4, 0x85, 0xa0, 0x8c, 0x83, 0x8d, 0x11, 0x10 };
    static const u1_t PROGMEM APPSKEY[16] = { 0xf6, 0x5a, 0xdf, 0x86, 0x63, 0x77, 0xc8, 0x0c, 0x67, 0x56, 0x76, 0xc0, 0x85, 0x29, 0x20, 0xd9 };
    static const u4_t DEVADDR = 0x0047a2ce;

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
            //dados += ("&ErDht="); dados += (error_dht); 
            error_dht = 1;
            
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

    // PRINT IN CONSOLE RESULT
    Serial.print("Humidity: "); Serial.print(hum); Serial.print(" %, Temp: "); Serial.print(temp); Serial.println(" Celsius");

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
            //Serial.println("Measurement failed");
            sgpFunc();
            //return;
        }
        else
        {
            // Se ler corretamente os valores mete o contador a ZERO 
            cntsgp=0;
            //dht22();
        }
    }
    else
    {
        // Sensor já excedeu as tentivas de erro | Avisar USER (PISCAR LED) | Enviar mensagem de erro para Dash 
            // Printar na console 
            Serial.println(F("Failed to read from SGP30 sensor!"));
            // Init LED 
                // !!! FALTA !!!
            
        // Voltar a meter a variavel do contador a ZERO
        cntsgp = 0;
    }
}

/*  SGP30 FUNCTION  */
void sgpFunc()
{
    // if (! sgp.IAQmeasure()) 
    // {
    //     Serial.println("Measurement failed");
    //     return;
    // }
    // Check Erros
    ErrorSGP();
    // Copia para as variaveis globais o valor lido
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;


//      NÃO SEI SE É PRECISO ???
//    delay(1000);
//   counter++;
//   if (counter == 30) {
//     counter = 0;

//     uint16_t TVOC_base, eCO2_base;
//     if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
//       Serial.println("Failed to get baseline readings");
//       return;
//     }
//     Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
//     Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
//   }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                      END SGP30 PROCESS                                                                   //
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
    
    /*
        Errors Part
    */
   
      // DHT22 Part
    if(error_dht != 0)
    {
      dados += ("&ErDht="); dados += (error_dht);   // Erro do dht22 
    }
      // SPS Part
      // SGP Part
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
            Se  0.0 >= PM2.5 =< 12.0 && 0.0 >= PM10 =< 54 && 0.0 >= TVOC =< 0.5 -> GOOD                         (GREEN)
            Se 12.1 >= PM2.5 =< 35.4 && 55 >= PM10 =< 154 && TVOC > 0.5 -> MODERATE                             (YELLOW)
            Se 35.5 >= PM2.5 =< 55.4 && 155 >= PM10 =< 254 && TVOC > 0.5 -> UNHEALTHY FOR SENTIVE GROUP         (ORANGE)
            Se PM2.5 >= 55.5 && PM10 >= 255 && TVOC > 0.5 -> UNHEALTHY                                          (RED)
                ATENÇÃO !!! FALTA A PARTE DAS PARTÍCULAS E DO CO2 (https://www.ebay.com/itm/112691626960?mkevt=1&siteid=1&mkcid=2&mkrid=711-153320-877651-5&source_name=google&mktype=pla_ssc&campaignid=11826484955&groupid=114180021839&targeted=pla-293946777986&MT_ID=&adpos=&device=c&googleloc=1011776&itemid=112691626960&merchantid=116792603&geo_id=164&gclid=EAIaIQobChMIte_IhKba9QIViQUGAB0nfQFtEAYYAyABEgIPf_D_BwE)
        */

       // String to float 
       float tvocF = 0.0;
       tvocF = tvoc.toFloat();

    if (hum < 20.0)
    {
        // Acender luz vermelha
    }

    if (hum > 85.0)
    {
        // Acender luz vermelha
    }

    if (hum >= 20.0 && hum <= 85.0)
    {
        if (temp < 19.0)
        {
            // Acender luz amarela
        }
        if (temp > 27.0)
        {
            // Acender luz amarela
        }
        if (temp >= 19.0 && temp <= 27.0)
        {
/*
    val.MassPM2 -> Meter isto numa variavel global quando for lida (agora vou chamar-lhe de pm25)
    val.MassPM10 -> Meter isto numa variavel global quando for lida (agora vou chamar-lhe de pm10)
*/
            // CASO 1 - GOOD
            // Se  0.0 >= PM2.5 =< 12.0 && 0.0 >= PM10 =< 54 && 0.0 >= TVOC =< 0.5 -> GOOD                         (GREEN)
            if ((pm25 >= 0.0 && pm25 =< 12.0) && (pm10 >= 0 && pm10 =< 54) && (tvocF =< 0.5))
            {
                // Acender led com cor verde
            }

            // CASO 2 - MODERATE
            // Se 12.1 >= PM2.5 =< 35.4 && 55 >= PM10 =< 154 && TVOC > 0.5 -> MODERATE                             (YELLOW)
            if ((pm25 >= 12.1 && pm25 =< 35.4) && (pm10 >= 55 && pm10 =< 154) && (tvocF > 0.5))
            {
                // Acender led com cor amarela
            }

            // CASO 3 - UNHEALTHY FOR SENTIVE GROUP
            // Se 35.5 >= PM2.5 =< 55.4 && 155 >= PM10 =< 254 && TVOC > 0.5 -> UNHEALTHY FOR SENTIVE GROUP         (ORANGE)
            if ((pm25 >= 35.5 && pm25 =< 55.4) && (pm10 >= 155 && pm10 =< 254) && (tvocF > 0.5))
            {
                // Acender led com cor laranja
            }

            // CASO 4 - UNHEALTHY
            // Se PM2.5 >= 55.5 && PM10 >= 255 && TVOC > 0.5 -> UNHEALTHY                                          (RED)
            if ((pm25 >= 55.5) && (pm10 >= 255) && (tvocF > 0.5))
            {
                // Acender led com cor vermelha
            }
            
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                          //
//                                                       INIT SPS30 PROCESS                                                                   //
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
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("Product name  : "));

    if(strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

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
          return(false);
        }
        delay(1000);
    }

    // if other error
    else if(ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
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
  dados += ("&a="); dados+= (val.MassPM1);
  Serial.print(F("\t"));
  Serial.print(val.MassPM2);
  dados += ("&b="); dados+= (val.MassPM2);
  Serial.print(F("\t"));
  Serial.print(val.MassPM4);
  Serial.print(F("\t"));
  Serial.print(val.MassPM10);
  dados += ("&c="); dados+= (val.MassPM10);
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
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
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
//                                                       END SPS30 PROCESS                                                                   //
//                                                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() 
{
  Serial.begin(9600);

  /*

      SPS30 SETUP

  */
  sps30.EnableDebugging(DEBUG);

  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);

  // Begin communication channel;
  if (! sps30.begin(SPS30_COMMS))
    Errorloop((char *) "could not initialize communication channel.", 0);

  // check for SPS30 connection
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
  else  Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0);

  // read device info
  GetDeviceInfoSps30();

  // start measurement
  if (sps30.start()) Serial.println(F("Measurement started"));
  else Errorloop((char *) "Could NOT start measurement", 0);

  //serialTrigger((char *) "Hit <enter> to continue reading");

  if (SPS30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }

  // Initialize the digital pin as an output 
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  // Initialize LED LOW
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);

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
  
  // Initialize dht 
  dht.begin();

  // Initialize SGP 30
  if (!sgp.begin())
  {
    Serial.println("Sensor SGP30 not found ...");
    //while (1);
    return;
  }
  Serial.print("Found SGP30 //Serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // Initialize SPS 30
  Serial.println(F("Trying to connect ..."));
  // set driver debug level
  sps30.EnableDebugging(DEBUG);
  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);
}

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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Function to create/prepare DADOS
    CreateDados();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Function to show the Air Quality (LED)
    AirQual();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    dados.getBytes(mydata, sizeof(mydata)-1);
    // Clear Data
    dados = "";

    // Delay to new Read    ( 5min = 30 000ms )
    delay(30000);
}


/*
    O QUE FALTA:
        * Comunicação LoRa;                         (Feito mas falta testar!)
        * LED Output;
        * Lógica dos erros;
*/
