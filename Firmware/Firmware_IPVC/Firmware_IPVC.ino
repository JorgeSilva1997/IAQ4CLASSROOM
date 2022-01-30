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
#define DEBUG 2
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();


/*  DHT22 CONFS */
#define DHTPIN 27            // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22       // DHT 22
DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT sensor.



/*  LED RGB CONFS   */
#define LEDR 25   
#define LEDG 26    
#define LEDB 27  
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




/*  LoRaWAN */

    #define SEND_BY_TIMER 2 // Send a message every TX_INTERVAL seconds
    
    static const PROGMEM u1_t NWKSKEY[16] = { 0xd9, 0x75, 0x24, 0xc0, 0xf2, 0x09, 0x52, 0x65, 0xa4, 0x85, 0xa0, 0x8c, 0x83, 0x8d, 0x11, 0x10 };
    static const u1_t PROGMEM APPSKEY[16] = { 0xf6, 0x5a, 0xdf, 0x86, 0x63, 0x77, 0xc8, 0x0c, 0x67, 0x56, 0x76, 0xc0, 0x85, 0x29, 0x20, 0xd9 };
    static const u4_t DEVADDR = 0x0047a2ce;

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
    //#ifdef SEND_BY_TIMER
    //#define TX_INTERVAL 60 // Message send interval in seconds
    //#endif

/*

Tipo de erros:
    -> Má leitura OU Leitura sem sucesso
    -> Não detetar sensor

*/





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
            //dht22();
        }
    }
    else
    {
        // Sensor já excedeu as tentivas de erro | Avisar USER (PISCAR LED) | Enviar mensagem de erro para Dash 
            // Printar na console 
            Serial.println(F("Failed to read from DHT sensor!"));
            // Init LED 
                // !!! FALTA !!!
            
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
    
    // Errors 
    // dados += ("&E1="); dados += (errordht);   // Erro do dht22       ???
}

void setup() 
{
  Serial.begin(9600);

  // Initialize the digital pin as an output 
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  // Initialize LED LOW
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
  
  // Initialize dht 
  dht.begin();
//  if (!dht.begin())
//  {
//    Serial.println("Sensor DHT22 not found ...");
    //while (1);
//    return;
//  }

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

    // Call dht22 function
    dht22();

        // Dealy between function ( 1sec = 1 000ms )
        delay(1000);

    // Call sgp30 function
//    sgpFunc();

        // Dealy between function ( 1sec = 1 000ms )
//        delay(1000);

    // Function to create/prepare DADOS
//    CreateDados();

//    dados.getBytes(mydata, sizeof(mydata)-1);
    // Clear Data
    dados = "";

    // Delay to new Read    ( 5min = 30 000ms )
//    delay(30000);
}


/*

    O QUE FALTA:
        * Implementar função para o SPS 30;
        * Comunicação LoRa;
        * LED Output;
        * Lógica dos erros;

*/
