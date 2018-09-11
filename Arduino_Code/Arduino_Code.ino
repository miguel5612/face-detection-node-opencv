#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_CCS811.h>
#include <TinyGPS.h>
#include "SparkFunCCS811.h"
#include "DHT.h"

#define DHTPIN_INT 7
#define DHTPIN_EXT 8
#define DHTTYPE DHT11

#define timeDelay 50
#define baudRateGPS 9600
#define gpsTxPin  8
#define gpsRxPin  9
#define RST_PIN  9    //Pin 9 para el reset del RC522
#define SS_PIN  10   //Pin 10 para el SS (SDA) del RC522
#define ledStatus 13
#define CCS811_ADDR 0x5A //Alternate I2C Address

static void smartdelay(unsigned long ms);
unsigned long age, date, time, chars = 0;
unsigned short sentences = 0, failed = 0;
float flat, flon,alt,h1,h2,t1,t2;
boolean statusFlag, ledFlag;

TinyGPS gps;
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);
MFRC522 mfrc522(SS_PIN, RST_PIN); //Creamos el objeto para el RC522
DHT dht_int(DHTPIN_INT, DHTTYPE);
DHT dht_ext(DHTPIN_EXT, DHTTYPE);

CCS811 myCCS811(CCS811_ADDR);

void setup() {
  Serial.begin(9600); //Iniciamos la comunicación  serial
  gpsSerial.begin(baudRateGPS);
  SPI.begin();        //Iniciamos el Bus SPI
  mfrc522.PCD_Init(); // Iniciamos  el MFRC522
  pinMode(ledStatus,OUTPUT);
  //This begins the CCS811 sensor and prints error status of .begin()
  CCS811Core::status returnCode = myCCS811.begin();
  Serial.print("begin exited with: ");
  printDriverError( returnCode );
  Serial.println();
  dht_int.begin();
  dht_ext.begin();
}

void loop() {
  // Revisamos si hay nuevas tarjetas  presentes
  if ( mfrc522.PICC_IsNewCardPresent()) 
        {  
      //Seleccionamos una tarjeta
            if ( mfrc522.PICC_ReadCardSerial()) 
            {
                  // Enviamos serialemente su UID
                  Serial.print("Card UID:");
                  for (byte i = 0; i < mfrc522.uid.size; i++) {
                          Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
                          Serial.print(mfrc522.uid.uidByte[i], HEX);   
                  }
                  if (myCCS811.dataAvailable())
                  {
                    myCCS811.readAlgorithmResults();
                    
                    float h1 = dht_int.readHumidity();
                    float t1 = dht_int.readTemperature();
                    float h2 = dht_ext.readHumidity();
                    float t2 = dht_ext.readTemperature();
                    readGPS();
                    Serial.print("CO2[");
                    Serial.print(myCCS811.getCO2());
                    Serial.print("] tVOC[");
                    Serial.print(myCCS811.getTVOC());
                    Serial.print("] millis[");
                    Serial.print(millis());
                    Serial.print("] ");
                    printRunTime();
                    Serial.print(" ");
                    Serial.print(" T = ");
                    Serial.print(t1);
                    Serial.print(" ");
                    Serial.print("T2 = ");
                    Serial.print(t2);
                    Serial.print(" ");
                    Serial.print("H1= ");
                    Serial.print(h1);
                    Serial.print(" ");
                    Serial.print("H2 = ");
                    Serial.print(h2);
                    Serial.print(" ");
                    
                    Serial.print("LAT = ");
                    Serial.print(flat);
                    Serial.print(" ");
                    
                    Serial.print("LON = ");
                    Serial.print(flon);
                    Serial.print(" ");
                    Serial.println();
                  } 
                  Serial.println();
                  // Terminamos la lectura de la tarjeta  actual
                  mfrc522.PICC_HaltA();         
            }      
  } 
}
void readGPS(){
  sentences = 0, failed = 0;chars = 0;
  gps.f_get_position(&flat, &flon, &age);
  gps.stats(&chars, &sentences, &failed);
  if ((flat == 1000 && flon == 1000)  ) {
    //showError("GPS: flat = 1000, flon = 1000");
  }
}
void showError(String sensor){
  ledFlag = !ledFlag; //Cambio de estado (intermitente)
    digitalWrite(ledStatus,!ledFlag);
    delay(timeDelay);
    Serial.print("Fallo el sensor: ");
    Serial.println(sensor);
}

 static void smartdelay(unsigned long ms)
  {
    unsigned long start = millis();
    do
    {
      while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
      }
    } while (millis() - start < ms);


 }
void printRunTime()
{
  char buffer[50];

  unsigned long runTime = millis();

  int hours = runTime / (60 * 60 * 1000L);
  runTime %= (60 * 60 * 1000L);
  int minutes = runTime / (60 * 1000L);
  runTime %= (60 * 1000L);
  int seconds = runTime / 1000L;

  sprintf(buffer, "RunTime[%02d:%02d:%02d]", hours, minutes, seconds);
  Serial.print(buffer);

  if (hours == 0 && minutes < 20) Serial.print(" Not yet valid");
}

 void printDriverError( CCS811Core::status errorCode )
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      Serial.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      Serial.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      Serial.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      Serial.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      Serial.print("GENERIC_ERROR");
      break;
    default:
      Serial.print("Unspecified error.");
  }
}

 void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
  }
  else
  {
    Serial.print("Error: ");
    if (error & 1 << 5) Serial.print("HeaterSupply");
    if (error & 1 << 4) Serial.print("HeaterFault");
    if (error & 1 << 3) Serial.print("MaxResistance");
    if (error & 1 << 2) Serial.print("MeasModeInvalid");
    if (error & 1 << 1) Serial.print("ReadRegInvalid");
    if (error & 1 << 0) Serial.print("MsgInvalid");
    Serial.println();
  }
}
