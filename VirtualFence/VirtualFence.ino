#include "config.h"
#include "BluetoothSerial.h"

TTGOClass *watch;
TFT_eSPI *tft;
BMA *sensor;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

float raio = 1;
double lat0 = -23.6687860310263;
double lon0 = -46.54485601780751;
double dist = 0;

String dentro = "Fora da Area";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  
  // Get TTGOClass instance
  watch = TTGOClass::getWatch();

  // Initialize the hardware, the BMA423 sensor has been initialized internally
  watch->begin();

  // Turn on the backlight
  watch->openBL();

  //Receive objects for easy writing
  tft = watch->tft;
  sensor = watch->bma;

  
  // Some display settings
  tft->setTextColor(random(0xFFFF));
  tft->drawString("Virtual Fence",  25, 50, 4);
  tft->setTextFont(4);
  tft->setTextColor(TFT_WHITE, TFT_BLACK);
  
}

void loop() {
  /*
  // put your main code here, to run repeatedly:
  
  */

  if(SerialBT.available()){
    String coor = SerialBT.readString();
    String lat = coor.substring(0, coor.indexOf('/'));
    String lon = coor.substring(coor.indexOf('/') + 1);
    Serial.write(SerialBT.read());
    tft->fillRect(98, 100, 70, 85, TFT_BLACK);
    tft->setCursor(80, 100);
    tft->drawString("Latitude:"+lat, 25, 100, 4);
    tft->drawString("Longitude:"+lon, 25, 125, 4);

    Serial.println(lat0);  
    Serial.println(lat);
    //Serial.println(lon.toDouble());
    //Serial.println(pow((lat.toDouble() - (lat0)),2)+pow((lon.toDouble()-(lon0)),2));
    
    dist = sqrt(pow((lat.toDouble() - (lat0)),2)+pow((lon.toDouble()-(lon0)),2));

    //Serial.println(dist);

    if (dist>raio){
      dentro = "Fora da Area";
      tft->drawString(dentro, 25, 150, 4);
    }else{
      dentro = "Dentro da Area";
      tft->drawString(dentro, 25, 150, 4);
    }

    
    uint8_t buf[dentro.length()];
    memcpy(buf,dentro.c_str(),dentro.length());
    SerialBT.write(buf,dentro.length());
  }

  delay(5000);
}
