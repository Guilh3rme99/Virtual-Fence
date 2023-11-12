#include "config.h"
#include "BluetoothSerial.h"
#include <limits.h>


#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)
#define INSIDE 0
#define OUTSIDE 1


TTGOClass *watch;
TFT_eSPI *tft;
BMA *sensor;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

 
struct Point {
    double x, y;
};
 
BluetoothSerial SerialBT;

float raio = 1;
double lat0 = -23.6687860310263;
double lon0 = -46.54485601780751;
double dist = 0;
int lados = 4;
String dentro = "Fora da Area";


Point polygon[] = { { -23.666663523968463, -46.543718761178326 }, { -23.665975667081618, -46.5439655244077 }, { -23.665474511927933, -46.542109435769405 }, { -23.666476820314173, -46.54173392650733 } };



bool verificaCirulo(String la, String lo){
  dist = sqrt(pow((la.toDouble() - (lat0)),2)+pow((lo.toDouble()-(lon0)),2));

  //Serial.println(dist);
  if (dist>raio){
    return false;
  }else{
    return true;
  }  
}
int InsidePolygon(Point *polygon,int N,Point p)
{
  int counter = 0;
  int i;
  double xinters;
  Point p1,p2;

  p1 = polygon[0];
  for (i=1;i<=N;i++) {
    p2 = polygon[i % N];
    if (p.y > MIN(p1.y,p2.y)) {
      if (p.y <= MAX(p1.y,p2.y)) {
        if (p.x <= MAX(p1.x,p2.x)) {
          if (p1.y != p2.y) {
            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
            if (p1.x == p2.x || p.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return(OUTSIDE);
  else
    return(INSIDE);
}

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

    Point p = {lat.toDouble(), lon.toDouble()};
    
    //Serial.println(lon.toDouble());
    //Serial.println(pow((lat.toDouble() - (lat0)),2)+pow((lon.toDouble()-(lon0)),2));
    
    //dist = sqrt(pow((lat.toDouble() - (lat0)),2)+pow((lon.toDouble()-(lon0)),2));

    //Serial.println(dist);
    //bool d = verificaCirculo(lat,lon);    
    int d = InsidePolygon(polygon, lados, p);

    if (d == 1){
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
