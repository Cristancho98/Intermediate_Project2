#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire ourWire(2);
DallasTemperature sensors(&ourWire);

#define SLAVE_ADDRESS 0X04
int x=0;

void setup(){
    pinMode(13, OUTPUT);
    Serial.begin(9600);
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(rpi);
    sensors.begin();
}
void loop(void){
  sensors.requestTemperatures();
  x=sensors.getTempCByIndex(0);
  Serial.print("envio : ");
  Serial.println(x);
  delay(1000);
  Wire.onRequest(rpi);
}
void receiveData(int byteCount){
    
    while(Wire.available()) {
         byteCount = Wire.read();
         Serial.println(byteCount);
         if( byteCount==0 ){
         digitalWrite(13,HIGH);
         }
         if( byteCount==1 ){
           digitalWrite(13,LOW);
         }        
     }
}
void rpi(){
  Wire.write(x); 
}
