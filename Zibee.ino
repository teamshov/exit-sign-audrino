#include <HardwareSerial.h>

HardwareSerial mySerial(2);
#define RXD2 16
#define TXD2 17

void setup()
{
  // Note the format for setting a serial port is as follows:
  //Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{

//Recieving the messages  
while(mySerial.available())Serial.println(char(mySerial.read()));
}
