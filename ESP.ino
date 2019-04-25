#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
WiFiClient espClient;
PubSubClient client(espClient);

uint64_t chipidnum = ESP.getEfuseMac();
unsigned long c1 = (unsigned long)((chipidnum & 0xFFFF0000) >> 16);
unsigned long c2 = (unsigned long)((chipidnum & 0xFFFF0000));

//Getting the mac address
String chipid = String(c1, HEX) + String(c2, HEX);

HardwareSerial mySerial(2);
#define RXD2 16
#define TXD2 17

//const char* ssid = "pollution"; // username for the router
//const char* password = "aus12345"; // password for the router

const char* ssid = "~7sn Al Noman~"; // username for the router
const char* password = "hassan10"; // password for the router


String colortopics = String("device/esp32/" + chipid + "/color");
const char * colortopic = colortopics.c_str();

// host address that we will connect to
const char* mqttServer = "omaraa.ddns.net";
const int mqttPort = 1883;

//Initializing the RGB Ports
int red = 5;
int green = 25;
int blue = 14;

int wificounter = 0;
bool flag = 0;



//MQTT callback
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String s = "";
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    s += (char)payload[i];
    //Serial.print(payload[i]);
  }
  Serial.println();
  //Serial.println("-----------------------");
  Serial.println("----------" + s);

  if (s == "red") {
    digitalWrite(red, LOW); // turn the LED on
    digitalWrite(blue, HIGH); // turn the LED on
    digitalWrite(green, HIGH); // turn the LED on
  }

  if (s == "yellow") {
    digitalWrite(blue, HIGH); // turn the LED on
    digitalWrite(red, LOW); // turn the LED off
    digitalWrite(green, LOW); // turn the LED off
  }

  if (s == "blue") {
    digitalWrite(blue, LOW); // turn the LED on
    digitalWrite(red, HIGH); // turn the LED on
    digitalWrite(green, HIGH); // turn the LED on
  }

  if (s == "green") {
    digitalWrite(green, LOW); // turn the LED on
    digitalWrite(blue, HIGH); // turn the LED on
    digitalWrite(red, HIGH); // turn the LED on
  }

}

void setup()
{

  //Initilizing the xbee TX and RX
  //Serial.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);


  // initialize the digital pin as an output.
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);

  //Turning the LEDs off
  digitalWrite(red, HIGH);
  digitalWrite(green, HIGH);
  digitalWrite(blue, HIGH);

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");

    wificounter++;

    if (wificounter > 5)
    {
      flag = 1;
      Serial.println("Wifi is down");
      break;
    }


  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected() && !flag) {
    Serial.println("Connecting to MQTT...");

    //if (client.connect("ESP32Client", mqttUser, mqttPassword ))
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected to MQTT");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  Serial.println(colortopic);
  client.subscribe(colortopic);

}

void loop()
{
  char slpg[8];
  float sensor_volt;
  float sensorValue;
  float danger_val;
  sensorValue = analogRead(38);
  sensor_volt = sensorValue / 1024 * 5.0;

  if (sensorValue < 900)
    danger_val = 0;
  else
    danger_val = (sensorValue - 900) / 3195;

  //Serial.print("sensor_volt = ");
  //Serial.print(sensor_volt);
  //Serial.println("V");

  dtostrf(danger_val, 4, 4, slpg);
  //  itoa(danger_val, slpg, 10); // converting value to string
  const char * topic = String("device/esp32/" + chipid + "/mq2").c_str();
  client.publish(topic, slpg);
  Serial.println(danger_val);
  delay(1000);

  if (flag == 1)
  {
    String SS;
    //Recieving the messages
    while (mySerial.available()) {
      //Serial.print(char(mySerial.read()));
      SS += char(mySerial.read());
      
    }
    Serial.println(SS);
      if (SS == "red") {
        digitalWrite(red, LOW); // turn the LED on
        digitalWrite(blue, HIGH); // turn the LED on
        digitalWrite(green, HIGH); // turn the LED on
      }

      if (SS == "green") {
        digitalWrite(green, LOW); // turn the LED on
        digitalWrite(blue, HIGH); // turn the LED on
        digitalWrite(red, HIGH); // turn the LED on
      }
      for(int i = 0; i < 8; i++) {
        mySerial.write(slpg[i]);
      }
      

    

    /*
    while (Serial.available()) {
      //mySerial.write(char(Serial.read()));
      mySerial.write(slpg);
    }
    */
  }
  client.loop();
}
