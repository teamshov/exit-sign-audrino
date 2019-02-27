#include <WiFi.h>
#include <PubSubClient.h>

//const char* ssid = "Abdulla Al Noman";
//const char* password = "alnoman8877";

const char* ssid = "pollution"; // username for the router
const char* password = "aus12345"; // password for the router

const char* mqttServer = "m16.cloudmqtt.com"; // host address that we will connect to
const int mqttPort = 18357;
const char* mqttUser = "tgfiquve";
const char* mqttPassword = "X4GCp1RDg7zh";

int red = 5;
int green = 25;
int blue = 14;
  

WiFiClient espClient;
PubSubClient client(espClient);
 
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
Serial.println("-----------------------");
Serial.println("----------" + s);

if(s == "red"){
  digitalWrite(red, LOW); // turn the LED on 
  digitalWrite(blue, HIGH); // turn the LED on 
  digitalWrite(green, HIGH); // turn the LED on 
}

if(s == "yellow"){
  digitalWrite(blue, HIGH); // turn the LED on 
  digitalWrite(red, LOW); // turn the LED off 
  digitalWrite(green, LOW); // turn the LED off 
}

if(s == "blue"){
  digitalWrite(blue, LOW); // turn the LED on 
  digitalWrite(red, HIGH); // turn the LED on 
  digitalWrite(green, HIGH); // turn the LED on 
}

if(s == "green"){
  digitalWrite(green, LOW); // turn the LED on 
  digitalWrite(blue, HIGH); // turn the LED on 
  digitalWrite(red, HIGH); // turn the LED on 
}

}
 
void setup()
{
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
}
Serial.println("Connected to the WiFi network");
 
client.setServer(mqttServer, mqttPort);
client.setCallback(callback);
 
while (!client.connected()) {
Serial.println("Connecting to MQTT...");
 
if (client.connect("ESP32Client", mqttUser, mqttPassword ))
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
client.subscribe("esp32/esp32test");
 
}
 
void loop()
{
   char slpg[10];
   float sensor_volt;
    float sensorValue;

    sensorValue = analogRead(38);
    sensor_volt = sensorValue/1024*5.0;

    Serial.print("sensor_volt = ");
    Serial.print(sensor_volt);
    
    Serial.println("V");
    itoa(sensorValue,slpg,10); // converting value to string
    
    client.publish("esp32/esp32test1",slpg);
    Serial.println(sensorValue);
    delay(1000);

client.loop();
}
