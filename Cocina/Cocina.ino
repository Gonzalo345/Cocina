/*
   Instancia MONITOREO cloudmqtt
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include "dht.h"// DHT11 temperature and humidity sensor Predefined library
#include "DHTesp.h"

#define DHTTYPE DHT11   // DHT 11
#define dht_dpin 0      //GPIO-0 D3 pin of nodemcu

#define led_1 D0
#define led_2 D1 //D4 led de la placa
#define pinRele D8
#define pulsador D2


//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;

const char *ssid = "Gon5A"; //"MANTENIMIENTO_NC";
const char *password = "gonzalo3";//"wdzg-veid-ed4p";

/*
  const char *ssid = "Red-INTI";
  const char *password = "reDES1957";
*/
char   SERVER[50]   = "tailor.cloudmqtt.com";
int    SERVERPORT   = 12034;
String USERNAME = "Gonzalo";
char   PASSWORD[50] = "12345";

unsigned long previousMillis = 0;

char charPulsador [15];
String strPulsador;
String strPulsadorUltimo;

char PLACA[50];

char valueStr[15];
String strtemp = "";
String strext = "";
char TEMPERATURA[50];
char HUMEDAD[50];
char PULSADOR[50];
char LED_1[50];
char LED_2[50];
//char LED_3[50];
char RELE[50];
char EXTRACTOR[50];

int led2state;
float temp;

//-------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);


// Temperature sensonr

DHTesp dht;
//------------------------CALLBACK-----------------------------
void callback(char* topic, byte* payload, unsigned int length) {

  char PAYLOAD[5] = "    ";

  Serial.print("Mensaje recibido: [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    PAYLOAD[i] = (char)payload[i];
  }
  Serial.println(PAYLOAD);

  if (String(topic) ==  String(LED_1)) {
    if (payload[0] == '1') {
      digitalWrite(led_1, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(led_1, LOW);
    }
  }

   if (String(topic) ==  String(LED_2)) {
    if (payload[0] == '1') {
      digitalWrite(led_2, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(led_2, LOW);
    }
  }

  if (String(topic) ==  String(RELE)) {
    if (payload[0] == '1') {
      digitalWrite(pinRele, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(pinRele, LOW);
    }
  }


}


//------------------------RECONNECT-----------------------------
void reconnect() {
  uint8_t retries = 3;
  // Loop hasta que estamos conectados
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT...");
    // Crea un ID de cliente al azar
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    USERNAME.toCharArray(PLACA, 50);
    if (client.connect("", PLACA, PASSWORD))
    {
      Serial.println("conectado");
      client.subscribe(LED_1);
      client.subscribe(LED_2);
      client.subscribe(RELE);
      client.subscribe(EXTRACTOR);

    }
    else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intenta nuevamente en 5 segundos");
      // espera 5 segundos antes de reintentar
      delay(5000);
    }
    retries--;
    if (retries == 0) {
      // esperar a que el WDT lo reinicie
      while (1);
    }
  }
}

//------------------------SETUP-----------------------------
void setup() {


  delay(10);


  dht.setup(0, DHTesp::DHT11); // Connect DHT sensor to GPIO 17

  pinMode(pulsador, INPUT);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(pinRele, OUTPUT);

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, LOW);
  digitalWrite(pinRele, LOW);


  // Inicia Serial
  Serial.begin(115200);
  Serial.println("");

  // Conexión WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED and contconexion < 50) { //Cuenta hasta 50 si no se puede conectar lo cancela
    ++contconexion;
    delay(500);
    Serial.print(".");
  }
  if (contconexion < 50) {
    Serial.println("WiFi conectado");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("");
    Serial.println("Error de conexion");
  }

  client.setServer(SERVER, SERVERPORT);
  client.setCallback(callback);

  String temperatura = "/" + USERNAME + "/" + "temperatura";
  temperatura.toCharArray(TEMPERATURA, 50);

  String humedad = "/" + USERNAME + "/" + "humedad";
  humedad.toCharArray(HUMEDAD, 50);

  String pulsador = "/" + USERNAME + "/" + "pulsador";
  pulsador.toCharArray(PULSADOR, 50);

  String led_1 = "/" + USERNAME + "/" + "led_1";
  led_1.toCharArray(LED_1, 50);

  String led_2 = "/" + USERNAME + "/" + "led_2";
  led_2.toCharArray(LED_2, 50);

  String rele = "/" + USERNAME + "/" + "rele";
  rele.toCharArray(RELE, 50);

  String extractor = "/" + USERNAME + "/" + "extractor";
  extractor.toCharArray(EXTRACTOR, 50);


}

//--------------------------LOOP--------------------------------
void loop() {


  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis >= 20000) { //envia la temperatura cada 10 segundos
    delay(dht.getMinimumSamplingPeriod());

    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();

  /*  Serial.print(dht.getStatusString());
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.print("\t\t");*/
    previousMillis = currentMillis;
    strtemp = String(temperature, 1); //1 decimal
    strtemp.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(TEMPERATURA) + "] " + strtemp);
    client.publish(TEMPERATURA, valueStr);
    strtemp = String(humidity, 1); //1 decimal
    strtemp.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(HUMEDAD) + "    ] " + strtemp);
    client.publish(HUMEDAD, valueStr);
/*
    if (temp >= 30)
    {
      digitalWrite(led_2, HIGH);
      led2state = digitalRead(led_2);
      strext = String(led2state, 1); //1 decimal
      strext.toCharArray(valueStr, 15);
      client.publish(EXTRACTOR, valueStr);
    }
    else
    {
      digitalWrite(led_2, LOW);
      led2state = digitalRead(led_2);
      strext = String(led2state, 1); //1 decimal
      strext.toCharArray(valueStr, 15);
      client.publish(EXTRACTOR, valueStr);
    }
    */
  }



  if (digitalRead(pulsador) == 0) {
    strPulsador = "SI";
  } else {
    strPulsador = "NO";
  }

  if (strPulsador != strPulsadorUltimo) { //envia el estado del pulsador solamente cuando cambia.
    strPulsadorUltimo = strPulsador;
    strPulsador.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(PULSADOR) + "] " + strPulsador);
    client.publish(PULSADOR, valueStr);
  }

}