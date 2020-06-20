#ifndef Cocina_h
#define Cocina_h


#define luzAzul     D0
#define luzVerde    D1
#define luzRoja     D2
#define dht_dpin    D3      // GPIO-0 D3 pin of nodemcu
#define oneWireBus  D5      // GPIO where the DS18B20 is connected to
#define caldera     D6
#define movimiento  D7




//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;

const char *ssid = "Gon5A"; //"MANTENIMIENTO_NC";
const char *password = "gonzalo3";//"wdzg-veid-ed4p";

char   SERVER[50]   = "tailor.cloudmqtt.com";
int    SERVERPORT   = 12034;
String USERNAME = "Gonzalo";
char   PASSWORD[50] = "12345";

#endif  /* PANTALLA_H */
