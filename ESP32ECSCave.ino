#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Adafruit_ADS1015.h>

#define OTAName                    "ESP32ECSCave" // Url domain weMos ECS Cave
#define WifiSSID                   "TBSOFT"       // Nom  SSID
#define WifiPass                   "TB04011966"   // Pass SSID
#define IP                         54             // Ip = 192.168.0.54

#define P0                         16             // N° Pin WeMos
#define P1                         5              // N° Pin WeMos
#define P2                         4              // N° Pin WeMos
#define P3                         0              // N° Pin WeMos
#define P4                         2              // N° Pin WeMos
#define P5                         14             // N° Pin WeMos
#define P6                         12             // N° Pin WeMos
#define P7                         13             // N° Pin WeMos
#define P8                         15             // N° Pin WeMos
#define LIBRE                      P5             // N° Pin WeMos
#define ONE_WIRE_BUS               P3             // N° Pin WeMos
#define LED                        P4             // N° Pin WeMos
#define PACEN                      P0             // N° Pin WeMos
#define PACRES                     P6             // N° Pin WeMos
#define EXTRES                     P7             // N° Pin WeMos
#define SSR                        P8             // N° Pin WeMos

#define ADCMaxInterval             160            // lecture de Irms toutes les 150 micro-secondes pour atteindre environ 3300 lec/s
#define readsMaxInterval           1000           // calcul de Irms toutes les 1s -> somme de ADCread*ADCread 
#define alphaWattMaxInterval       10000          // si watts<100 sur PacRes avec alpha>0 pendant plus de 10s => la bilame PacRes s'est déclenchée
#define state0MaxInterval          1200000        // 15min avant retour a wattState 1 (PacRes) quand avgWatts < minCircuWatts
#define pacEnMaxInterval           43200000       // 12h max de chauffage par PAC
#define wifiMaxInterval            60000          // si pas de reception alpha pendant 1min => on met alpha = 0
#define tempMaxInterval            60000          // lecture de temperatures toutes les minutes
#define httpHandleMaxInterval      333            // appel de server.handleClient(); ArduinoOTA.handle(); toutes les 333ms
#define pwmMinFreq                 3              // frequance correspondant à alpha = 0+
#define pwmMaxFreq                 20             // frequance correspondant à alpha = 1

#define VtoA30                     30             // convertion tension-intensite de la bobine de mesure
#define minCircuWatts              250            // 250w de limite inférieure pour le passage en wattState 1 (PacRes) 
#define maxExtResPower             0.6            // alpha correspondant à la puissance max autorisée sur ExtRes
#define PacResWatts                2100           // 150w de limite inférieure pour le passage en wattState 1 (PacRes) 
#define ExtResWatts                7300           // 150w de limite inférieure pour le passage en wattState 1 (PacRes) 
#define minReads                   1700           // nombre min de lectures ADC pour calculer un Irms valable 

float consigneEcsPac               = 45;          // consigne de temp du chauffage par PAC
float consigneEcsResMin            = 55;          // consigne de temp pour re-mise en route du chauffage par ExtRes
float consigneEcsResMax            = 60;          // consigne de temp pour arret du du chauffage par ExtRes
float alpha                        = 0;           // tx de hachage du SSD
float ECStemp                      = 0;           // temperature ECS 
float CAVEtemp                     = 0;           // temperature Cave (du WeMos)
float watts                        = 0;           // puissance instantanée calculée avec Irms et Vrms
float totA2                        = 0;           // totalisation des puissances instantanées pour le calcul de Irms
float ADCFactor                    = 0;           // ADCFactor = ADCCorrector*ADCmultiplier*VtoA30
float ADCmultiplier                = 0;           // ADCFactor = ADCCorrector*ADCmultiplier*VtoA30
float Irms                         = 0;           // Irms = sqrt(totA2/reads)
float Vrms                         = 230;         // Vrms est receptionnée en meme temps que alpha en wifi

unsigned long alphaWattTimer       = millis();    // Timer pour le delais min quand alpha > 0.1 && avgWatts <100 (declanchement bilame PacRes)
unsigned long circuWattsTimer      = millis();    // Timer pour le passage à PacRes quand avgWatts < minCircuWatts
unsigned long wifiTimer            = millis();    // Timer pour le delais min quand plus de reception wifi
unsigned long readsTimer           = millis();    // Timer pour le calcul de Irms
unsigned long tempTimer            = millis();    // Timer pour les delais de re-lecture des température
unsigned long pacEnTimer           = millis();    // Timer pour le delais max de chaffage par PAC
unsigned long httpHandleTimer      = millis();    // Timer pour l'appel de server.handleClient(); ArduinoOTA.handle();
unsigned long ADCTimer             = micros();    // Timer pour la re-lecture ads1015.getLastConversionResults()

unsigned long reads                = 0;           // nombre de lectures ads1015.getLastConversionResults()
unsigned long readsBak             = 0;           // nombre de lectures ads1015.getLastConversionResults() -> Sauvegarde

unsigned long Counter              = 0;           // Compteur 
float         Average              = 0;           // Moyenne 

int wattState                      = 1;           // PacRes = 1 / ExtRes = 2
int pwmFreq                        = 3;           // SSR -> Pwm Frequency
int wattZero                       = 0;           // compteur du nombre de fois que watts == 0 avant de conclure que watts == 0
int minReadsCounter                = 0;           // compteur du nombre de fois que reads < minReads
int wattZeroCounter                = 0;           // compteur du nombre de fois que watt=0 quand alpha > 0
int wattStateCounter               = 0;           // compteur du nombre de fois que wattState change
int consigneEcsResMaxCounter       = 0;           // compteur du nombre de fois que consigneEcsResMax est depassé
int ADCread                        = 0;           // Stockage de la valeur ads1015.getLastConversionResults()
//int16_t ADCread                  = 0;           // Stockage de la valeur ads1015.getLastConversionResults()

bool startAlphaWattTimer           = false;       // true quand AlphaWattTimer à démaré
bool modePacRes                    = false;       // true en mode PacRes uniquement (pas de ExtRes)
bool modeDegrade                   = false;       // true en mode dégradé (pas de ExtRes) car ECStemp > consigneEcsRes

// EEPROM variables
float ADCCorrector                 = 1.0950000;   // Correcteur pour ads1015 => ADCFactor = ADCCorrector*ADCmultiplier*VtoA30
float ECSTempCorrector             = 1.0222200;   // Correcteur pour temperature ECS
float CAVETempCorrector            = 10.000000;   // Correcteur pour temperature Cave (WeMos)

// Class instances
ESP8266WebServer server(80); OneWire oneWire(ONE_WIRE_BUS); DallasTemperature sensors(&oneWire, ONE_WIRE_BUS); Adafruit_ADS1015 ads1015; 

void loop() {         
  if (millis() - httpHandleTimer > httpHandleMaxInterval) { server.handleClient(); ArduinoOTA.handle(); Counter++; Average += millis() - httpHandleTimer; httpHandleTimer = millis(); }   
  if (micros() - ADCTimer        > ADCMaxInterval)        { ADCread = ads1015.getLastConversionResults()*ADCFactor; totA2 = totA2 + ADCread*ADCread; reads++; ADCTimer = micros(); } 
  if (millis() - readsTimer      > readsMaxInterval)      {  
    if (reads < minReads) minReadsCounter++; 
    if (totA2 == 0 && alpha != 0) { 
      if (wattState == 1) { 
        wattZero++;  
        if (wattZero > 5) { wattZeroCounter++; Irms = 0; watts = 0;} }
      else { watts = alpha * ExtResWatts; Irms = watts / Vrms; } }  
    else { Irms = sqrt(totA2/reads); watts = Vrms*Irms; watts = watts * watts / ((digitalRead(EXTRES)==LOW)? ExtResWatts:PacResWatts); } 
    if (!modePacRes && !modeDegrade) {
      if (wattState == 1) { // wattState == 1  
        if (startAlphaWattTimer) {
          if (millis() - alphaWattTimer > alphaWattMaxInterval)  { wattState = 2; if (alpha > maxExtResPower) alpha = maxExtResPower; digitalWrite(EXTRES, LOW ); digitalWrite(PACRES, LOW); wattStateCounter++; startAlphaWattTimer = false;  circuWattsTimer=millis(); }               
          else if (watts > 0) startAlphaWattTimer = false; 
        } else if (alpha > 0.1 && watts == 0) { startAlphaWattTimer = true; alphaWattTimer = millis(); } 
      } else {              // wattState == 2
        if (watts > minCircuWatts) circuWattsTimer=millis(); 
        else if (millis() - circuWattsTimer > state0MaxInterval) { wattState = 1; digitalWrite(EXTRES, HIGH); digitalWrite(PACRES, HIGH); startAlphaWattTimer = false; circuWattsTimer=millis(); }            
      }
    }
    if (millis() - tempTimer      > tempMaxInterval)     {     
      if (millis() - wifiTimer    > wifiMaxInterval)     { alpha = 0; analogWrite(SSR, alpha); wifiTimer = millis(); }  
      if (millis() - pacEnTimer   > pacEnMaxInterval)    { digitalWrite(PACEN, HIGH); pacEnTimer = millis(); } 
      CAVEtemp=sensors.getTempCByIndex(0)*ECSTempCorrector-CAVETempCorrector;  
      ECStemp=sensors.getTempCByIndex(1)*ECSTempCorrector;                 
      sensors.requestTemperatures();       
      if (ECStemp > consigneEcsPac && digitalRead(PACEN) == LOW) digitalWrite(PACEN, HIGH); 
      // if (pacBottomTemp < maxPacBottomTemp)        { modePacRes = false; digitalWrite(LED, HIGH); }
      // if (ECStemp < consigneEcsResMin && modePacRes)  { modePacRes = false; digitalWrite(LED, HIGH);  }
      if (ECStemp > consigneEcsResMax && !modePacRes) { modePacRes = true;  digitalWrite(LED, LOW); wattState = 1; digitalWrite(EXTRES, HIGH); digitalWrite(PACRES, HIGH); consigneEcsResMaxCounter++; }
      // if (ExtResTemp > maxExtResTemp)              { modePacRes = true;  digitalWrite(LED, LOW); wattState = 1; digitalWrite(EXTRES, HIGH); digitalWrite(PACRES, HIGH); maxExtResTemp++; }
      tempTimer=millis(); 
    }                   
    readsBak=reads; reads=0; totA2=0; readsTimer=millis(); 
  }
}
void root() { 
  if (server.arg("alpha" )!="") { alpha = server.arg("alpha").toFloat(); 
    if (wattState == 2 && alpha > maxExtResPower) alpha = maxExtResPower;
    pwmFreq = pwmMinFreq+round(alpha*(pwmMaxFreq-pwmMinFreq)); analogWriteFreq(pwmFreq); analogWrite(SSR, alpha*255); 
  }   
  if (server.arg("wattState" )!="") { wattState = server.arg("wattState").toInt(); 
    if (wattState == 2) { digitalWrite(EXTRES, LOW ); digitalWrite(PACRES, LOW);  }
    else                { digitalWrite(EXTRES, HIGH); digitalWrite(PACRES, HIGH); }
    startAlphaWattTimer = false; alphaWattTimer = millis(); circuWattsTimer = millis();
  }    
  if (server.arg("RazCounters"  )!="")      { Counter = 0; Average = 0; minReadsCounter = 0; wattZeroCounter = 0; wattStateCounter = 0; consigneEcsResMaxCounter = 0; }
  if (server.arg("Vrms"  )!="")             { Vrms = server.arg("Vrms").toFloat(); } 
  if (server.arg("consigneEcsPac" )!="")    { consigneEcsPac = server.arg("consigneEcsPac").toFloat(); }  
  if (server.arg("consigneEcsResMax" )!="") { consigneEcsResMax = server.arg("consigneEcsResMax").toFloat(); }   
  if (server.arg("consigneEcsResMin" )!="") { consigneEcsResMin = server.arg("consigneEcsResMin").toFloat(); }
  if (server.arg("pwmFreq" )!="")           { pwmFreq = server.arg("pwmFreq").toFloat(); analogWriteFreq(pwmFreq); }  
  if (server.arg("modeDegrade" )!="")       { modeDegrade = (server.arg("modeDegrade")=="true")? true:false; if (modeDegrade) { wattState = 1; digitalWrite(LED, LOW); digitalWrite(EXTRES, HIGH); digitalWrite(PACRES, HIGH); } else digitalWrite(LED, HIGH);}
  if (server.arg("modePacRes" )!="")        { modePacRes = (server.arg("modePacRes")=="true")? true:false; }  
  if (server.arg("PACen" )!="")             { digitalWrite(PACEN, (server.arg("PACen")=="true")? LOW:HIGH); pacEnTimer = millis(); }  
  
  sendJsonResponse(); RelayTests(); wifiTimer = millis(); 
}
void RelayTests() {
  if (server.arg("EXTRES")!="") { digitalWrite(EXTRES, LOW); delay(1000); digitalWrite(EXTRES, HIGH); }  
  if (server.arg("PACRES")!="") { digitalWrite(PACRES, LOW) ; delay(1000); digitalWrite(PACRES, HIGH); }  
  if (server.arg("LIBRE" )!="") { digitalWrite(LIBRE,  LOW); delay(1000); digitalWrite(LIBRE,  HIGH); }  
  if (server.arg("Relais")!="") { digitalWrite(LIBRE,  LOW); digitalWrite(PACEN,  LOW); digitalWrite(PACRES,  LOW); digitalWrite(EXTRES,  LOW); delay(2000); digitalWrite(LIBRE,  HIGH); digitalWrite(PACEN,  HIGH); digitalWrite(PACRES,  HIGH); digitalWrite(EXTRES,  HIGH); }     
}
void sendJsonResponse() { 
  setHeaders(); server.send(200, "application/json", "{ "
  "\"api\": \"ecsCaveStatus\", "
  "\"ECStemp\": " + String(ECStemp, 2) + ", "
  "\"alpha\": " + String(alpha, 6) + ", "
  "\"watts\": " + String(watts) + ", "  
  "\"reads\": " + String(readsBak) + ", "   
  "\"PACen\": " + ((digitalRead(PACEN)==HIGH)? "false":"true") + ", "
  "\"circuAndExtRes\": " + ((digitalRead(EXTRES)==HIGH)? "false":"true") + ", "
  "\"modePacRes\": " + ((modePacRes)? "true":"false") + ", "
  "\"modeDegrade\": " + ((modeDegrade)? "true":"false") + " ,"
  "\"wattState\": " + String(wattState) + " }"); 
}
void getExtraParams() {
  setHeaders(); server.send(200, "application/json", "{ "  
  "\"api\": \"ecsCaveExtraParams\", "
  "\"ECStemp\": " + String(ECStemp, 2) + ", "
  "\"alpha\": " + String(alpha, 6) + ", "
  "\"watts\": " + String(watts) + ", "  
  "\"reads\": " + String(readsBak) + ", "   
  "\"PACen\": " + ((digitalRead(PACEN)==HIGH)? "false":"true") + ", "
  "\"circuAndExtRes\": " + ((digitalRead(EXTRES)==HIGH)? "false":"true") + ", "
  "\"modePacRes\": " + ((modePacRes)? "true":"false") + ", "
  "\"modeDegrade\": " + ((modeDegrade)? "true":"false") + ", "  
  "\"pwmFreq\": " + String(pwmFreq) + ", "  
  "\"CAVEtemp\": " + String(CAVEtemp, 2) + ", "
  "\"consigneEcsPac\": " + String(consigneEcsPac, 2) + ", "
  "\"consigneEcsResMin\": " + String(consigneEcsResMin, 2) + ", "  
  "\"consigneEcsResMax\": " + String(consigneEcsResMax, 2) + ", "
  "\"Irms\": " + String(Irms, 2) + ", "   
  "\"Vrms\": " + String(Vrms) + ", "
  "\"minReadsCounter\": " + String(minReadsCounter) + ", " 
  "\"wattZeroCounter\": " + String(wattZeroCounter) + ", " 
  "\"consigneEcsResMaxCounter\": " + String(consigneEcsResMaxCounter) + ", "   
  "\"Average\": " + String(Average/Counter, 0) + ", "             
  "\"wattStateCounter\": " + String(wattStateCounter) + ", " 
  "\"wattState\": " + String(wattState) + " }");   
}
void cors()             { setHeaders(); server.send(200, "text/plain", "" ); }
void handleNotFound()   { if (server.method() == HTTP_OPTIONS) { setHeaders(); server.send(204); } else server.send(404, "text/plain", ""); }
void redirectToRoot()   { server.sendHeader("Location", "/",true); server.send(302, "text/html",""); }
void paramsGet()        { setHeaders(); server.send(200, "application/json", "{ \"api\": \"ecsCaveParamsGet\", \"ECSTempCorrector\": " + String(ECSTempCorrector,6) + ", \"CAVETempCorrector\": " + String(CAVETempCorrector,6) + ", \"ADCCorrector\": " + String(ADCCorrector,6) + " }"); }
void paramsSet()        { 
  if (server.arg("ADCCorrector")!="")      ADCCorrector      = server.arg("ADCCorrector" ).toFloat(); ADCFactor         = ADCCorrector*ADCmultiplier*VtoA30/1000;
  if (server.arg("ECSTempCorrector")!="")  ECSTempCorrector  = server.arg("ECSTempCorrector" ).toFloat();
  if (server.arg("CAVETempCorrector")!="") CAVETempCorrector = server.arg("CAVETempCorrector").toFloat();      
  EEPROMWrite(); paramsGet();
}
void setHeaders() {
  server.sendHeader("Access-Control-Max-Age", "10000");
  server.sendHeader("Access-Control-Allow-Methods", "GET,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");  
  server.sendHeader("Access-Control-Allow-Origin","*");      
}
void EEPROMRead() { int eeAddress = 0; 
  EEPROM.get(eeAddress, ADCCorrector);      eeAddress += sizeof(ADCCorrector);    
  EEPROM.get(eeAddress, ECSTempCorrector);  eeAddress += sizeof(ECSTempCorrector);  
  EEPROM.get(eeAddress, CAVETempCorrector); eeAddress += sizeof(ECSTempCorrector);    
}
void EEPROMWrite() { int eeAddress = 0; 
  EEPROM.put(eeAddress, ADCCorrector);      eeAddress += sizeof(ADCCorrector);      
  EEPROM.put(eeAddress, ECSTempCorrector);  eeAddress += sizeof(ECSTempCorrector);        
  EEPROM.put(eeAddress, CAVETempCorrector); eeAddress += sizeof(CAVETempCorrector); 
  EEPROM.commit();   
}
void setup() {  
  Serial.begin(9600); Serial.println(""); Serial.println("Esp32EcsCave Setup Start");
  EEPROM.begin(32); //EEPROMWrite(); // Doit être activé lors du premier lancement pour initialiser l'EEPROM avec les valeurs du programme
  //EEPROMRead();  
  pinMode(LED,    OUTPUT); 
  pinMode(SSR,    OUTPUT); 
  pinMode(PACEN,  OUTPUT);   
  pinMode(PACRES, OUTPUT);   
  pinMode(EXTRES, OUTPUT);        
  pinMode(LIBRE,  OUTPUT);    
  digitalWrite(LED,    HIGH);  
  digitalWrite(SSR,    HIGH);  
  digitalWrite(PACEN,  HIGH);
  digitalWrite(PACRES, HIGH);
  digitalWrite(EXTRES, HIGH);   
  digitalWrite(LIBRE,  HIGH);     

  analogWriteFreq(pwmFreq); 

  WiFi.config(IPAddress(192, 168, 0, IP), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0));
  WiFi.hostname(OTAName); WiFi.mode(WIFI_STA); WiFi.begin(WifiSSID, WifiPass);
  while (WiFi.status() != WL_CONNECTED) { delay(250); }

  ArduinoOTA.setHostname(OTAName); ArduinoOTA.begin();  

  server.on("/",               root); server.on("/", HTTP_OPTIONS, cors);
  server.on("/paramsGet",      paramsGet);    
  server.on("/paramsSet",      paramsSet);  
  server.on("/getExtraParams", getExtraParams);    
  server.onNotFound(handleNotFound);
  server.begin(); 
  
  Wire.setClock(400000); ads1015.setGain(GAIN_FOUR); 
  ADCmultiplier = 0.5F; ADCFactor = ADCCorrector*ADCmultiplier*VtoA30/1000;
  ads1015.begin(); ads1015.startContinuousMode_DIFF_2_3();  
  
  sensors.begin(); sensors.setResolution(9); sensors.requestTemperatures(); 
  CAVEtemp=sensors.getTempCByIndex(0)*ECSTempCorrector-CAVETempCorrector;  
  ECStemp=sensors.getTempCByIndex(1)*ECSTempCorrector; 
  sensors.setWaitForConversion(false);
  Serial.println("Esp32EcsCave Setup End");
}

