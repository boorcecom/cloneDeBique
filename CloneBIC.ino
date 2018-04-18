#include <mcp_can.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*****************************************************************************
 *          Clone de bique !
 *          Application Arduino bridge CAN : 
 *          spécifique pour CAN Véhicule + CAN multimédia.
 *          
******************************************************************************
 Mapping des Arduino (ici Nano et Mini Pro)
******************************************************************************
  Pin 0 (RX) :                          Pin 1 (TX) : 
  Pin 2      : Interruption CAN1        Pin 3 (PMW): Interruption CAN2 
  Pin 4      : Interface Temp. Int.     Pin 5 (PMW): CAN1
  Pin 6 (PMW):                          Pin 7      :
  Pin 8      :                          Pin 9 (PMW):
  Pin 10(PMW): CAN2                     Pin 11(PWM): CAN SPI MOSI (SI sur cartes CAN)
  Pin 12     : CAN SPI MISO (SO sur CAN)Pin 13     : CAN SPI CLOCK (SCK sur cartes CAN)
  Pin 14 A0  :                          Pin 15 A1  :
  Pin 16 A2  :                          Pin 17 A3  :
  Pin 18 A4  :                          Pin 19 A5  : 
******************************************************************************
  Pour le CAN1 : Le CANH est reliée au port 6 de l'ODB2
  Pour le CAN1 : Le CANL est reliée au port 14 de l'ODB2
  Pour le CAN2 : Le CANH est reliée au port 13 de l'ODB2
  Pour le CAN2 : Le CANL est reliée au port 12 de l'ODB2
  
  L'alimentation (/!\ entre 13 et 15v ) via un régulateur pour abaisser à
  8-12v est connecté aux broches 16 (+power) 4 et  5 (GND)
  
  
*/

/* Section paramétrage */

// Connexion pins 
const int interruptCAN1 = 2; // Port 2 pour interruption DATA CAN1
const int interruptCAN2 = 3; // Port 3 pour interruption DATA CAN2
const int portCAN1 = 5; // Le CAN1 (véhicule) est sur le port 5
const int portCAN2 = 10;// Le CAN2 (véhicule) est sur le port 10
const int oneWireTemp = 4;

/* Version pour la configuration */
const byte cdbVersion=101;

/* paramètre application */
/* activation/désactivation de fonctions */
bool hasEco2 = true; // Positionner à true pour activer remontée ECO2, si non a false.
bool hasClim = true; // Positionner a true pour activer remontée clim, si non a false.
// Pour les températures, si 2 éléments sont activés, alors les deux vont s'afficher alternativement
// suivant le paramètre  CycleDurationMS
bool hasEngTemp = true; // Positionner à true pour remonter la température moteur, false sinon.
bool hasExtTemp = true; // Positionner à true pour remonter la température extérieur, false sinon.
bool hasIntTemp = false; // Positionner à true pour remonter la température intérieur via OneWire, false sinon.

// Ici on paramètre combiens de temps on veux voir les températures (cas avec 2 température à true):
unsigned long cycleDurationMS=10000; // Environ 10s d'affichage 
unsigned long refreshTime=1000; // Temps entre 2 affichage

/* Variables pour le fonctionnement de l'applicaiton */

unsigned long currentMillis=0;
unsigned long oldMillis=0;

// Gestion de l'affichage de la température (cycle entre 2 type de températures)
unsigned long beginCycleTimeMS = 0;
unsigned long runningCycleTimeMS = 0;


// Variables liées à l'affichage des températures.
unsigned int oldTemp = 0x7F;
unsigned int newTemp= 0x7F;
unsigned int engTemp = 0x7F;
unsigned int extTemp = 0x7F;
unsigned int intTemp = 0x7F;

unsigned long numberTempSource=1;
unsigned int tempCounter=0;
unsigned int tempArray[3] = { 0x7F, 0x7F, 0x7F };

OneWire OneWire(oneWireTemp);//capteur de température 4
DallasTemperature sensors(&OneWire);

// Variables liées au CAN
unsigned char stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Modèle de message
MCP_CAN CAN1(portCAN1); // Définit CS broche 5 pour can1
MCP_CAN CAN2(portCAN2); // Définit CS broche 10 pour can2

void setup(){
    byte counter=0;

    Serial.begin(9600);
    
    CAN1.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz
    CAN2.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz
    loadConfiguration();
}

void menuConfig(){
  Serial.println(F("Welcome in the CloneDeBique configuration interface."));
  Serial.println(F("For activable functions, press + to activate, or - to unactivate"));
  Serial.println(F("For value options, as times, enter numerical data."));
  hasEco2=serialSelectableOption("Activate eco² info forwarding ?",hasEco2);
  hasExtTemp=serialSelectableOption("Activate External temp forwarding ?",hasExtTemp);
  hasEngTemp=serialSelectableOption("Activate Engin temp as external temp forwarding ?",hasEngTemp);
  hasIntTemp=serialSelectableOption("Activate Internal Vehicule temp (external sensor on pin 4) as external temp forwarding ?",hasIntTemp);
  hasClim=serialSelectableOption("Activate automatic clim info forwarding ?",hasClim);
  Serial.println(F("The next value are timing values in ms."));
  Serial.println(F("The first one defines the time eatch temp is displayed (engine, exterior, etc.) : 10s by default"));
  Serial.println(F("The second one defines the time of refresh for one temperature, 1s by default"));
  unsigned long initialList[4]={10000,20000,30000,40000};
  cycleDurationMS=serialGetNewUIntValue("When on or more temperature are activated, time (in ms) one temp stays displayed before switching to the other one.",cycleDurationMS,initialList);
  initialList[0]=100;initialList[1]=2000;initialList[2]=3000;initialList[3]=4000;
  refreshTime=serialGetNewUIntValue("Time (in ms) before CloneDeBique send the next temperature value (buffering) (< to time between to different temperature source)",refreshTime,initialList);
  Serial.println(F("saving configuration...."));
  EEPROM.write(0,cdbVersion);
  eepromWriteULong(3,cycleDurationMS);
  eepromWriteULong(7,refreshTime);
  byte writeData = B00000000;
  if(hasEco2) {
    writeData = writeData | B00000001;
  }
  if(hasExtTemp) {
    writeData = writeData | B00000010;
  }
  if(hasEngTemp) {
    writeData = writeData | B00000100;
  }
  if(hasClim) {
    writeData = writeData | B00001000;
  }
  if(hasIntTemp) {
    writeData = writeData | B00010000;
  }  
  EEPROM.write(1,writeData);
  EEPROM.write(2,0);
  Serial.println(F("Configuration saved, running in BIC mode now"));
  Serial.flush();
  Serial.read();
  loadConfiguration();  
}

bool serialSelectableOption(char *question, bool actualValue) {
  Serial.print(question);
  if(actualValue) {
    Serial.println(F("(Activated)"));
  } else {
    Serial.println(F("(Desactivated)"));
  }
  byte readChar=0;
  while(readChar!=43 && readChar!=45 && readChar!=13 ) {
    readChar=Serial.read();
  }
  if(readChar==43) {
    return true;
  } else if(readChar==45) {
    return false;
  } else {
    return actualValue;
  }
}

unsigned long serialGetNewUIntValue(char *question, unsigned long actualValue,unsigned long inputChoice[4]) {
  Serial.print(question);
  Serial.print(F(" ("));
  Serial.print(actualValue);
  Serial.println(F("ms)."));
  Serial.println(F("Choose value by pressing choice number (Enter to cancel) :"));
  for(unsigned int counter=0;counter<4;counter++){
    if(inputChoice[counter]==actualValue) {
      Serial.print("*");
    } else {
      Serial.print(" ");
    }
    Serial.print(" ");
    Serial.print(counter+1);
    Serial.print(": ");
    Serial.println(inputChoice[counter]);
  }
  Serial.println(F("Choice ? [1-4]"));
  byte readChar=0;
  while(!(readChar>48 && readChar<53) && readChar!=13 ) {
    readChar=Serial.read();
  }
  if(readChar>48 && readChar<53) {
    return inputChoice[readChar-49];
  } else {
    return actualValue;
  }  
}

unsigned long eepromReadULong(int address)
{
 //use word read function for reading upper part
 unsigned long dword = eepromReadUInt(address);
 //shift read word up
 dword = dword << 16;
 // read lower word from EEPROM and OR it into double word
 dword = dword | eepromReadUInt(address+2);
 return dword;
}

void eepromWriteULong(int address, unsigned long value)
{
 //truncate upper part and write lower part into EEPROM
 eepromWriteUInt(address+2, word(value));
 //shift upper part down
 value = value >> 16;
 //truncate and write
 eepromWriteUInt(address, word(value));
}


void eepromWriteUInt(int address, unsigned int value) 
{
   EEPROM.write(address,highByte(value));
   EEPROM.write(address+1 ,lowByte(value));
}

unsigned int eepromReadUInt(int address) 
{
   unsigned int word = word(EEPROM.read(address), EEPROM.read(address+1));
   return word;
}

void loadConfiguration() {
  if(EEPROM.read(0)!=cdbVersion AND EEPROM.read(0)!=cdbVersion-1) {
    EEPROM.write(0,cdbVersion);
    eepromWriteULong(3,cycleDurationMS);
    eepromWriteULong(7,refreshTime);
    byte writeData = B00000000;
    if(hasEco2) {
      writeData = writeData | B00000001;
    }
    if(hasExtTemp) {
      writeData = writeData | B00000010;
    }
    if(hasEngTemp) {
      writeData = writeData | B00000100;
    }
    if(hasClim) {
      writeData = writeData | B00001000;
    }
    if(hasIntTemp) {
      writeData = writeData | B00010000;
    }  
    EEPROM.write(1,writeData);
    EEPROM.write(2,0);
  } 
  cycleDurationMS=eepromReadULong(3);
  refreshTime=eepromReadULong(7);
  byte readData=EEPROM.read(1);
  hasEco2=((readData & B00000001)==B00000001);
  hasExtTemp=((readData & B00000010)==B00000010);
  hasEngTemp=((readData & B00000100)==B00000100);
  hasClim=((readData & B00001000)==B00001000);
  hasIntTemp=((readData & B00010000)==B00010000);
  if(hasExtTemp&&hasEngTemp&&hasIntTemp) {
    numberTempSource=3;
  } else if ((!hasExtTemp&&hasEngTemp&&hasIntTemp)||(hasExtTemp&&!hasEngTemp&&hasIntTemp)||(hasExtTemp&&hasEngTemp&&!hasIntTemp)) {
    numberTempSource=2;
  } else {
    numberTempSource=1;
  }
}

void checkCAN1()
{
        unsigned char len = 0;
        unsigned char rxBuf[8];
        long unsigned int rxId;

        CAN1.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN1.getCanId(); // Récupère l'identifiant du message
        
        if(hasEngTemp)  {
          if(rxId == 0x5DA && rxBuf[0]!=0xFF){  // Récupération de la température moteur.
              tempArray[tempCounter]=rxBuf[0]; // On stock pour traitement ultérieur ! 
            }
          tempCounter++;
        }
        
        if(hasExtTemp) {
            if(rxId == 0x3B7 && rxBuf[0]!=0xFF){ // Récupération de la température exterieur.
              tempArray[tempCounter]=rxBuf[0]; // On stock pour traitement ultérieur !            
            }
          tempCounter++;
        }    
        
        // Envoyer les données Driving ECO2
        if(rxId == 0x646 && hasEco2 ){
            CAN2.sendMsgBuf(0x314, 0, 8, rxBuf); // On ne cherche pas, on envoie au MediaNav.
        }
        
        // Envoyer les données Climatisation
        if(rxId == 0x699 && hasClim ){
            CAN2.sendMsgBuf(0x31B, 0, 8, rxBuf); // On ne cherche pas, on envoie au MediaNav.
        }  
}

/*
void checkCAN2() // Non utilisé pour le moment !
{
        unsigned char len = 0;
        unsigned char rxBuf[8];
        long unsigned int rxId;
        CAN2.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN2.getCanId(); // Récupère l'identifiant du message
        
}
*/
void loop(){
   tempCounter=0
   if(Serial.available()>0) {
     Serial.read();
     Serial.flush();
     menuConfig();
   }
   
   if(currentMillis<oldMillis) { // gestion de l'overflow de la fonction millis() : On réinitialise aux valeurs par défaut les compteurs temps.
      oldMillis=millis();
      beginCycleTimeMS=millis();
      runningCycleTimeMS=millis();      
    } else { // cas standard : préparation de la détection de l'overflow.
      oldMillis=currentMillis;      
    }
    currentMillis = millis();

    if(!digitalRead(interruptCAN1)) { // Une donnée sur CAN1 ?
      checkCAN1();
    } else { // Nous devons quand même faire "courir" le compteur de relevés de températures.
      if(hasExtTemp) {
        tempCounter++;
      }
      if(hasEngTemp) {
        tempCounter++;
      }      
    }

    if(hasIntTemp) {
      tempArray[tempCounter]=sensors.getTempCByIndex(0))+40;
      tempCounter++;
    }
    
/* Non utilisé pour le moment    
    if(!digitalRead(interruptCAN2)) { // Une donnée sur CAN2 ?
      checkCAN2();
    }
*/

    // Gestion du cycle d'affichage : Remise à 0 si on est à X * durée du cycle
    
    if(currentMillis > (beginCycleTimeMS+numberTempSource*cycleDurationMS)) {
      beginCycleTimeMS=millis();
    }
 
    // Récupération de la température correspondant au moment du cycle où nous sommes.
    int tempIndex=currentMillis-beginCycleTimeMS;
    tempIndex=abs(tempIndex)/cycleDurationMS;
    newTemp=tempArray[tempIndex]; 

    if((numberTempSource>0) && (currentMillis>(runningCycleTimeMS+refreshTime)) ) { // Nous devons rafraîchir l'affichage de la température !
      if(oldTemp!=newTemp) {
        stmp[2]=0xFF;
        CAN2.sendMsgBuf(0x558, 0, 8, stmp);    // On envoie la réinitialisation au MediaNav.
        delay(10);
      }
      runningCycleTimeMS=millis();
      oldTemp=newTemp;
      stmp[2]=newTemp;
      CAN2.sendMsgBuf(0x558, 0, 8, stmp);    // On envoie la température au MediaNav.
    }
}
