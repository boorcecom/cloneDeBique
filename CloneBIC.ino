#include <mcp_can.h>
#include <EEPROM.h>
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
  Pin 4      :                          Pin 5 (PMW): CAN1
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

/* paramètre application */
/* activation/désactivation de fonctions */
bool hasEco2 = true; // Positionner à true pour activer remontée ECO2, si non a false.
bool hasClim = true; // Positionner a true pour activer remontée clim, si non a false.
// Pour les températures, si 2 éléments sont activés, alors les deux vont s'afficher alternativement
// suivant le paramètre  CycleDurationMS
bool hasEngTemp = true; // Positionner à true pour remonter la température moteur, false sinon.
bool hasExtTemp = true; // Positionner à true pour remonter la température extérieur, false sinon.

// Ici on paramètre combiens de temps on veux voir les températures (cas avec 2 température à true):
unsigned int CycleDurationMS=10000; // Environ 10s d'affichage 
unsigned int refreshTime=1000; // Temps entre 2 affichage

/* Variables pour le fonctionnement de l'applicaiton */
// Gestion du temps de réinitialisation de l'affichage de température.
const unsigned int waitReinitTempMS=10; // 10ms d'attente. 

unsigned long currentMillis=0;
unsigned long oldMillis=0;

// Gestion de l'affichage de la température (cycle entre 2 type de températures)
unsigned long beginCycleTimeMS = 0;
unsigned long runningCycleTimeMS = 0;

// Variables liées à l'affichage des températures.
unsigned int temp = 0x7F;
unsigned int newTemp= 0x7F;
unsigned int engTemp = 0x7F;
unsigned int extTemp = 0x7F;

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
   configureInterface();
}

void configureInterface()
{
   Serial.flush();
   Serial.println("CloneDeBique serial configuration interface.");
   Serial.println("Press any key on in next 5s to configure.");
   unsigned long initSerialWait = millis();
   unsigned long serialWaitMillis= millis();
   while((serialWaitMillis-initSerialWait)<5000) {
     if(Serial.available()>0) {
       byte tempo=Serial.read();
       Serial.flush();
       menuConfig();
       break;
     }
   }

}

void menuConfig(){
  Serial.println("######################################################################");
  Serial.println("Welcome in the CloneDeBique configuration interface.");
  Serial.println("Each option will be reviewed.");
  Serial.println("For activable functions, press + to activate, or - to unactivate (Enter to pass)");
  Serial.println("For value options, as times, enter numerical data.");
  Serial.println("######################################################################");
  hasEco2=serialSelectableOption("Activate eco² information forwarding ?",hasEco2);
  hasExtTemp=serialSelectableOption("Activate External temperatur forwarding ?",hasExtTemp);
  hasEngTemp=serialSelectableOption("Activate Engin temp as external temperature forwarding ?",hasEngTemp);
  hasClim=serialSelectableOption("Activate automatic climatisation information forwarding ?",hasClim);
  Serial.println("The next value are timing values in ms.");
  Serial.println("The first one defines the time eatch temperature is displayed (engine, exterior, etc.) : 10s by default");
  Serial.println("The second one defines the time of refresh for one temperature, 1s by default");
  unsigned int initialList[4]={5000,10000,20000,40000};
  CycleDurationMS=serialGetNewUIntValue("When on or more temperature are activated, time (in ms) one temp stays displayed before switching to the other one.",CycleDurationMS,initialList);
  initialList[0]=250;initialList[1]=500;initialList[2]=1000;initialList[3]=2000;
  refreshTime=serialGetNewUIntValue("Time (in ms) before CloneDeBique send the next temperature value (buffering) (< to time between to different temperature source)",refreshTime,initialList);
  Serial.println("saving configuration....");
  EEPROM.write(0,100);
  eepromWriteUInt(3,CycleDurationMS);
  eepromWriteUInt(7,refreshTime);
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
  EEPROM.write(1,writeData);
  EEPROM.write(2,0);
  Serial.println("Configuration saved, running in BIC mode now");
  Serial.flush();  
}

bool serialSelectableOption(char *question, bool actualValue) {
  Serial.print(question);
  if(actualValue) {
    Serial.println("(Activated)");
  } else {
    Serial.println("(Desactivated)");
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

unsigned long serialGetNewUIntValue(char *question, unsigned int actualValue,unsigned int inputChoice[4]) {
  Serial.print(question);
  Serial.print(" (");
  Serial.print(actualValue);
  Serial.println("ms).");
  Serial.println("Choose value by pressing choice number (Enter to cancel) :");
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
  Serial.println("Choice ? [1-4]");
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

//write word to EEPROM
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
  if(EEPROM.read(0)!=100) {
    EEPROM.write(0,100);
    eepromWriteUInt(3,CycleDurationMS);
    eepromWriteUInt(7,refreshTime);
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
    EEPROM.write(1,writeData);
    EEPROM.write(2,0);
  }
  CycleDurationMS=eepromReadUInt(3);
  refreshTime=eepromReadUInt(7);
  byte readData=EEPROM.read(1);
  hasEco2=((readData & B00000001)==B00000001);
  hasExtTemp=((readData & B00000010)==B00000010);
  hasEngTemp=((readData & B00000100)==B00000100);
  hasClim=((readData & B00001000)==B00001000);
}

void checkCAN1()
{
        unsigned char len = 0;
        unsigned char rxBuf[8];
        long unsigned int rxId;

        CAN1.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN1.getCanId(); // Récupère l'identifiant du message
        
        if(rxId == 0x5DA && hasEngTemp ){  // Récupération de la température moteur.
            if(rxBuf[0]!=0xFF) {
              engTemp=rxBuf[0]; // On stock pour traitement ultérieur !
            }
        }
        
        if(rxId == 0x3B7 && hasExtTemp){ // Récupération de la température exterieur.
            if(rxBuf[0]!=0xFF) {
              extTemp=rxBuf[0]; // On stock pour traitement ultérieur !
            }
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
  
    if(currentMillis<=oldMillis) { // gestion de l'overflow de la fonction millis() : On réinitialise aux valeurs par défaut les compteurs temps.
      oldMillis=millis();
      currentMillis = millis();
      beginCycleTimeMS=millis();
      runningCycleTimeMS=millis();      
    } else { // cas standard : préparation de la détection de l'overflow.
      oldMillis=currentMillis;      
      currentMillis = millis();
    }
    // Gestion du cycle d'affichage : Remise à 0 si on est à 2xdurée du cycle
    if( ((currentMillis - beginCycleTimeMS)  > 2*CycleDurationMS) && hasEngTemp && hasExtTemp) {
          beginCycleTimeMS=millis();
    }

    if(!digitalRead(interruptCAN1)) { // Une donnée sur CAN1 ?
      checkCAN1();
    }

/* Non utilisé pour le moment    
    if(!digitalRead(interruptCAN2)) { // Une donnée sur CAN2 ?
      checkCAN2();
    }
*/
 
    if((((currentMillis - beginCycleTimeMS) < CycleDurationMS) && hasEngTemp) || (hasEngTemp && !hasExtTemp)) { // Si nous sommes sous la durée du cycle paramétré :
      newTemp = engTemp; // La nouvelle température est celle du moteur.
    } else if((((currentMillis- beginCycleTimeMS)  >CycleDurationMS) && hasExtTemp) || (!hasEngTemp && hasExtTemp)) {  // Si non
      newTemp = extTemp; // La nouvelle température est celle de l'extérieur.                
    }
        
    if((hasEngTemp || hasExtTemp) && (currentMillis-runningCycleTimeMS>refreshTime) ) { // Nous devons rafraîchir l'affichage de la température !
      runningCycleTimeMS=millis();
      stmp[2]=0xFF;
      CAN2.sendMsgBuf(0x558, 0, 8, stmp); // On envoie la réinitialisation.
      delay(waitReinitTempMS);
      temp=newTemp; // On positionne la nouvelle température
      stmp[2]=temp;
      CAN2.sendMsgBuf(0x558, 0, 8, stmp);    // On envoie la température au MediaNav.
    }
}
