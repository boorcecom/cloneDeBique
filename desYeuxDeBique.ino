#include <mcp_can.h>

/*****************************************************************************
 *          Des yeux de bique !
 *          Application Arduino pour monitoring CAN 
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
  
******************************************************************************
   Communication Serial over USB : 
   Configuration du port série : 
   115200 8N1
******************************************************************************
    
*/

/* Section paramétrage */

// Connexion pins 
const int interruptCAN1 = 2; // Port 2 pour interruption DATA CAN1
const int interruptCAN2 = 3; // Port 3 pour interruption DATA CAN2
const int portCAN1 = 5; // Le CAN1 (véhicule) est sur le port 5
const int portCAN2 = 10;// Le CAN2 (véhicule) est sur le port 10

// Variables liées au CAN
unsigned char stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Modèle de message
MCP_CAN CAN1(portCAN1); // Définit CS broche 5 pour can1
MCP_CAN CAN2(portCAN2); // Définit CS broche 10 pour can2

void setup(){

    Serial.begin(115200); // il faut au moins ça !    
    CAN1.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz
    CAN2.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz

    attachInterrupt(digitalPinToInterrupt(interruptCAN1),CAN1_INTERRUPT,FALLING); // Mise en place de l'interruption en cas de données sur le CAN1 
    attachInterrupt(digitalPinToInterrupt(interruptCAN2),CAN2_INTERRUPT,FALLING); // Mise en place de l'interruption en cas de données sur le CAN2

    while(!Serial) {
      ; // Attente de l'initialisation
    }
    Serial.println("Traceur démarré...");

}

void CAN1_INTERRUPT()
{
        unsigned char len = 0;
        unsigned char rxBuf[8];
        long unsigned int rxId;

        CAN1.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN1.getCanId(); // Récupère l'identifiant du message

        Serial.print("CAN-V : CANID - ");
        Serial.print(rxId, HEX);
        Serial.print(", CANDATA -"");
        for(byte i=0; i<len; i++) {
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

}

void CAN2_INTERRUPT()
{
        unsigned char len = 0;
        unsigned char rxBuf[8];
        long unsigned int rxId;

        CAN2.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN2.getCanId(); // Récupère l'identifiant du message
        
        Serial.print("CAN-M : CANID - ");
        Serial.print(rxId, HEX);
        Serial.print(", CANDATA -"");
        for(byte i=0; i<len; i++) {
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();        
}

void loop(){
     /* Tout est réalisé par interruptions ! */
}