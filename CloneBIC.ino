#include <mcp_can.h>


/*****************************************************************************
 Mapping des Arduino (ici Nano et Mini Pro)
******************************************************************************
  Pin 0 (RX) :                          Pin 1 (TX) : 
  Pin 2      : Interruption CAN1        Pin 3 (PMW): Interruption CAN2 (non uilisé)
  Pin 4      :                          Pin 5 (PMW): CAN1
  Pin 6 (PMW):                          Pin 7      :
  Pin 8      :                          Pin 9 (PMW):
  Pin 10(PMW): CAN2                     Pin 11(PWM):
  Pin 12     :                          Pin 13     :
  Pin 14 A0  :                          Pin 15 A1  :
  Pin 16 A2  :                          Pin 17 A3  :
  Pin 18 A4  :                          Pin 19 A5  : 
*/

/* Section paramétrage */

// Connexion pins 
const int interruptCAN1 = 2; // Port 2 pour interruption DATA CAN1
const int interruptCAN2 = 3; // Port 3 pour interruption DATA CAN2
const int portCAN1 = 5; // Le CAN1 (véhicule) est sur le port 5
const int portCAN2 = 10;// Le CAN2 (véhicule) est sur le port 10

/* paramètre application */
/* activation/désactivation de fonctions */
const bool hasEco2 = true; // Positionner à true pour activer remontée ECO2, si non a false.
const bool hasClim = true; // Positionner a true pour activer remontée clim, si non a false.
// Pour les températures, si 2 éléments sont activés, alors les deux vont s'afficher alternativement
// suivant le paramètre  CycleDurationMS
const bool hasEngTemp = true; // Positionner à true pour remonter la température moteur, false sinon.
const bool hasExtTemp = true; // Positionner à true pour remonter la température extérieur, false sinon.

// Ici on paramètre combiens de temps on veux voir les températures (cas avec 2 température à true):
const unsigned long CycleDurationMS=10000; // Environ 10s d'affichage 


/* Variables pour le fonctionnement de l'applicaiton */
// Gestion du temps de réinitialisation de l'affichage de température.
unsigned long beginWaitReinitTemp = 0; 
const unsigned long waitReinitTempMS=10; // 10ms d'attente. 

// Gestion de l'affichage de la température (cycle entre 2 type de températures)
unsigned long beginCycleTimeMS = 0;
unsigned long runningCycleTimeMS = 0;

// Variables liées à l'affichage des températures.
unsigned int temp = 0x0;
unsigned int newTemp= 0x0;
unsigned int engTemp = 0x0;
unsigned int extTemp = 0x0;
bool refreshTemp = false;

// Variables liées au CAN
unsigned char len = 0;
long unsigned int rxId;
unsigned char rxBuf[8];
unsigned char stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Modèle de message
MCP_CAN CAN1(portCAN1); // Définit CS broche 5 pour can1
MCP_CAN CAN2(portCAN2); // Définit CS broche 10 pour can2

void setup(){
    CAN1.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz
    CAN2.begin(CAN_500KBPS, MCP_8MHz); // init can bus : baudrate = 500k / 8MHz

    CAN1.init_Mask(0,0,0x07FFFF00);
    CAN1.init_Filt(0,0,0x03B70000);
    CAN1.init_Filt(1,0,0x05DA0000);
    CAN1.init_Filt(2,0,0x06460000);
    CAN1.init_Filt(3,0,0x06990000);

    attachInterrupt(digitalPinToInterrupt(interruptCAN1),CAN1_INTERRUPT,FALLING); // Mise en place de l'interruption en cas de données sur le CAN1 

    if(hasEngTemp && hasExtTemp) {
      beginCycleTimeMS=millis();
    }
}

void CAN1_INTERRUPT()
{
        CAN1.readMsgBuf(&len, rxBuf); // Lire les données: len = longueur des données, rxBuf = data des données        
        rxId = CAN1.getCanId(); // Récupère l'identifiant du message
        
        if(rxId == 0x5DA && hasEngTemp ){  // Récupération de la température moteur.
            engTemp=rxBuf[0]; // On stock pour traitement ultérieur !
        }
        
        if(rxId == 0x3B7 && hasExtTemp){ // Récupération de la température exterieur.
            extTemp=rxBuf[0]; // On stock pour traitement ultérieur !
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

void loop(){
    // Gestion du cycle d'affichage : Remise à 0 si on est à 2xdurée du cycle
    if(beginCycleTimeMS + 2*CycleDurationMS < millis() && hasEngTemp && hasExtTemp) {
          beginCycleTimeMS=millis();
    }

    if((beginCycleTimeMS + CycleDurationMS > millis() && hasEngTemp) || (hasEngTemp && !hasExtTemp)) { // Si nous sommes sous la durée du cycle paramétré :
      newTemp = engTemp; // La nouvelle température est celle du moteur.
    } else if((beginCycleTimeMS + CycleDurationMS < millis() && hasExtTemp) || (!hasEngTemp && hasExtTemp)) {  // Si non
      newTemp = extTemp; // La nouvelle température est celle de l'extérieur.                
    }
        
    if(newTemp != temp && !refreshTemp && (hasEngTemp || hasExtTemp) ) { // Nous devons rafraîchir l'affichage de la température !
      refreshTemp=true; // On active le compteur pour attendre 10ms sans couper le fonctionnement du programme
      beginWaitReinitTemp=millis();
      stmp[2]=0xFF;
      CAN2.sendMsgBuf(0x558, 0, 8, stmp); // On envoie la réinitialisation.           
    }

    if(refreshTemp && beginWaitReinitTemp+waitReinitTempMS < millis() && (hasEngTemp || hasExtTemp) ) { // Si nous avons attendus les 10ms
      refreshTemp=false; // On arrête d'attendre !
      temp=newTemp; // On positionne la nouvelle température
      stmp[2]=temp;
      CAN2.sendMsgBuf(0x558, 0, 8, stmp);    // On envoie la température au MediaNav.
    }
}
