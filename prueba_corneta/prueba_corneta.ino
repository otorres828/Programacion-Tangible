#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Definimos los pines 12 (TX del Nano -> RX del DFPlayer) 
// y 13 (RX del Nano <- TX del DFPlayer)
SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(115200); // Para monitoreo en PC

  Serial.println(F("Iniciando DFPlayer..."));

  // Inicialización del módulo
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Error: Revisa conexiones o tarjeta SD."));
    while(true);
  }
  
  Serial.println(F("DFPlayer Online."));
  myDFPlayer.volume(20);  // Volumen de 0 a 30
  myDFPlayer.play(1);     // Reproduce el primer archivo (0001.mp3)
}

void loop() {
  // El loop puede estar vacío o añadir controles aquí
}
