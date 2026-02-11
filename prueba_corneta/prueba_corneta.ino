#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

SoftwareSerial mp3Serial(10, 11); 
DFRobotDFPlayerMini mp3;

const int TRACK_FIRST = 1;
const int TRACK_LAST  = 4;
const unsigned long DELAY_BETWEEN_TRACKS_MS = 5000;
const int VOLUME = 25;

int currentTrack = TRACK_FIRST;
unsigned long lastPlayMillis = 0;

void setup() {
  Serial.begin(9600);
  mp3Serial.begin(9600);


  delay(1000);

  if (!mp3.begin(mp3Serial)) {
    Serial.println("DFPlayer no inicializado");
    while (true);
  }

  Serial.println("DFPlayer inicializado");
  mp3.volume(VOLUME);

  mp3.playMp3Folder(currentTrack);
  lastPlayMillis = millis();
}

void loop() {
  if (millis() - lastPlayMillis >= DELAY_BETWEEN_TRACKS_MS) {
    currentTrack++;

    if (currentTrack > TRACK_LAST) {
      currentTrack = TRACK_FIRST;
    }
    Serial.print("Track ");
    Serial.println(currentTrack);

    mp3.playMp3Folder(currentTrack);
    lastPlayMillis = millis();
  }
}
