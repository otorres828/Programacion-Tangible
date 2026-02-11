// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Esclavo (CNC Control de Motores) - cnc_esclavo.ino
// Recibe instrucciones del Arduino Maestro vía Bluetooth y controla 2 motores paso a paso.
// Adaptado para un sistema H-BOT.
// Incluye proceso de homing/calibración con finales de carrera (sensor de efecto hall) y reproducción de audio.
// ---------------------------------------------------------------------------------------

#include <SoftwareSerial.h> // Librería para comunicación Bluetooth
#include <DFRobotDFPlayerMini.h> // Librería para DFPlayer Mini (la oficial de DFRobot)

// --- Pines del módulo Bluetooth HC-05 ---
SoftwareSerial bluetoothSerial(10, 11); // RX, TX para Bluetooth (conectado a TX, RX del CNC)

// --- Opciones de detección de conexión Bluetooth ---
#define BT_STATE_PIN A2
const int ledVerde = A3;  // LED de Conectado
const int ledRojo = A4;   // LED de Desconectado
bool conectado = false;

// Variables de estado para detección
unsigned long lastBtReceiveMillis = 0;
bool btConnected = false; // estado lógico (basado en STATE pin o en actividad)
bool homingDone = false; // Bandera para evitar ejecutar homing más de una vez por conexión

// Variables para debounce del pin STATE
int btRawLast = LOW;
int btStableState = LOW;
unsigned long btLastDebounceTime = 0;
const unsigned long BT_STATE_DEBOUNCE_MS = 200; // tiempo de debounce para pin STATE (ms)

// --- Pines para SoftSerial del DFPlayer Mini ---
// SoftwareSerial myMp3Serial(12, 13); // RX, TX para DFPlayer (conectado a TX, RX del DFPlayer)

// Objeto del DFPlayer Mini (DFRobotDFPlayerMini)
DFRobotDFPlayerMini mp3;
bool mp3Ok = false; // indica si el DFPlayer fue inicializado correctamente

// --- Definiciones para los motores 28BYJ-48 con ULN2003 ---
// Motor 1 (izquierda)
#define IN1_M1 2
#define IN2_M1 3
#define IN3_M1 4
#define IN4_M1 5
// Motor 2 (derecha)
#define IN1_M2 6
#define IN2_M2 7
#define IN3_M2 8
#define IN4_M2 9

// Pasos por revolución para el 28BYJ-48 (con factor de reducción de 1/64)
const int STEPS_PER_REVOLUTION = 2048;

// --- Definición de pines para Finales de Carrera ---
#define ENDSTOP_X_PIN A0 // Pin para el final de carrera del Eje X
#define ENDSTOP_Y_PIN A1 // Pin para el final de carrera del Eje Y

// --- Velocidad de los motores ---
const int MOTOR_SPEED_RPM = 12; // RPMs (ajusta según tus necesidades)
const int HOMING_SPEED_RPM = 5; // RPMs para el homing (más lento para mayor seguridad)

// --- Definiciones de Acciones ---
enum ActionType {
  MOVER_ARRIBA = 1,
  MOVER_ABAJO = 2,
  MOVER_IZQUIERDA = 3,
  MOVER_DERECHA = 4,
  MELODIA_1 = 6,
  DO_HOMMING = 7,
};

// --- Array de secuencias para el motor 28BYJ-48 (paso completo) ---
const int steps[][4] = {
  {1, 0, 0, 0}, // Paso 1 (IN1 ON)
  {0, 1, 0, 0}, // Paso 2 (IN2 ON)
  {0, 0, 1, 0}, // Paso 3 (IN3 ON)
  {0, 0, 0, 1}  // Paso 4 (IN4 ON)
};

const int NUM_STEPS_SEQUENCE = 4;

// --- CABECERAS DE FUNCIONES ---
void setMotorPins(int in1, int in2, int in3, int in4, int stepIndex);
void moveHbot(int stepsX, int stepsY, int motorSpeedRpm);
void doHoming();
void playInstructionAudio(ActionType action);
void executeAction(ActionType action);
int manejoLedBT(int stableState);

// --- SETUP Y LOOP ---
void setup() {

  // iniciar ambos SoftSerial
  Serial.begin(9600); // Inicializar puerto Serial para Monitor Serial
  bluetoothSerial.begin(9600);
  Serial.println("CNC Esclavo Iniciado.");

  // Inicializar pin STATE
  pinMode(BT_STATE_PIN, INPUT);
  int s = digitalRead(BT_STATE_PIN);
  btConnected = (s == HIGH);

  //leds que indican conexion con bluetooth
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);

  // Inicializar debounce/estados
  btRawLast = s;
  btStableState = s;
  btLastDebounceTime = millis();
  Serial.print("BT_STATE_PIN inicial: "); Serial.println(s);
  // Inicializar LEDs según estado actual
  manejoLedBT(s);

  // Configurar pines de motores
  pinMode(IN1_M1, OUTPUT);
  pinMode(IN2_M1, OUTPUT);
  pinMode(IN3_M1, OUTPUT);
  pinMode(IN4_M1, OUTPUT);
  pinMode(IN1_M2, OUTPUT);
  pinMode(IN2_M2, OUTPUT);
  pinMode(IN3_M2, OUTPUT);
  pinMode(IN4_M2, OUTPUT);

  // Configurar pines de finales de carrera
  pinMode(ENDSTOP_X_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_PIN, INPUT_PULLUP);


  Serial.println("FINAL DE LA CONFIGURACION INICIAL...");
}

void loop() {


  // Leer pin STATE con debounce
  int raw = digitalRead(BT_STATE_PIN);
  if (raw != btRawLast) {
      btLastDebounceTime = millis();
      btRawLast = raw;
  }

  if (millis() - btLastDebounceTime > BT_STATE_DEBOUNCE_MS) {


      // El estado se considera estable
      if (raw != btStableState) {
        Serial.println("imprimiendo loop...");
        btStableState = raw;
        // Actualizar LEDs al cambiar el estado estable
        manejoLedBT(btStableState);
        if (btStableState == HIGH && !btConnected) {
          btConnected = true;
          Serial.println("Bluetooth: conectado (STATE pin). Ejecutando homing...");
          if (!homingDone) {
            doHoming(); // Homing al conectar por STATE pin
            homingDone = true;
          }
        } else if (btStableState == LOW && btConnected) {
          btConnected = false;
          homingDone = false; // permitir homing en la próxima conexión
          Serial.println("Bluetooth: desconectado (STATE pin).");
        }
      }
  }
  
  if (btConnected) {
    if (bluetoothSerial.available()) {
      int receivedAction = bluetoothSerial.parseInt();
      while (bluetoothSerial.available()) {
        bluetoothSerial.read();
      }

      Serial.print("Recibido del Maestro: ");
      Serial.println(receivedAction);

      if (receivedAction > 0) {
        ActionType action = static_cast<ActionType>(receivedAction);
        executeAction(action);
      }
    }
  } else {
    // Si no está conectado, vaciar cualquier dato pendiente y no ejecutar instrucciones
    while (bluetoothSerial.available()) {
      bluetoothSerial.read();
    }
  }
}

// ------------------------------------------------------------------------
// FUNCION MANEJO DE LED BLUETOOTH
// ------------------------------------------------------------------------
int manejoLedBT(int stableState){
  bool prev = conectado;
  // Set LEDs deterministically based on debounced state
  if (stableState == HIGH) {
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledRojo, LOW);
    conectado = true;
  } else {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    conectado = false;
  }

  // Log only on transitions
  if (conectado && !prev) {
    Serial.println(">>> ESTADO: CONECTADO <<<");
  } else if (!conectado && prev) {
    Serial.println(">>> ESTADO: DESCONECTADO <<<");
  }

  return conectado ? 1 : 0;
}

void setMotorPins(int in1, int in2, int in3, int in4, int stepIndex) {
  digitalWrite(in1, steps[stepIndex][0]);
  digitalWrite(in2, steps[stepIndex][1]);
  digitalWrite(in3, steps[stepIndex][2]);
  digitalWrite(in4, steps[stepIndex][3]);
}

void moveHbot(int stepsX, int stepsY, int motorSpeedRpm) {
  if (motorSpeedRpm <= 0) motorSpeedRpm = 1;
  unsigned long denom = (unsigned long)STEPS_PER_REVOLUTION * (unsigned long)motorSpeedRpm;
  unsigned long delayBetweenSteps = 60000000UL / denom; // microsegundos
  
  // Calcular los pasos para cada motor en base a la cinemática del H-bot
  // stepsX > 0 mueve DERECHA, stepsX < 0 mueve IZQUIERDA
  // stepsY > 0 mueve ARRIBA, stepsY < 0 mueve ABAJO
  int stepsM1 = stepsY - stepsX;
  int stepsM2 = stepsY + stepsX;
  
  int maxSteps = max(abs(stepsM1), abs(stepsM2));

  int currentStepM1 = 0;
  int currentStepM2 = 0;
  int directionM1 = (stepsM1 > 0) ? 1 : -1;
  int directionM2 = (stepsM2 > 0) ? 1 : -1;

  for (int i = 0; i < maxSteps; i++) {
    if (i < abs(stepsM1)) {
      setMotorPins(IN1_M1, IN2_M1, IN3_M1, IN4_M1, currentStepM1);
      currentStepM1 = (currentStepM1 + directionM1 + NUM_STEPS_SEQUENCE) % NUM_STEPS_SEQUENCE;
    }
    if (i < abs(stepsM2)) {
      setMotorPins(IN1_M2, IN2_M2, IN3_M2, IN4_M2, currentStepM2);
      currentStepM2 = (currentStepM2 + directionM2 + NUM_STEPS_SEQUENCE) % NUM_STEPS_SEQUENCE;
    }
    delayMicroseconds(delayBetweenSteps);
  }
  
  // Apagar todas las bobinas
  digitalWrite(IN1_M1, LOW); digitalWrite(IN2_M1, LOW); digitalWrite(IN3_M1, LOW); digitalWrite(IN4_M1, LOW);
  digitalWrite(IN1_M2, LOW); digitalWrite(IN2_M2, LOW); digitalWrite(IN3_M2, LOW); digitalWrite(IN4_M2, LOW);
}

void doHoming() {
  Serial.println("Iniciando Homing (H-Bot)...");
  const int steps = 50;
  // const int steps = STEPS_PER_REVOLUTION / 2;

  // Mover el Eje X (ambos motores en la misma dirección)
  Serial.println("Homing Eje X (moviendo izquierda)...");
  // playInstructionAudio(MOVER_IZQUIERDA);
  while (digitalRead(ENDSTOP_X_PIN) == HIGH) {
    moveHbot(-steps, 0, HOMING_SPEED_RPM);
  }

  delay(1000);
  Serial.println("Final de carrera X alcanzado.");
  moveHbot(steps - 20, 0, HOMING_SPEED_RPM);
  delay(1500);

  // Mover el Eje Y (motores en direcciones opuestas)
  Serial.println("Homing Eje Y (moviendo abajo)...");
  // playInstructionAudio(MOVER_ABAJO);
  while (digitalRead(ENDSTOP_Y_PIN) == HIGH) {
    moveHbot(steps, -1, HOMING_SPEED_RPM);
  }

  delay(1000);
  Serial.println("Final de carrera Y alcanzado.");
  moveHbot(0, steps - 20, HOMING_SPEED_RPM);

  Serial.println("Homing Completo.");
}

void playInstructionAudio(ActionType action) {
  int trackNumber = 0;
  switch (action) {
    case MOVER_ARRIBA:    trackNumber = 1; break;
    case MOVER_ABAJO:     trackNumber = 2; break;
    case MOVER_IZQUIERDA: trackNumber = 3; break;
    case MOVER_DERECHA:   trackNumber = 4; break;
    case MELODIA_1:       trackNumber = 5; break;
    default:              return;
  }
  mp3.play(trackNumber);
  // Pequeño retardo para permitir comenzar la reproducción
  delay(100);
}

void executeAction(ActionType action) {
  
  // playInstructionAudio(action);

  const int steps = STEPS_PER_REVOLUTION / 2;

  switch (action) {
    case MOVER_ARRIBA:
      Serial.println("Moviendo ARRIBA...");
      moveHbot(-steps, 0, MOTOR_SPEED_RPM);
      break;
    case MOVER_ABAJO:
      Serial.println("Moviendo ABAJO...");
      moveHbot(steps, 0, MOTOR_SPEED_RPM);
      
      break;
    case MOVER_IZQUIERDA:
      Serial.println("Moviendo IZQUIERDA...");
      moveHbot(0, steps, MOTOR_SPEED_RPM);
      break;
    case MOVER_DERECHA:
      Serial.println("Moviendo DERECHA...");
      moveHbot(0, -steps, MOTOR_SPEED_RPM);
      break;
    case MELODIA_1:
      Serial.println("Reproduciendo Melodia 1...");
      break;
    case DO_HOMMING:
      doHoming();
      break;
    default:
      Serial.println("Instruccion desconocida.");
      break;
  }
  delay(1000);
}