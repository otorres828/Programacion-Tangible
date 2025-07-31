// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Esclavo (CNC Control de Motores) - cnc_esclavo.ino
// Recibe instrucciones del Arduino Maestro vía Bluetooth y controla 2 motores paso a paso.
// Incluye proceso de homing/calibración con finales de carrera y reproducción de audio.
// ---------------------------------------------------------------------------------------

#include <SoftwareSerial.h> // Librería para comunicación Bluetooth
// #include <Stepper.h>        // Librería para motores paso a paso (ELIMINADA: no es ideal para 28BYJ-48 con ULN2003)

// --- Pines del módulo Bluetooth HC-05 ---
SoftwareSerial bluetoothSerial(10, 11); // RX, TX para Bluetooth (conectado a TX, RX del Maestro)

// --- Definiciones para los motores 28BYJ-48 con ULN2003 ---
// Los pines se conectan a IN1, IN2, IN3, IN4 del driver ULN2003
// Motor Eje X
#define IN1_X 2
#define IN2_X 3
#define IN3_X 4
#define IN4_X 5

// Motor Eje Y
#define IN1_Y 6
#define IN2_Y 7
#define IN3_Y 8
#define IN4_Y 9

// Pasos por revolución para el 28BYJ-48 (con factor de reducción de 1/64)
// Pasos reales = 32 (pasos por revolución interna) * 64 (relación de engranajes) = 2048
const int STEPS_PER_REVOLUTION = 2048;

// --- Definición de pines para Finales de Carrera ---
#define ENDSTOP_X_PIN A0 // Pin para el final de carrera del Eje X
#define ENDSTOP_Y_PIN A1 // Pin para el final de carrera del Eje Y

// --- Definición para el Módulo de Audio (ej. DFPlayer Mini) ---
// Asumo un módulo de audio que se controla por Serial
// Si usas otro, adapta la librería y las funciones de control.
#include <DFMiniMp3.h> // Librería de ejemplo para DFPlayer Mini

// Pines para SoftSerial del DFPlayer Mini (si se usa SoftwareSerial)
SoftwareSerial myMp3Serial(12, 13); // RX, TX para DFPlayer (conectado a TX, RX del DFPlayer)

// Clase para el DFPlayer Mini
class Mp3Notify
{
public:
  static void OnError(uint8_t errorCode)
  {
    // Manejo de errores del reproductor MP3
    Serial.print("Error MP3: ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished(uint8_t track)
  {
    Serial.print("Reproduccion MP3 terminada: ");
    Serial.println(track);
  }
  static void OnCardOnline(uint8_t r) { Serial.println("SD Card Online"); }
  static void OnCardInserted(uint8_t r) { Serial.println("SD Card Inserted"); }
  static void OnCardRemoved(uint8_t r) { Serial.println("SD Card Removed"); }
};

DFMiniMp3<SoftwareSerial, Mp3Notify> mp3;


// --- Velocidad de los motores ---
const int MOTOR_SPEED_RPM = 10; // RPMs (ajusta según tus necesidades)
const int HOMING_SPEED_RPM = 5; // RPMs para el homing (más lento para mayor seguridad)

// --- Definiciones de Acciones (COPIADAS DEL MAESTRO) ---
enum ActionType {
  MOVER_ARRIBA = 1,
  MOVER_ABAJO = 2,
  MOVER_IZQUIERDA = 3,
  MOVER_DERECHA = 4,
  BLOQUE_CONTROL = 5, // Aunque el esclavo no lo ejecuta, lo recibe
  MELODIA_1 = 7,
};

// --- Array de secuencias para el motor 28BYJ-48 (paso completo) ---
// Esto es más apropiado para controlar el ULN2003 directamente que la librería Stepper.h.
// Se asume que el cableado es IN1, IN2, IN3, IN4 del ULN2003 al Arduino.
// La secuencia es: 1000, 0100, 0010, 0001 (para pasos completos)
const int steps[][4] = {
  {1, 0, 0, 0}, // Paso 1 (IN1 ON)
  {0, 1, 0, 0}, // Paso 2 (IN2 ON)
  {0, 0, 1, 0}, // Paso 3 (IN3 ON)
  {0, 0, 0, 1}  // Paso 4 (IN4 ON)
};
const int NUM_STEPS_SEQUENCE = 4; // Número de pasos en la secuencia anterior

// --- Funciones de control de motores 28BYJ-48 ---
void setMotorPins(int in1, int in2, int in3, int in4, int stepIndex) {
  digitalWrite(in1, steps[stepIndex][0]);
  digitalWrite(in2, steps[stepIndex][1]);
  digitalWrite(in3, steps[stepIndex][2]);
  digitalWrite(in4, steps[stepIndex][3]);
}

/**
 * @brief Mueve un motor paso a paso 28BYJ-48.
 * @param in1, in2, in3, in4 Pines de control del driver ULN2003.
 * @param stepsToMove Número de pasos a mover. Positivo para un sentido, negativo para el otro.
 * @param motorSpeedRpm Velocidad deseada en RPM.
 */
void moveMotor(int in1, int in2, int in3, int in4, int stepsToMove, int motorSpeedRpm) {
  if (motorSpeedRpm <= 0) motorSpeedRpm = 1; // Evitar división por cero o velocidades inválidas
  unsigned long delayBetweenSteps = 60000000L / (STEPS_PER_REVOLUTION * motorSpeedSpeedRpm); // microsegundos
  int currentStep = 0; // Para llevar la cuenta del paso actual en la secuencia de 4
  
  // Determinar la dirección y el ajuste inicial de 'currentStep'
  int direction = (stepsToMove > 0) ? 1 : -1;
  int initialStepForDirection = (direction == 1) ? 0 : (NUM_STEPS_SEQUENCE - 1); // Empezar en el último paso para retroceder bien

  for (int i = 0; i < abs(stepsToMove); i++) {
    setMotorPins(in1, in2, in3, in4, currentStep);
    delayMicroseconds(delayBetweenSteps);
    
    currentStep = (currentStep + direction + NUM_STEPS_SEQUENCE) % NUM_STEPS_SEQUENCE;
  }
  // Apagar todas las bobinas al final del movimiento para ahorrar energía y evitar calentamiento
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


// --- Declaración de funciones ---
void doHoming();
void playInstructionAudio(ActionType action);
void executeAction(ActionType action);

void setup() {
  Serial.begin(9600);       // Para depuración con el Monitor Serial
  bluetoothSerial.begin(9600); // Iniciar comunicación Bluetooth con el Maestro

  // Configurar pines de motores
  pinMode(IN1_X, OUTPUT);
  pinMode(IN2_X, OUTPUT);
  pinMode(IN3_X, OUTPUT);
  pinMode(IN4_X, OUTPUT);

  pinMode(IN1_Y, OUTPUT);
  pinMode(IN2_Y, OUTPUT);
  pinMode(IN3_Y, OUTPUT);
  pinMode(IN4_Y, OUTPUT);

  // Configurar pines de finales de carrera
  pinMode(ENDSTOP_X_PIN, INPUT_PULLUP); // Usar pull-up interno
  pinMode(ENDSTOP_Y_PIN, INPUT_PULLUP); // Usar pull-up interno

  // Iniciar módulo MP3
  myMp3Serial.begin(9600);
  mp3.begin();
  delay(100); // Dar tiempo al módulo MP3 para iniciar
  mp3.setVolume(20); // Ajustar volumen (0-30)

  Serial.println("CNC Esclavo Iniciado.");
  Serial.println("Realizando proceso de homing...");
  doHoming(); // Ejecutar homing al inicio
  Serial.println("Homing completado. Esperando instrucciones...");
}

void loop() {
  // Mantener el módulo MP3 actualizado (necesario para la librería DFMiniMp3)
  mp3.loop();

  if (bluetoothSerial.available()) {
    int receivedAction = bluetoothSerial.parseInt(); // Leer el entero enviado por el Maestro

    // Limpiar el buffer serial por si quedan caracteres (ej. '\n')
    while (bluetoothSerial.available()) {
      bluetoothSerial.read();
    }

    if (receivedAction > 0) { // Asegurarse de que sea una acción válida
      ActionType action = static_cast<ActionType>(receivedAction);
      Serial.print("Recibido del Maestro: ");
      Serial.println(action);

      executeAction(action); // Ejecutar la acción recibida
    }
  }
}

// --- FUNCIONES ---

// Proceso de Homing: Mueve los ejes hasta que toquen sus respectivos finales de carrera.
void doHoming() {
  // Homing para el Eje X (mover hacia la "izquierda" hasta que toque el final de carrera)
  // Asumiendo que el final de carrera X está en el límite izquierdo (X=0)
  Serial.println("Homing Eje X...");
  playInstructionAudio(MOVER_IZQUIERDA); // Indica que se mueve a la izquierda para homing
  while (digitalRead(ENDSTOP_X_PIN) == HIGH) { // HIGH significa que el switch no ha sido presionado (pull-up)
    // Mover un pequeño paso en dirección negativa (hacia el final de carrera)
    moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, -1, HOMING_SPEED_RPM);
  }
  Serial.println("Eje X en posicion HOME.");
  // Un pequeño movimiento en dirección opuesta para liberar el switch, si es necesario para evitar rebotes
  moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, 10, HOMING_SPEED_RPM); // Mueve 10 pasos en dirección positiva (lejos del switch)

  // Homing para el Eje Y (mover hacia "abajo" hasta que toque el final de carrera)
  // Asumiendo que el final de carrera Y está en el límite inferior (Y=0)
  Serial.println("Homing Eje Y...");
  playInstructionAudio(MOVER_ABAJO); // Indica que se mueve hacia abajo para homing
  while (digitalRead(ENDSTOP_Y_PIN) == HIGH) { // HIGH significa que el switch no ha sido presionado (pull-up)
    // Mover un pequeño paso en dirección negativa (hacia el final de carrera)
    moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, -1, HOMING_SPEED_RPM);
  }
  Serial.println("Eje Y en posicion HOME.");
  // Un pequeño movimiento en dirección opuesta para liberar el switch
  moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, 10, HOMING_SPEED_RPM); // Mueve 10 pasos en dirección positiva (lejos del switch)

  Serial.println("Homing Completo.");
  // Si deseas una confirmación de audio para el homing completo, puedes añadirla aquí
  // mp3.play(6); // Asumiendo que el archivo de audio 6 es "Homing Completo"
}

// Reproduce el archivo de audio correspondiente a la instrucción
void playInstructionAudio(ActionType action) {
  int trackNumber = 0; // Número de pista en la tarjeta SD
  switch (action) {
    case MOVER_ARRIBA:    trackNumber = 1; break; // Asume que track1.mp3 dice "Mover Arriba"
    case MOVER_ABAJO:     trackNumber = 2; break; // Asume que track2.mp3 dice "Mover Abajo"
    case MOVER_IZQUIERDA: trackNumber = 3; break; // Asume que track3.mp3 dice "Mover Izquierda"
    case MOVER_DERECHA:   trackNumber = 4; break; // Asume que track4.mp3 dice "Mover Derecha"
    case MELODIA_1:       trackNumber = 5; break; // Asume que track5.mp3 es la melodía
    // BLOQUE_CONTROL y NEGACION no tienen audio asociado en el esclavo
    default:              return; // No reproducir nada para acciones no reconocidas o no audibles
  }
  // Detener cualquier reproducción actual antes de iniciar una nueva
  mp3.stop(); 
  mp3.play(trackNumber);
  // Pequeño retardo para permitir que el audio comience a reproducirse.
  // Podrías necesitar un delay más largo o un manejo no bloqueante si el audio es muy largo.
  delay(100); 
}

// Ejecuta la acción de movimiento o reproducción de audio
void executeAction(ActionType action) {
  // Primero, reproduce el audio de la instrucción
  playInstructionAudio(action);

  // Luego, ejecuta el movimiento o la melodía
  // NOTA IMPORTANTE: Para un sistema H-Bot o CoreXY, MOVER_ARRIBA/ABAJO y MOVER_IZQUIERDA/DERECHA
  // requieren que *ambos* motores (X e Y) se muevan simultáneamente en una combinación específica.
  // La implementación actual asume movimientos independientes de X e Y.
  // Para un H-Bot/CoreXY real, necesitarías una función de movimiento que coordine ambos motores.
  // Por ejemplo, para MOVER_ARRIBA, M1 se mueve +N pasos y M2 se mueve +N pasos.
  // Para MOVER_IZQUIERDA, M1 se mueve -N pasos y M2 se mueve +N pasos.
  // Esto dependerá de cómo estén configuradas tus correas y el sistema mecánico.
  // A continuación, se muestra una implementación *simplificada* que asume movimientos de un solo eje,
  // la cual DEBERÍA SER REEMPLAZADA POR LA LÓGICA COREXY/H-BOT COMPLETA.

  // Los valores de pasos (ej. 2048 / 5) son ejemplos. DEBES CALIBRARLOS según la distancia que
  // tu muñeco debe moverse por "casilla" en el tablero.

  switch (action) {
    case MOVER_ARRIBA:
      Serial.println("Moviendo ARRIBA...");
      // Para un H-Bot/CoreXY, esto requeriría movimiento coordinado de ambos motores.
      // Ejemplo simplificado (¡ajustar según tu cinemática real!):
      moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, 2048 / 5, MOTOR_SPEED_RPM); // Movimiento en X
      moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, 2048 / 5, MOTOR_SPEED_RPM); // Movimiento en Y
      break;
    case MOVER_ABAJO:
      Serial.println("Moviendo ABAJO...");
      // Ejemplo simplificado (¡ajustar según tu cinemática real!):
      moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, -2048 / 5, MOTOR_SPEED_RPM); // Movimiento en X
      moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, -2048 / 5, MOTOR_SPEED_RPM); // Movimiento en Y
      break;
    case MOVER_IZQUIERDA:
      Serial.println("Moviendo IZQUIERDA...");
      // Ejemplo simplificado (¡ajustar según tu cinemática real!):
      moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, -2048 / 5, MOTOR_SPEED_RPM); // Movimiento en X
      moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, 2048 / 5, MOTOR_SPEED_RPM); // Movimiento en Y
      break;
    case MOVER_DERECHA:
      Serial.println("Moviendo DERECHA...");
      // Ejemplo simplificado (¡ajustar según tu cinemática real!):
      moveMotor(IN1_X, IN2_X, IN3_X, IN4_X, 2048 / 5, MOTOR_SPEED_RPM); // Movimiento en X
      moveMotor(IN1_Y, IN2_Y, IN3_Y, IN4_Y, -2048 / 5, MOTOR_SPEED_RPM); // Movimiento en Y
      break;
    case MELODIA_1:
      Serial.println("Reproduciendo Melodia 1...");
      // playInstructionAudio(MELODIA_1) ya fue llamado arriba, lo cual es suficiente.
      // Si la melodía es muy larga, puedes añadir un delay aquí para que el robot "espere"
      // o implementar un manejo de audio no bloqueante si necesitas ejecutar otras cosas.
      break;
    default:
      Serial.println("Instruccion desconocida.");
      break;
  }
  // Añade un pequeño delay después de cada acción para dar tiempo al robot a completar su movimiento
  delay(1000); // Ajusta este delay según la duración de los movimientos
}