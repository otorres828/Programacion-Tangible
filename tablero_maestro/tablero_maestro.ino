// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C) - Archivo Único
// Solicita datos de 2 Arduinos de columna (Esclavos I2C, 5 resistencias cada uno)
// y un tercer Arduino (Esclavo I2C, Bloque de Control de 5 resistencias).
// Consolida los valores de resistencia en un array de 15 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion I2C con cada columna y con los LEDS de cada ficha.
// El movimiento del robot es en direcciones cardinales fijas (arriba, abajo, izquierda, derecha)
// ---------------------------------------------------------------------------------------

#include <Wire.h>                     // Librería para comunicación I2C
#include <SoftwareSerial.h>           // Libreria para comunicacion Bluetooth
#include <Adafruit_PWMServoDriver.h>  // Librería para el PCA9685 - Controlador de Leds de las fichas

#define BOTON_INICIO 2  // Pin del botón (con resistencia pull-up)

SoftwareSerial mySerial(10, 11); // RX, TX para Bluetooth

// Direcciones I2C de los Arduinos esclavos: Columna 1, Columna 2, Bloque de Control
const int SLAVE_ADDRESSES[] = { 0x01, 0x02, 0x03 };

// Arrays para almacenar los datos
float allResistances[15];         // Almacena todas las 15 resistencias leídas
float instruccionesColumnas[10];  // Las primeras 10 instrucciones (de las 2 columnas)
float bloqueControl[5];           // El bloque de control (las últimas 5 posiciones)

// --- DEFINICIONES DE ACCIONES ---
enum ActionType {
  MOVER_ARRIBA = 1,     // Resistencia 190-220 Ohms                   - 220 ohms
  MOVER_ABAJO = 2,      // Resistencia 500-810 Ohms                   - 680ohms 
  MOVER_IZQUIERDA = 3,  // Resistencia 920-1.2k Ohms                  - 1k
  MOVER_DERECHA = 4,    // Resistencia 1.7k-2.5k Ohms                 - 2k
  BLOQUE_CONTROL = 5,   // Resistencia 4k-6 Ohms (no invertible)      - 4.7k
  MELODIA_1 = 6,         // Resistencia 9k-11k Ohms (no invertible)   - 10k
};


// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- ESTADO GLOBAL DEL ROBOT ---
int robotX = 0;  // Posición actual en X (columna de la cuadrícula 0-4)
int robotY = 0;  // Posición actual en Y (fila de la cuadrícula 0-4)

// Crea un objeto PCA9685 , con Direccion 0x40 para comunicacion I2C con el driver Pca9685 que controlara los LEDS de las instrucciones
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Define el número total de LEDs que vas a controlar
const int NUM_LEDS = 15;

// Define los valores de brillo (PWM)
// El PCA9685 tiene una resolución de 12 bits, lo que significa un rango de 0 a 4095.
const int BRILLO_OFF         = 0;    // LED apagado
const int BRILLO_20_PORCIENTO  = 819;  // 20% de 4095
const int BRILLO_100_PORCIENTO = 4095; // LED al máximo brillo

// ---  VARIABLES GLOBALES PARA LA MÁQUINA DE ESTADOS Y BOTÓN ---
enum SystemState { STATE_IDLE, STATE_RUNNING, STATE_PAUSED };
SystemState estadoSistemaActual = STATE_IDLE;
int actualInstruccionIndex = 0; // Índice de la instrucción actual en la secuencia principal (0-9)

// Variables para el manejo del botón (Debouncing y Pulsación Larga)
unsigned long lastButtonStateChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool lastButtonReading = HIGH; //  INPUT_PULLUP, HIGH cuando no está presionado
bool currentButtonReading;
bool longPressTriggered = false; // Flag para saber si una pulsación larga ya fue manejada

const unsigned long BUTTON_DEBOUNCE_DELAY = 50; // ms de delay para debounce
const unsigned long LONG_PRESS_THRESHOLD = 3000; // 3 segundos para pulsación larga

// Variable para el retardo no bloqueante entre acciones
unsigned long lastActionExecutionTime = 0;
const unsigned long DELAY_POR_INSTRUCCION = 3000; // 3 segundo de retraso entre la ejecución de acciones

// --- DECLARACIÓN DE FUNCIONES---
void leerTodasColumnas();
void copiarArrays();
void botonPulsaciones(); // Nueva función para manejar el botón
void doNextInstructionStep(); // Nueva función para ejecutar una sola instrucción
void performAction(ActionType action, int globalIndex);
void ejecutarBlockControl();
void setLedBrightness(int ledIndex, int brightness);
bool validarPosicionXY(int x, int y);
String getAccionText(ActionType action);

void setup() {

  Serial.begin(2400);  // Para comunicación con el Monitor Serial del PC

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();  // Inicia la comunicación I2C como Maestro

  // Inicia el PCA9685 y la comunicación I2C / Establecemos la frecuencia.
  pwm.begin();
  pwm.setPWMFreq(1600);

  for (int q = 0; q < NUM_LEDS; q++) {
    pwm.setPWM(q, 0, BRILLO_20_PORCIENTO); // Canal i, ON_time = 0, OFF_time = BRILLO_OFF
  }

  pinMode(BOTON_INICIO, INPUT_PULLUP); // Configura el pin del botón con resistencia pull-up interna

  Serial.println("----------------------------------");
  Serial.println(" Arduino Central (Maestro I2C) ");
  Serial.println("----------------------------------");
  Serial.println("Iniciando. Esperando lecturas de columnas...");
  Serial.println("Estado actual: IDLE (esperando boton para iniciar)");

  mySerial.begin(9600); // Colocamos el módulo bluetooth en 9600
}

void loop() {

  botonPulsaciones(); // Procesa las entradas del botón en cada ciclo del loop

  switch (estadoSistemaActual) {
    case STATE_IDLE:
      // Cuando está en IDLE, sigue leyendo las fichas para actualizar los LEDs al 20%
      // y prepararse para el inicio.
      leerTodasColumnas();
      copiarArrays();
      // Los LEDs al 20% son manejados dentro de copiarArrays o justo después, como ya lo tienes.
      break;

    case STATE_RUNNING:

      // Si la secuencia está corriendo y ha pasado suficiente tiempo desde la última acción
      if (millis() - lastActionExecutionTime >= DELAY_POR_INSTRUCCION) {

        if (actualInstruccionIndex < 10) { // Si aún quedan instrucciones en la secuencia principal

          doNextInstructionStep(); // Ejecuta la siguiente instrucción
          lastActionExecutionTime = millis(); // Actualiza el tiempo de la última acción ejecutada

        } else {

          // La secuencia principal ha terminado
          Serial.println("Secuencia principal finalizada.");
          estadoSistemaActual = STATE_IDLE; // Vuelve al estado IDLE
          actualInstruccionIndex = 0; // Reinicia el índice para la próxima ejecución

          for (int i = 0; i < NUM_LEDS; i++) {
            setLedBrightness(i, BRILLO_20_PORCIENTO);
          }
        }

      }

      break;

    case STATE_PAUSED:

      // En el estado PAUSED, no se ejecutan nuevas acciones de movimiento.
      // Puedes optar por seguir leyendo las fichas para que los LEDs sigan reflejando las presentes al 20%.
      leerTodasColumnas();
      copiarArrays();
      break;

  }

  // Pequeña pausa para no saturar el serial 
  delay(10);
  
}

// -------------------------------------------------------------------------
// FUNCIONES DE MANEJO DEL BOTÓN
// -------------------------------------------------------------------------

void botonPulsaciones() {
  currentButtonReading = digitalRead(BOTON_INICIO);

  // Detectar cambio de estado para debounce
  if (currentButtonReading != lastButtonReading) {
    lastButtonStateChangeTime = millis();
  }

  if ((millis() - lastButtonStateChangeTime) > BUTTON_DEBOUNCE_DELAY) {
    // El estado del botón es estable
    if (currentButtonReading == LOW && lastButtonReading == HIGH) { // Botón presionado (flanco de bajada)
      buttonPressStartTime = millis();
      longPressTriggered = false; // Reiniciar flag de pulsación larga
    } else if (currentButtonReading == HIGH && lastButtonReading == LOW) { // Botón liberado (flanco de subida)
      if (!longPressTriggered) { // Si no fue una pulsación larga (ya manejada)
        // Esto es una PULSACIÓN CORTA (Iniciar/Pausar/Reanudar)
        if (estadoSistemaActual == STATE_IDLE) {
          Serial.println("Boton: INICIO de secuencia.");
          estadoSistemaActual = STATE_RUNNING;
          actualInstruccionIndex = 0; // Iniciar desde la primera instrucción
          lastActionExecutionTime = millis(); // Preparar el temporizador para la primera acción
        } else if (estadoSistemaActual == STATE_RUNNING) {
          Serial.println("Boton: PAUSA de secuencia.");
          estadoSistemaActual = STATE_PAUSED;
        } else if (estadoSistemaActual == STATE_PAUSED) {
          Serial.println("Boton: REANUDAR secuencia.");
          estadoSistemaActual = STATE_RUNNING;
          lastActionExecutionTime = millis(); // Re-preparar el temporizador para reanudar
        }
      }
    }
  }

  // Verificar pulsación larga mientras el botón está mantenido presionado
  if (currentButtonReading == LOW && !longPressTriggered) {
    if ((millis() - buttonPressStartTime) >= LONG_PRESS_THRESHOLD) {
      Serial.println("Boton: REINICIO COMPLETO (pulsacion larga).");
      longPressTriggered = true; // Marcar como manejado para que no se active de nuevo al soltar
      estadoSistemaActual = STATE_IDLE; // Reiniciar el sistema al estado IDLE
      actualInstruccionIndex = 0; // Reiniciar progreso de la secuencia

      // Opcional: Re-leer todas las fichas y asegurar LEDs a OFF/20%
      leerTodasColumnas();
      copiarArrays();
      // Asegurarse de que todos los LEDs del PCA9685 se apaguen por completo (serán luego 20% por lectura)
      for (int i = 0; i < NUM_LEDS; i++) {
        setLedBrightness(i, BRILLO_20_PORCIENTO);
      }
    }
  }
  lastButtonReading = currentButtonReading; // Guardar el estado actual para la próxima iteración
}

// -------------------------------------------------------------------------
// FUNCIONES DE LECTURA Y PROCESAMIENTO
// -------------------------------------------------------------------------

// Lee los datos de los 3 esclavos I2C y llena el array 'allResistances'
void leerTodasColumnas() {
  int currentGlobalIndex = 0; // Para llevar el control de la posición en allResistances

  // 1. Leer de los dos primeros esclavos (columnas), cada uno con 5 resistencias
  for (int col = 0; col < 2; col++) { // Para las 2 columnas
    int slaveAddress = SLAVE_ADDRESSES[col];
    int floatsToRequest = 5;
    int bytesToRequest = floatsToRequest * sizeof(float); // 5 floats * 4 bytes/float = 20 bytes

    // Serial.print("Solicitando "); Serial.print(floatsToRequest); Serial.print(" floats del esclavo 0x"); Serial.println(slaveAddress, HEX);

    Wire.requestFrom(slaveAddress, bytesToRequest);

    FloatBytes fb;
    for (int i = 0; i < floatsToRequest; i++) {
      if (Wire.available() >= sizeof(float)) {
        for (int j = 0; j < sizeof(float); j++) {
          fb.b[j] = Wire.read();
        }
        allResistances[currentGlobalIndex++] = fb.f;
      } else {
        allResistances[currentGlobalIndex++] = -1.0; // Valor por defecto si no hay datos
      }
    }
    delay(10); // Pequeña pausa para I2C
  }

  // 2. Leer del tercer esclavo (bloque de control), con 5 resistencias
  int blockControlSlaveAddress = SLAVE_ADDRESSES[2]; // El tercer esclavo
  int floatsToRequest = 5;                           // El bloque de control tiene 5 resistencias
  int bytesToRequest = floatsToRequest * sizeof(float);

  Wire.requestFrom(blockControlSlaveAddress, bytesToRequest);

  FloatBytes fb;
  for (int i = 0; i < floatsToRequest; i++) {
    if (Wire.available() >= sizeof(float)) {
      for (int j = 0; j < sizeof(float); j++) {
        fb.b[j] = Wire.read();
      }
      allResistances[currentGlobalIndex++] = fb.f;
    } else {
      allResistances[currentGlobalIndex++] = -1.0; // Valor por defecto si no hay datos
    }
  }
  delay(10); // Pequeña pausa para I2C

  // Actualizar LEDs al 20% para las fichas presentes cuando se lee en IDLE/PAUSED
  // Esta lógica ya está en el loop(), pero la ponemos aquí para ser explícitos
  // que los LEDs se actualizan cada vez que se leen las resistencias.
  for (int i = 0; i < NUM_LEDS; i++) {
    if (allResistances[i] > 0) {
      setLedBrightness(i, BRILLO_20_PORCIENTO);
    } else {
      setLedBrightness(i, BRILLO_OFF);
    }
  }
}

// Copiar los valores después de leer todas las columnas a sus arrays específicos
void copiarArrays() {
  
  // Copia las primeras 10 posiciones a instruccionesColumnas (las 2 columnas de 5 resistencias)
  for (int i = 0; i < 10; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  // Copia las últimas 5 posiciones a bloqueControl (el bloque de control)
  for (int i = 0; i < 5; i++) {
    bloqueControl[i] = allResistances[10 + i]; // Empieza en el índice 10
  }
}

// -------------------------------------------------------------------------
// FUNCIONES DE EJECUCIÓN DE SECUENCIA
// -------------------------------------------------------------------------

// Esta función ejecuta UNA SOLA instrucción de la secuencia principal (la actual de actualInstruccionIndex)
void doNextInstructionStep() {

  float instruccionActual = instruccionesColumnas[actualInstruccionIndex];
  ActionType actualAction = (ActionType)instruccionActual; // Castear a enum

  // Solo procesar si la instrucción es válida (resistencia positiva)
  if (instruccionActual > 0) {

      if (actualAction == BLOQUE_CONTROL) {

        Serial.print("Instruccion "); Serial.print(actualInstruccionIndex + 1);
        ejecutarBlockControl(); // Esta función es bloqueante.
        
      } else {
        // Para todas las demás acciones (movimiento, melodía)
        performAction(actualAction, actualInstruccionIndex);
      
      }
    

  } 
  // El incremento de actualInstruccionIndex lo maneja el loop()
  // para que pueda pausar y reanudar.
  actualInstruccionIndex++; // Mueve al siguiente índice de instrucción
}

// Ejecuta una acción específica basada en el ActionType.
void performAction(ActionType action, int globalIndex) {

  Serial.print("Instruccion ");
  Serial.print(globalIndex + 1);
  Serial.print(": ");
  Serial.print(getAccionText(action));

  int nextX = robotX;
  int nextY = robotY;

  // Enciende el LED de la ficha al 100% durante la ejecución
  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    setLedBrightness(globalIndex, BRILLO_100_PORCIENTO);
  }

  switch (action) {
    case MOVER_ARRIBA:
      nextY++;
      break;
    case MOVER_ABAJO:
      nextY--;
      break;
    case MOVER_IZQUIERDA:
      nextX--;
      break;
    case MOVER_DERECHA:
      nextX++;
      break;
    default:
      break;
  }

  // Actualiza la posición del robot solo si fue una acción de movimiento válida
  if (action == MOVER_ARRIBA || action == MOVER_ABAJO || action == MOVER_IZQUIERDA || action == MOVER_DERECHA) {
    if (validarPosicionXY(nextX, nextY)) {
      robotX = nextX;
      robotY = nextY;
    }
  }

  // Envía la acción por Bluetooth al Arduino de motores
  mySerial.println(action);
  Serial.println(); // Salto de línea para mejor legibilidad en el monitor serial

  // Baja el brillo del LED de la ficha de nuevo al 20% después de ejecutar la acción
  // (si la ficha sigue presente)
  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    if (allResistances[globalIndex] > 0) { // Verifica si la ficha aún es considerada válida
      setLedBrightness(globalIndex, BRILLO_20_PORCIENTO);
    } else {
      setLedBrightness(globalIndex, BRILLO_OFF); // Si la ficha fue quitada, apagar completamente
    }
  }
}

// Ejecuta la lógica del bloque de control, procesando las 5 resistencias de bloqueControl.
// NOTA: Esta función es BLOQUEANTE debido al bucle 'for' y al 'delay'.
void ejecutarBlockControl() {

  for (int i = 0; i < 5; i++) { // Las 5 resistencias del bloque de control
    float controlRawInstruction = bloqueControl[i];
    ActionType controlAction = (ActionType)controlRawInstruction;

    if (controlRawInstruction > 0) { // Solo procesar resistencias válidas
      if (controlAction == BLOQUE_CONTROL) {

        // No se permite una llamada recursiva para evitar bucles infinitos

      } else {

          // Para todas las demás acciones, ejecutar normalmente. globalIndex ajustado.
          performAction(controlAction, i + 10);
        
      }
    } 
    
    delay(DELAY_POR_INSTRUCCION); // Pausa entre instrucciones del bloque de control
  }

}

// Verifica si una posición (x, y) está dentro de los límites de la cuadrícula.
bool validarPosicionXY(int x, int y) {
  int GRID_SIZE = 5;
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

// Devuelve el tipo de acción en formato de texto para el monitor serial
String getAccionText(ActionType action) {
  switch (action) {
    case MOVER_ARRIBA:    return "Avanzar";
    case MOVER_ABAJO:     return "Retroceder";
    case MOVER_IZQUIERDA: return "Izquierda";
    case MOVER_DERECHA:   return "Derecha";
    case BLOQUE_CONTROL:  return "Bloque Control";
    case MELODIA_1:       return "Melodia";
    default:              return "Ninguna instruccion / Error";
  }
}

//  controla el brillo de un LED en el PCA9685
void setLedBrightness(int ledIndex, int brightness) {
  if (ledIndex >= 0 && ledIndex <= NUM_LEDS) {
    pwm.setPWM(ledIndex, 0, brightness);
  }
}