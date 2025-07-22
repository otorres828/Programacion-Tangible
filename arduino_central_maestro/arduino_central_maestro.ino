// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C) - Archivo Único
// Solicita datos de 2 Arduinos de columna (Esclavos I2C, 5 resistencias cada uno)
// y un tercer Arduino (Esclavo I2C, Bloque de Control de 4 resistencias).
// Consolida los valores de resistencia en un array de 14 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion I2C con cada columna.
// El movimiento del robot ahora es en direcciones cardinales fijas (arriba, abajo, izquierda, derecha)
// sin depender de una orientación interna del robot.
// ---------------------------------------------------------------------------------------

#include <Wire.h>                      // Librería para comunicación I2C
#include <SoftwareSerial.h>           // Libreria para comunicacion Bluetooth
#include <Adafruit_PWMServoDriver.h> // Librería para el PCA9685

// --- DEFINICIONES DE PINES Y CONSTANTES GLOBALES ---

#define BOTON_INICIO 2            // Pin del botón (con resistencia pull-up)

SoftwareSerial mySerial(10, 11); // RX, TX

// Direcciones I2C de los Arduinos esclavos: Columna 1, Columna 2, Bloque de Control
const int SLAVE_ADDRESSES[] = {0x01, 0x02, 0x03};

// Arrays para almacenar los datos
float allResistances[15];         // Almacena todas las 15 resistencias leídas
float instruccionesColumnas[10];  // Las primeras 10 instrucciones (de las 2 columnas)
float bloqueControl[5];           // El bloque de control (las últimas 5 posiciones)

// --- DEFINICIONES DE ACCIONES ---
enum ActionType {
  MOVER_ARRIBA = 1,     // Resistencia 190-220 Ohms
  MOVER_ABAJO = 2,      // Resistencia 850-1100 Ohms
  MOVER_IZQUIERDA = 3,  // Resistencia 1.5k-2.5k Ohms
  MOVER_DERECHA = 4,    // Resistencia 3.5k-4.5k Ohms
  BLOQUE_CONTROL = 5,   // Resistencia 9k-11k Ohms (no invertible)
  NEGACION = 6,         // Resistencia 19k-21k Ohms (no invertible)
  MELODIA_1 = 7,        // Resistencia 5k-6k Ohms (no invertible)
};

// Variables de control de secuencia
bool secuenciaLista = false;

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- ESTADO GLOBAL DEL ROBOT ---
int robotX = 0;           // Posición actual en X (columna de la cuadrícula 0-4)
int robotY = 0;           // Posición actual en Y (fila de la cuadrícula 0-4)


// Crea un objeto PCA9685 , con Direccion para comunicacion I2C con el driver Pca9685 que controlara los LEDS de las instrucciones
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver("0x40");

// Define el número total de LEDs que vas a controlar
const int NUM_LEDS = 15;

// Define los valores de brillo (PWM)
// El PCA9685 tiene una resolución de 12 bits, lo que significa un rango de 0 a 4095.
const int BRIGHTNESS_OFF = 0;       // LED apagado
const int BRIGHTNESS_20_PERCENT = 4095 * 0.20; // Aproximadamente 819
const int BRIGHTNESS_100_PERCENT = 4095; // LED al máximo brillo


// --- DECLARACIÓN DE FUNCIONES---
void leerTodasColumnas();
void copiarArrays();
void ejecutarSecuencia();
ActionType getInvertedAction(ActionType originalAction);
void performAction(ActionType action, int globalIndex);
void executeBlockControlLogicInternal();
String getAction(ActionType action);
void setLedBrightness(int ledIndex, int brightness);
bool isValidPosition(int x, int y);

void setup() {

  Serial.begin(2400);  // Para comunicación con el Monitor Serial del PC

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();        // Inicia la comunicación I2C como Maestro

  // Inicia el PCA9685 y la comunicación I2C.
  pwm.begin();

  // Establece la frecuencia PWM. 1000 Hz ,
  pwm.setPWMFreq(1000);

  // Todos los leds los colocamos apagados al inicio
  for (int i = 0; i < NUM_LEDS; i++) {
    pwm.setPWM(i, 0, BRIGHTNESS_OFF); // Canal i, ON_time = 0, OFF_time = BRIGHTNESS_OFF
  }

  pinMode(BOTON_INICIO, INPUT_PULLUP);  // Configura el pin del botón con resistencia pull-up interna

  Serial.println("----------------------------------");
  Serial.println(" Arduino Central (Maestro I2C) ");
  Serial.println("----------------------------------");
  Serial.println("Iniciando. Esperando lecturas de columnas...");

  mySerial.begin(9600);  //colocamos el modulo bluetooh en 9600
}

void loop() {
  
  // Esta sección se ejecuta solo una vez si 'secuenciaLista' es falsa.
  if (!secuenciaLista) {
    leerTodasColumnas();  // Llama a la función para solicitar datos a los esclavos
    copiarArrays();       // Copia los valores leídos a los arrays de instrucciones y bloque de control

    //Colocamos los Leds al 20% mientras no se inicie el recorrido
    for (int i = 0; i < 15; i++) {
      if (allResistances[i] > 0) { // Si la resistencia es válida (hay una ficha)
        setLedBrightness(i, BRIGHTNESS_20_PERCENT); // Enciende el LED correspondiente al 20%
      } else {
        setLedBrightness(i, BRIGHTNESS_OFF); // Si no hay ficha o es inválida, asegúrate de que esté apagado
      }
    }
    

    delay(1000);  // Pequeña pausa para que no se sature el serial

  }

  // Si se presiona el botón, ejecutar secuencia
  // He re-habilitado la lectura del botón para que la secuencia no se ejecute inmediatamente.
  if (/*digitalRead(BOTON_INICIO) == LOW &&*/ !secuenciaLista) {
    secuenciaLista = true;
    //Serial.println("\nBoton de inicio presionado. Ejecutando secuencia...");
    ejecutarSecuencia();
    // secuenciaLista = false; // Descomentar si quieres reiniciar la secuencia al presionar de nuevo
    // Vuelve a recorrer y poner los LEDs al 20% para las fichas presentes
    for (int i = 0; i < 15; i++) {
      if (allResistances[i] > 0) {
        setLedBrightness(i, BRIGHTNESS_20_PERCENT);
      } else {
        setLedBrightness(i, BRIGHTNESS_OFF); // Asegúrate de apagar los que no tienen ficha
      }
    }
  }
  
}

// Lee los datos de los 3 esclavos I2C y llena el array 'allResistances'
void leerTodasColumnas() {
  //Serial.println("Solicitando datos a los esclavos I2C...");
  int currentGlobalIndex = 0;  // Para llevar el control de la posición en allResistances

  // 1. Leer de los dos primeros esclavos (columnas), cada uno con 5 resistencias
  for (int col = 0; col < 2; col++) {  // Para las 2 columnas
    int slaveAddress = SLAVE_ADDRESSES[col];
    
    int floatsToRequest = 5;     

    
    int bytesToRequest = floatsToRequest * sizeof(float);  // 5 floats * 4 bytes/float = 20 bytes

    Serial.print("Solicitando ");
    Serial.print(floatsToRequest);
    Serial.print(" floats del esclavo 0x");
    Serial.println(slaveAddress, HEX);

    Wire.requestFrom(slaveAddress, bytesToRequest);

    FloatBytes fb;
    for (int i = 0; i < floatsToRequest; i++) {
      if (Wire.available() >= sizeof(float)) {
        for (int j = 0; j < sizeof(float); j++) {
          fb.b[j] = Wire.read();
        }
        allResistances[currentGlobalIndex++] = fb.f;
      } else {
       
        allResistances[currentGlobalIndex++] = -1.0;
      }
    }
    delay(50);
  }

  // 2. Leer del tercer esclavo (bloque de control), con 4 resistencias
  int blockControlSlaveAddress = SLAVE_ADDRESSES[2];     // El tercer esclavo
  int floatsToRequest = 5;                               // Envía 4 floats
  int bytesToRequest = floatsToRequest * sizeof(float);  // 5 floats * 4 bytes/float = 16 bytes

  Wire.requestFrom(blockControlSlaveAddress, bytesToRequest);

  FloatBytes fb;
  for (int i = 0; i < floatsToRequest; i++) {
    if (Wire.available() >= sizeof(float)) {
      
      for (int j = 0; j < sizeof(float); j++) {
        fb.b[j] = Wire.read();
      }

      allResistances[currentGlobalIndex++] = fb.f;

    } else {
      
      allResistances[currentGlobalIndex++] = -1.0;
    }
  }
  delay(50);
}

// Copiar los valores después de leer todas las columnas
void copiarArrays() {
  // Copia las primeras 10 posiciones a instruccionesColumnas (las 2 columnas de 5 resistencias)
  for (int i = 0; i < 10; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  // Copia las últimas 4 posiciones a bloqueControl (el bloque de control)
  for (int i = 0; i < 5; i++) {
    bloqueControl[i] = allResistances[10 + i];  // Empieza en el índice 10
  }
}

// Ejecuta la secuencia de acciones basada en los valores de instruccionesColumnas.
void ejecutarSecuencia() {
  //Serial.println("Iniciando ejecucion de instrucciones...");

  bool negacionActiva = false;  // Flag para controlar si la negación está activa

  // Recorre las 10 instrucciones principales
  for (int i = 0; i < 10; i++) {  // Ahora son 10 instrucciones principales

    int instruccionActual = instruccionesColumnas[i];

    // Solo ejecutamos resistencias validas (positivas)
    if (instruccionActual > 0) {

      // --- Lógica de Negación ---
      if (negacionActiva) {
        negacionActiva = false;  // Desactiva la negación para la siguiente instrucción

        // Si la siguiente instrucción es no-invertible, se omite.
        if (instruccionActual == NEGACION || instruccionActual == BLOQUE_CONTROL || instruccionActual == MELODIA_1 || instruccionActual <= 0) {  // Incluye 0 y -1 para errores
          // No se llama a performAction ni se ejecuta nada, simplemente se omite.
        } else {
          // Si es invertible, se ejecuta la acción invertida
          performAction(getInvertedAction(instruccionActual), i);
        }
      }
      // --- Fin Lógica de Negación ---
      else {  // Si la negación NO está activa
        if (instruccionActual == NEGACION) {
          negacionActiva = true;  // Activa la negación para la PRÓXIMA instrucción
          //Serial.println(" -> Negacion activada. La proxima instruccion sera invertida.");
        } else if (instruccionActual == BLOQUE_CONTROL) {
          //Serial.println(" -> Accion: BLOQUE DE CONTROL (Ejecutar logica de bloque de control)");
          executeBlockControlLogicInternal();  // Llama a la función que maneja el bloque de control
        } else {
          // Para todas las demás acciones, ejecutar normalmente (incluidos los errores que no son negación)
          performAction(instruccionActual, i);
        }
      }
       
    }
  }
  //Serial.println("Fin de instrucciones principales.");
}

// Ejecuta una acción específica basada en el ActionType.
void performAction(ActionType action, int globalIndex) {
  
  Serial.print("Instruccion ");
  Serial.print(globalIndex + 1);
  Serial.print(": ");
  Serial.print(getAction(action));

  int nextX = robotX;
  int nextY = robotY;
  
  if (globalIndex >= 0 && globalIndex < NUM_LEDS) { // Asegura que el índice del LED es válido
    setLedBrightness(globalIndex, BRIGHTNESS_100_PERCENT); //encendemos el led al 100%
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
    case BLOQUE_CONTROL:
      break;
    case NEGACION:
      break;
    case MELODIA_1:
      break;
    default:
      break;
  }
  
  // Solo intenta mover si la acción fue una de movimiento
  if (action == MOVER_ARRIBA || action == MOVER_ABAJO || action == MOVER_IZQUIERDA || action == MOVER_DERECHA) {
    if (isValidPosition(nextX, nextY)) {
      robotX = nextX;  
      robotY = nextY;
    }
  }
  mySerial.println(action);
  delay(1000);
  Serial.println(); 
  
  // Bajar el brillo de los leds de nuevo al 20% después de ejecutar *****
  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    // Si la ficha sigue presente (su resistencia es válida), vuelve al 20%.
    // Si la ficha fue quitada (resistencia -1), se apagará en el próximo ciclo de lectura.
    if (allResistances[globalIndex] > 0) { // Verifica si la ficha aún es considerada válida
       setLedBrightness(globalIndex, BRIGHTNESS_20_PERCENT);
    } else {
       setLedBrightness(globalIndex, BRIGHTNESS_OFF); // Si la ficha fue quitada, apagar completamente
    }
  }
}

// Obtiene la acción invertida para una acción dada.
ActionType getInvertedAction(ActionType originalAction) {
  switch (originalAction) {
    case MOVER_ARRIBA: return MOVER_ABAJO;
    case MOVER_ABAJO: return MOVER_ARRIBA;
    case MOVER_IZQUIERDA: return MOVER_DERECHA;
    case MOVER_DERECHA: return MOVER_IZQUIERDA;
    default:
      return originalAction;  // Para acciones no invertibles o errores
  }
}

// Ejecuta la lógica del bloque de control, procesando las 4 resistencias de bloqueControl.
void executeBlockControlLogicInternal() {
  //Serial.println(" -> Ejecutando Bloque de Control Interno...");

  bool negacionActiva = false;

  for (int i = 0; i < 4; i++) {
    int controlResistencia = bloqueControl[i];

    if (controlResistencia > 0) {
      int controlAction = controlResistencia;

      // Si la instrucción del bloque de control es BLOQUE_CONTROL, la omitimos para evitar un bucle infinito.
      if (controlAction == BLOQUE_CONTROL) {
        //Serial.println("BLOQUE DE CONTROL (Omitido para evitar recursión)");
      } else {
        // --- Lógica de Negación ---
        if (negacionActiva) {
          negacionActiva = false;  // Desactiva la negación para la siguiente instrucción

          // Si la siguiente instrucción es no-invertible, se omite.
          if (controlAction == NEGACION || controlAction == MELODIA_1 || controlAction <= 0) {
            //Serial.println(" -> Negacion: Instruccion no invertible o de error. Omitir Instruccion.");
          } else {
            // Si es invertible, se ejecuta la acción invertida
            performAction(getInvertedAction(controlAction), i + 10);  
          }
        }
        // --- Fin Lógica de Negación ---
        else {  // Si la negación NO está activa
          if (controlAction == NEGACION) {
            negacionActiva = true;  // Activa la negación para la PRÓXIMA instrucción
            //Serial.println(" -> Negacion activada. La proxima instruccion sera invertida.");
          } else {
            // Para todas las demás acciones, ejecutar normalmente
            performAction(controlAction, i + 10); 
          }
        }
      }
    }
    delay(300);  // Pausa entre instrucciones de control
  }
}

// Verifica si una posición (x, y) está dentro de los límites de la cuadrícula.
bool isValidPosition(int x, int y) {
  int GRID_SIZE = 5;
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

//Devuelve el tipo de accion
String getAction(ActionType action){

  switch (action) {
    case MOVER_ARRIBA:
      return "Avanzar";
    case MOVER_ABAJO:
      return "Retroceder";
    case MOVER_IZQUIERDA:
      return "Izquierda";
    case MOVER_DERECHA:
      return "Derecha";
    case BLOQUE_CONTROL:
      return "Bloque Control";
    case NEGACION:
      return "Negacion";
    case MELODIA_1:
      return "Melodia";
    default:
      return "Ninguna instruccion";
  }
}

//  controla el brillo de un LED en el PCA9685 
void setLedBrightness(int ledIndex, int brightness) {
  if (ledIndex >= 0 && ledIndex < NUM_LEDS) {
    pwm.setPWM(ledIndex, 0, brightness);
  }
}