// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C) - Archivo Único
// Solicita datos de 2 Arduinos de columna (Esclavos I2C, 5 resistencias cada uno)
// y un tercer Arduino (Esclavo I2C, Bloque de Control de 4 resistencias).
// Consolida los valores de resistencia en un array de 14 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion I2C con cada columna.
// El movimiento del robot ahora es en direcciones cardinales fijas (arriba, abajo, izquierda, derecha)
// sin depender de una orientación interna del robot.
// ---------------------------------------------------------------------------------------

#include <Wire.h>  // Librería para comunicación I2C
#include <SoftwareSerial.h>

// --- DEFINICIONES DE PINES Y CONSTANTES GLOBALES ---

#define BOTON_INICIO 2  // Pin del botón (con resistencia pull-up)

SoftwareSerial mySerial(10, 11); // RX, TX

// Direcciones I2C de los Arduinos esclavos: Columna 1, Columna 2, Bloque de Control
const int SLAVE_ADDRESSES[] = {0x01, 0x02, 0x03};

// Arrays para almacenar los datos
float allResistances[14];         // Almacena todas las 14 resistencias leídas
float instruccionesColumnas[10];  // Las primeras 10 instrucciones (de las 2 columnas)
float bloqueControl[5];           // El bloque de control (las últimas 4 posiciones)

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
const int GRID_SIZE = 5;  // Cuadrícula de 5x5 (0 a 4)

// --- DECLARACIÓN DE FUNCIONES---
void leerTodasColumnas();
void copiarArrays();
void ejecutarSecuencia();
ActionType mapResistanceToAction(float resistanceValue);
ActionType getInvertedAction(ActionType originalAction);
void performAction(ActionType action, int globalIndex, float resistanceValue);
void executeBlockControlLogicInternal();

// Funciones de movimiento y auxiliares del robot
bool isValidPosition(int x, int y);
// void moveXY_steps(int deltaX_steps, int deltaY_steps); // Esta función está declarada pero no definida en el código proporcionado


void setup() {

  Serial.begin(2400);  // Para comunicación con el Monitor Serial del PC

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();        // Inicia la comunicación I2C como Maestro

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
    //Serial.println("Instrucciones cargadas. Esperando botón...");

    // Imprimir todos los valores leídos para verificación en el Monitor Serial
    //Serial.println("Valores de todas las resistencias leídas:");
    for (int i = 0; i < 15; i++) {  // Ahora es 15 para allResistances
      //Serial.print(" (Resistencia ");
      //Serial.print(i + 1);
      //Serial.print("): ");

      // Formatear la salida para mayor legibilidad (Ohms, kOhms, MOhms)
      if (allResistances[i] < 0) {
        //Serial.println("CORTO / Valor Invalido");
      } else if (allResistances[i] >= 1000.0) {
        //Serial.print(allResistances[i] / 1000.0, 2);
        //Serial.println(" kOhms");
      } else {
        //Serial.print(allResistances[i], 2);
        //Serial.println(" Ohms");
      }
    }
    //Serial.println("----------------------------------");
    delay(1000);  // Pequeña pausa para que no se sature el serial

  }

  // Si se presiona el botón, ejecutar secuencia
  // He re-habilitado la lectura del botón para que la secuencia no se ejecute inmediatamente.
  if (/*digitalRead(BOTON_INICIO) == LOW &&*/ !secuenciaLista) {
    secuenciaLista = true;
    //Serial.println("\nBoton de inicio presionado. Ejecutando secuencia...");
    ejecutarSecuencia();
    secuenciaLista = false; // Descomentar si quieres reiniciar la secuencia al presionar de nuevo
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

  Serial.print("Solicitando ");
  Serial.print(floatsToRequest);
  Serial.print(" floats del esclavo Bloque de Control 0x");
  Serial.println(blockControlSlaveAddress, HEX);

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
    float resistenciaActual = instruccionesColumnas[i];

    // Solo ejecutamos resistencias validas (positivas)
    if (resistenciaActual > 0) {
      ActionType action = mapResistanceToAction(resistenciaActual);  // Mapea la resistencia a un tipo de acción

      // --- Lógica de Negación ---
      if (negacionActiva) {
        negacionActiva = false;  // Desactiva la negación para la siguiente instrucción

        // Si la siguiente instrucción es no-invertible, se omite.
        if (action == NEGACION || action == BLOQUE_CONTROL || action == MELODIA_1 || action <= 0) {  // Incluye 0 y -1 para errores
          //Serial.println(" -> Negacion: Instruccion no invertible o de error. Omitir Instruccion.");
          // No se llama a performAction ni se ejecuta nada, simplemente se omite.
        } else {
          // Si es invertible, se ejecuta la acción invertida
          //Serial.println(" -> Negacion: Invirtiendo la siguiente instruccion.");
          performAction(getInvertedAction(action), i, resistenciaActual);
        }
      }
      // --- Fin Lógica de Negación ---
      else {  // Si la negación NO está activa
        if (action == NEGACION) {
          negacionActiva = true;  // Activa la negación para la PRÓXIMA instrucción
          //Serial.println(" -> Negacion activada. La proxima instruccion sera invertida.");
        } else if (action == BLOQUE_CONTROL) {
          //Serial.println(" -> Accion: BLOQUE DE CONTROL (Ejecutar logica de bloque de control)");
          executeBlockControlLogicInternal();  // Llama a la función que maneja el bloque de control
        } else {
          // Para todas las demás acciones, ejecutar normalmente (incluidos los errores que no son negación)
          performAction(action, i, resistenciaActual);
        }
      }
       
    }
  }
  //Serial.println("Fin de instrucciones principales.");
}

// Mapea un valor de resistencia a un tipo de acción (no ejecuta, solo clasifica).
ActionType mapResistanceToAction(float resistanceValue) {
  if (resistanceValue >= 50.0 && resistanceValue <= 220.0) {
    return MOVER_ARRIBA;
  } else if (resistanceValue >= 850.0 && resistanceValue <= 1100.0) {
    return MOVER_ABAJO;
  } else if (resistanceValue >= 1500.0 && resistanceValue <= 2500.0) {
    return MOVER_IZQUIERDA;
  } else if (resistanceValue >= 3500.0 && resistanceValue <= 4800.0) {
    return MOVER_DERECHA;
  } else if (resistanceValue >= 9000.0 && resistanceValue <= 11000.0) {
    return BLOQUE_CONTROL;
  } else if (resistanceValue >= 19000.0 && resistanceValue <= 21000.0) {
    return NEGACION;
  } else if (resistanceValue >= 5000.0 && resistanceValue <= 6000.0) {
    return MELODIA_1;
  }
  // Manejo de errores o valores fuera de rango
  else if (resistanceValue <= 0) {
    return (ActionType)-1;  // Valor de error para indicar corto/valor inválido
  } else {
    return (ActionType)0;  // Si es un valor positivo pero no cae en ningún rango definido
  }
}

// Ejecuta una acción específica basada en el ActionType.
void performAction(ActionType action, int globalIndex, float resistanceValue) {
  //Serial.print(" Accion -> ");
  //Serial.print(action);
  //Serial.print(" de la ");

  Serial.print(" Resistencia ");
  Serial.print(globalIndex + 1);
  Serial.print(": ");
  if (resistanceValue >= 1000.0) {
    Serial.print(resistanceValue / 1000.0, 2);
    Serial.print(" kOhms -> ");
  } else {
    Serial.print(resistanceValue, 2);
    Serial.print(" Ohms -> ");
  }

  int nextX = robotX;
  int nextY = robotY;
  
  switch (action) {
    case MOVER_ARRIBA:
      //Serial.print("MOVER ARRIBA (Mover motor hacia +Y)");
      nextY++;
      break;
    case MOVER_ABAJO:
      //Serial.print("MOVER ABAJO (Mover motor hacia -Y)");
      nextY--;
      break;
    case MOVER_IZQUIERDA:
      //Serial.print("MOVER IZQUIERDA (Mover motor hacia -X)");
      nextX--;
      break;
    case MOVER_DERECHA:
      //Serial.print("MOVER DERECHA (Mover motor hacia +X)");
      nextX++;
      break;
    case BLOQUE_CONTROL:
      //Serial.println("BLOQUE DE CONTROL (Ya manejado en ejecutarSecuencia)");
      break;
    case NEGACION:
      //Serial.println("NEGACION (Ya manejado en ejecutarSecuencia)");
      break;
    case MELODIA_1:
      //Serial.println("MELODIA 1 (Reproducir melodia 1)");
      break;
    default:
      //Serial.print("NINGUNA / DESCONOCIDA (No ejecutar accion)");
      break;
  }

  // Solo intenta mover si la acción fue una de movimiento
  if (action == MOVER_ARRIBA || action == MOVER_ABAJO || action == MOVER_IZQUIERDA || action == MOVER_DERECHA) {
    if (isValidPosition(nextX, nextY)) {
      // Asumimos 100 pasos por unidad de cuadrícula para CoreXY. Ajusta según tu calibración.
      int deltaX_steps = (nextX - robotX) * 100;
      int deltaY_steps = (nextY - robotY) * 100;
      robotX = nextX;  // Actualiza la posición del robot
      robotY = nextY;
    }
  }
  mySerial.println(action);
  delay(3000);// Pausa entre el procesamiento de cada resistencia (instrucción)
  Serial.println();  // Salto de línea para la siguiente impresión
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
    float controlResistencia = bloqueControl[i];

    if (controlResistencia > 0) {
      ActionType controlAction = mapResistanceToAction(controlResistencia);

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
            //Serial.println(" -> Negacion: Invirtiendo la siguiente instruccion.");
            performAction(getInvertedAction(controlAction), i + 10, controlResistencia);  // Ajusta globalIndex para bloque de control
          }
        }
        // --- Fin Lógica de Negación ---
        else {  // Si la negación NO está activa
          if (controlAction == NEGACION) {
            negacionActiva = true;  // Activa la negación para la PRÓXIMA instrucción
            //Serial.println(" -> Negacion activada. La proxima instruccion sera invertida.");
          } else {
            // Para todas las demás acciones, ejecutar normalmente
            performAction(controlAction, i + 10, controlResistencia);  // Ajusta globalIndex
          }
        }
      }
    }
    delay(300);  // Pausa entre instrucciones de control
  }
  //Serial.println(" -> Fin de Bloque de Control Interno.");
}

// --- FUNCIONES DE MOVIMIENTO Y AUXILIARES DEL ROBOT ---

// Verifica si una posición (x, y) está dentro de los límites de la cuadrícula.
bool isValidPosition(int x, int y) {
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}
