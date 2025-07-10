// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C)
// Solicita datos de 4 Arduinos de columna (Esclavos I2C) y consolida los valores
// de resistencia en un array de 16 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion IC2 con cada columna.
// ---------------------------------------------------------------------------------------

#include <Wire.h>    // Librería para comunicación I2C
#include <Stepper.h> // Librería para motor paso a paso

// --- DEFINICIONES DE PINES Y CONSTANTES GLOBALES ---

#define BOTON_INICIO 2 // Pin del botón (con resistencia pull-up)

// Definición de los pines para los motores paso a paso
// Asignación de pines para 3 motores paso a paso (CoreXY X, CoreXY Y, Eje Z)
// Ajusta estos pines según tus conexiones físicas.
// Si tu motor usa 2 pines (paso/dirección) o 4 pines (bobinas), ajusta la definición de Stepper.
// Aquí asumimos 4 pines por motor.
#define MOTOR_X_PIN1 8
#define MOTOR_X_PIN2 9
#define MOTOR_X_PIN3 10
#define MOTOR_X_PIN4 11

#define MOTOR_Y_PIN1 6  // Pines digitales disponibles: D6, D7, D12, D13
#define MOTOR_Y_PIN2 7
#define MOTOR_Y_PIN3 12
#define MOTOR_Y_PIN4 13

#define MOTOR_Z_PIN1 A0 // Pines analógicos como digitales: A0, A1, A2, A3
#define MOTOR_Z_PIN2 A1
#define MOTOR_Z_PIN3 A2
#define MOTOR_Z_PIN4 A3

// Objetos de los motores paso a paso. Se inicializan aquí globalmente.
// Los 2048 pasos por revolución es un ejemplo (para un 28BYJ-48 con driver ULN2003).
Stepper motorX(2048, MOTOR_X_PIN1, MOTOR_X_PIN2, MOTOR_X_PIN3, MOTOR_X_PIN4);
Stepper motorY(2048, MOTOR_Y_PIN1, MOTOR_Y_PIN2, MOTOR_Y_PIN3, MOTOR_Y_PIN4);
Stepper motorZ(2048, MOTOR_Z_PIN1, MOTOR_Z_PIN2, MOTOR_Z_PIN3, MOTOR_Z_PIN4);


// Direcciones I2C de los Arduinos de las columnas
const int COLUMN_ADDRESSES[] = {0x01, 0x02, 0x03, 0x04};
const int NUM_COLUMNS = 4;
const int RESISTANCES_PER_COLUMN = 4;
const int TOTAL_RESISTANCES = NUM_COLUMNS * RESISTANCES_PER_COLUMN; // 4 columnas * 4 resistencias = 16

// Arrays para almacenar los datos
float allResistances[TOTAL_RESISTANCES];         // Almacena todas las 16 resistencias leídas
float instruccionesColumnas[12];                 // Almacena las instrucciones de las primeras 3 columnas (12 posiciones)
float bloqueControl[4];                          // Almacena el bloque de control (última columna, 4 posiciones)

// --- DEFINICIONES DE ACCIONES ---
// Estos valores representan el tipo de acción que se realizará.
enum ActionType {
  ACTION_NONE = 0,            // Sin acción / Valor fuera de rango
  AVANZADA = 1,               // Resistencia 190-220 Ohms
  RETROCESO = 2,              // Resistencia 850-1100 Ohms
  GIRAR_IZQUIERDA = 3,        // Resistencia 1.5k-2.5k Ohms
  GIRAR_DERECHA = 4,          // Resistencia 3.5k-4.5k Ohms
  BLOQUE_CONTROL = 5,         // Resistencia 9k-11k Ohms (no invertible)
  NEGACION = 6,               // Resistencia 19k-21k Ohms (no invertible)
  MELODIA_1 = 7,              // Resistencia 5k-6k Ohms (no invertible)
  MELODIA_2 = 8,              // Resistencia 7k-8k Ohms (no invertible)
  ERROR = -1,                // Error en lectura de Vin / comunicacion i2c / circuito abierto / corto invalido / resistencia fuera de rango 
};

// Variables de control de secuencia
bool secuenciaLista = false;

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
// Esto es necesario porque Wire.write() y Wire.read() operan con bytes.
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- ESTADO GLOBAL DEL ROBOT ---
int robotX = 0; // Posición actual en X (columna de la cuadrícula 0-5)
int robotY = 0; // Posición actual en Y (fila de la cuadrícula 0-5)

enum RobotOrientation {
  NORTH, // Orientación Norte (hacia +Y)
  EAST,  // Orientación Este (hacia +X)
  SOUTH, // Orientación Sur (hacia -Y)
  WEST   // Orientación Oeste (hacia -X)
};
RobotOrientation robotOrientation = NORTH; // Orientación inicial del robot

// --- DIMENSIONES DEL TABLERO ---
const int GRID_SIZE = 6; // Cuadrícula de 6x6

// --- DECLARACIÓN DE FUNCIONES (PROTOTIPOS) ---
// Se declaran aquí porque las funciones se llaman entre sí antes de ser definidas.
void leerTodasColumnas();
void copiarArrays();
void ejecutarSecuencia();
ActionType mapResistanceToAction(float resistanceValue);
void performAction(ActionType action, int globalIndex, float resistanceValue);
ActionType getInvertedAction(ActionType originalAction);
void executeBlockControlLogicInternal();

// Funciones de movimiento del robot
bool isValidPosition(int x, int y);
void printRobotState();
void moveXY_steps(int deltaX_steps, int deltaY_steps); // Para movimiento CoreXY
void moveZ_steps(int deltaZ_steps); // Para movimiento del eje Z
void rotateRobot(int direction); // Para rotación (ej. del cabezal o del robot si es omnidireccional)

// Funciones de acción (placeholders, implementar según tu tesis)
void playMelody1();
void playMelody2();
void handleNegationAction();


void setup() {
  Wire.begin(); // Inicia la comunicación I2C como Maestro
  Serial.begin(4800); // Para comunicación con el Monitor Serial del PC

  pinMode(BOTON_INICIO, INPUT_PULLUP); // Configura el pin del botón con resistencia pull-up interna

  // Configura la velocidad de los motores paso a paso (pasos por minuto)
  motorX.setSpeed(60); // Ejemplo: 60 RPM
  motorY.setSpeed(60);
  motorZ.setSpeed(60);

  Serial.println("----------------------------------");
  Serial.println("   Arduino Central (Maestro I2C)  ");
  Serial.println("----------------------------------");
  Serial.println("Iniciando. Esperando lecturas de columnas...");

  printRobotState(); // Imprime el estado inicial del robot
}

// --- FUNCIÓN loop() ---
// Se ejecuta repetidamente después de setup().
void loop() {
  // 1. Leer todas las columnas y guardar en el array
  // Esta sección se ejecuta solo una vez al inicio o hasta que se presione el botón
  // si 'secuenciaLista' es falsa.
  if (!secuenciaLista) {
    leerTodasColumnas(); // Llama a la función para solicitar datos a los esclavos
    copiarArrays();     // Copia los valores leídos a los arrays de instrucciones y bloque de control
    Serial.println("Instrucciones cargadas. Esperando botón...");

    // Imprimir todos los valores leídos para verificación en el Monitor Serial
    Serial.println("Valores de todas las resistencias leídas:");
    for (int i = 0; i < TOTAL_RESISTANCES; i++) {
      Serial.print(" (Columna ");
      Serial.print((i / RESISTANCES_PER_COLUMN) + 1); // Calcula la columna
      Serial.print(", Rx ");
      Serial.print((i % RESISTANCES_PER_COLUMN) + 1); // Calcula el Rx dentro de la columna
      Serial.print("): ");

      Serial.print(": ");
      // Formatear la salida para mayor legibilidad (Ohms, kOhms, MOhms)
      if (allResistances[i] < 0) {
        Serial.println("CORTO / Valor Invalido");
      } else if (allResistances[i] >= 1000.0) {
        Serial.print(allResistances[i] / 1000.0, 2);
        Serial.println(" kOhms");
      } else {
        Serial.print(allResistances[i], 2);
        Serial.println(" Ohms");
      }
    }
    Serial.println("----------------------------------");
    delay(1000); // Pequeña pausa para que no se sature el serial
  }

  // 2. Si se presiona el botón, ejecutar secuencia
  // digitalRead(BOTON_INICIO) == LOW porque se usa INPUT_PULLUP (botón a GND)
  if (digitalRead(BOTON_INICIO) == LOW && !secuenciaLista) {
    secuenciaLista = true; // Marca que la secuencia está lista para ejecutarse
    ejecutarSecuencia();   // Llama a la función que contiene la lógica de tu tesis
    secuenciaLista = false; // Permite reiniciar la secuencia después de completarla
    Serial.println("Secuencia completada.");
  }
}

// --- DEFINICIÓN DE FUNCIONES ---

// Lee las 4 columnas via I2C y llena el array 'allResistances'
void leerTodasColumnas() {
  Serial.println("Solicitando datos a las columnas I2C...");
  for (int col = 0; col < NUM_COLUMNS; col++) {
    int slaveAddress = COLUMN_ADDRESSES[col];
    // Cada float ocupa 4 bytes. Queremos 4 floats por columna, así que 16 bytes.
    int bytesToRequest = RESISTANCES_PER_COLUMN * sizeof(float); // 4 floats * 4 bytes/float = 16 bytes

    // Solicita los bytes de datos al esclavo
    Wire.requestFrom(slaveAddress, bytesToRequest);

    // Lee los bytes recibidos y los convierte de nuevo a floats
    FloatBytes fb; // Objeto union para la conversión de bytes a float
    for (int i = 0; i < RESISTANCES_PER_COLUMN; i++) {
      // Índice en el array principal (allResistances)
      int globalIndex = col * RESISTANCES_PER_COLUMN + i;

      if (Wire.available() >= sizeof(float)) { 
        for (int j = 0; j < sizeof(float); j++) {
          fb.b[j] = Wire.read(); // Lee los 4 bytes que componen el float
        }
        // Almacena el float reconstruido en el array principal
        allResistances[globalIndex] = fb.f;
      } else {
        // Manejo de error si no se recibieron suficientes bytes para un float completo
        Serial.print("ERROR: No se recibieron suficientes bytes de 0x");
        Serial.print(slaveAddress, HEX);
        Serial.print(" para la resistencia ");
        Serial.println(i + 1);
        allResistances[globalIndex] = ERROR; // Valor de error para indicar fallo en la lectura I2C
      }
    }
    delay(50); // Pequeña pausa entre solicitudes a diferentes esclavos para evitar saturación
  }
}

// Copiar los valores después de leer todas las columnas
void copiarArrays() {
  // Copia las primeras 12 posiciones a instruccionesColumnas (columnas 1 a 3)
  for (int i = 0; i < 12; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  // Copia las últimas 4 posiciones a bloqueControl (columna 4)
  for (int i = 0; i < 4; i++) {
    bloqueControl[i] = allResistances[12 + i];
  }
}

// Ejecuta la secuencia de acciones basada en los valores de instruccionesColumnas.
void ejecutarSecuencia() {
  Serial.println("Iniciando ejecucion de instrucciones...");

  bool negacionActiva = false; // Flag para controlar si la negación está activa

  // Recorre las primeras 3 columnas (12 resistencias) para las instrucciones principales
  for (int i = 0; i < 12; i++) {
    float resistenciaActual = instruccionesColumnas[i];
    ActionType action = mapResistanceToAction(resistenciaActual); // Mapea la resistencia a un tipo de acción

    // --- Lógica de Negación ---
    if (negacionActiva) {
      negacionActiva = false; // Desactiva la negación para la siguiente instrucción

      // Si la siguiente instrucción es no-invertible, se ejecuta normalmente
      if (action == NEGACION || action == BLOQUE_CONTROL || action == MELODIA_1 || action == MELODIA_2) {
        Serial.println("  -> Negacion: Instruccion no invertible. Omitir Instruccion.");
        continue; // Omite la ejecución de esta instrucción y pasa a la siguiente en el bucle
      } else {
        // Si es invertible, se ejecuta la acción invertida
        Serial.println("  -> Negacion: Invirtiendo la siguiente instruccion.");
        ActionType invertedAction = getInvertedAction(action);
        performAction(invertedAction, i, resistenciaActual);
      }
    }
    // --- Fin Lógica de Negación ---
    else { // Si la negación NO está activa
      if (action == NEGACION) {
        negacionActiva = true; // Activa la negación para la PRÓXIMA instrucción
        Serial.println("  -> Negacion activada. La proxima instruccion sera invertida.");
      } else if (action == BLOQUE_CONTROL) {
        Serial.println("  -> Accion: BLOQUE DE CONTROL (Ejecutar logica de bloque de control)");
        executeBlockControlLogicInternal(); // Llama a la función que maneja el bloque de control
      } else {
        // Para todas las demás acciones, ejecutar normalmente
        performAction(action, i, resistenciaActual);
      }
    }
    delay(500); // Pausa entre el procesamiento de cada resistencia (instrucción)
  }
  Serial.println("Fin de instrucciones principales.");
}

// Mapea un valor de resistencia a un tipo de acción (no ejecuta, solo clasifica).
ActionType mapResistanceToAction(float resistanceValue) {
  if (resistanceValue >= 190.0 && resistanceValue <= 220.0) {
    return AVANZADA;
  } else if (resistanceValue >= 850.0 && resistanceValue <= 1100.0) {
    return RETROCESO;
  } else if (resistanceValue >= 1500.0 && resistanceValue <= 2500.0) {
    return GIRAR_IZQUIERDA;
  } else if (resistanceValue >= 3500.0 && resistanceValue <= 4500.0) {
    return GIRAR_DERECHA;
  } else if (resistanceValue >= 9000.0 && resistanceValue <= 11000.0) {
    return BLOQUE_CONTROL;
  } else if (resistanceValue >= 19000.0 && resistanceValue <= 21000.0) {
    return NEGACION;
  } else if (resistanceValue >= 5000.0 && resistanceValue <= 6000.0) {
    return MELODIA_1;
  } else if (resistanceValue >= 7000.0 && resistanceValue <= 8000.0) {
    return MELODIA_2;
  }
  // Manejo de errores o valores fuera de rango (usando los códigos de error definidos)
  else {
    return ERROR;
  }
}

// Ejecuta una acción específica basada en el ActionType.
void performAction(ActionType action, int globalIndex, float resistanceValue) {
  // Imprime el valor de la resistencia actual para el contexto
  Serial.print("  Resistencia ");
  Serial.print(globalIndex + 1);
  Serial.print(": ");
  if (resistanceValue >= 1000.0) {
    Serial.print(resistanceValue / 1000.0, 2);
    Serial.print(" kOhms -> ");
  } else {
    Serial.print(resistanceValue, 2);
    Serial.print(" Ohms -> ");
  }

  printRobotState(); // Imprime el estado del robot ANTES de la acción

  // Ejecución de la acción basada en el tipo de acción
  switch (action) {
    case AVANZADA:
      Serial.println("AVANZADA (Mover motor hacia adelante)");
      { // Bloque de ámbito para variables locales
        int nextX = robotX;
        int nextY = robotY;
        // Calcula la próxima posición basada en la orientación actual
        switch (robotOrientation) {
          case NORTH: nextY++; break;
          case EAST:  nextX++; break;
          case SOUTH: nextY--; break;
          case WEST:  nextX--; break;
        }
        if (isValidPosition(nextX, nextY)) {
          // Asumimos 100 pasos por unidad de cuadrícula para CoreXY. Ajusta según tu calibración.
          int deltaX_steps = (nextX - robotX) * 100;
          int deltaY_steps = (nextY - robotY) * 100;
          moveXY_steps(deltaX_steps, deltaY_steps);
          robotX = nextX; // Actualiza la posición del robot
          robotY = nextY;
        } else {
          Serial.println("    IGNORADO: Limite de tablero alcanzado.");
        }
      }
      break;
    case RETROCESO:
      Serial.println("RETROCESO (Mover motor hacia atras)");
      { // Bloque de ámbito para variables locales
        int nextX = robotX;
        int nextY = robotY;
        // Calcula la próxima posición basada en la orientación actual (dirección opuesta)
        switch (robotOrientation) {
          case NORTH: nextY--; break;
          case EAST:  nextX--; break;
          case SOUTH: nextY++; break;
          case WEST:  nextX++; break;
        }
        if (isValidPosition(nextX, nextY)) {
          // Asumimos 100 pasos por unidad de cuadrícula para CoreXY. Ajusta según tu calibración.
          int deltaX_steps = (nextX - robotX) * 100;
          int deltaY_steps = (nextY - robotY) * 100;
          moveXY_steps(deltaX_steps, deltaY_steps);
          robotX = nextX; // Actualiza la posición del robot
          robotY = nextY;
        } else {
          Serial.println("    IGNORADO: Limite de tablero alcanzado.");
        }
      }
      break;
    case GIRAR_IZQUIERDA:
      Serial.println("GIRAR IZQUIERDA (Mover motor para girar izquierda)");
      rotateRobot(-1); // -1 para girar a la izquierda
      // Actualiza la orientación del robot
      switch (robotOrientation) {
        case NORTH: robotOrientation = WEST; break;
        case EAST:  robotOrientation = NORTH; break;
        case SOUTH: robotOrientation = EAST; break;
        case WEST:  robotOrientation = SOUTH; break;
      }
      break;
    case GIRAR_DERECHA:
      Serial.println("GIRAR DERECHA (Mover motor para girar derecha)");
      rotateRobot(1); // +1 para girar a la derecha
      // Actualiza la orientación del robot
      switch (robotOrientation) {
        case NORTH: robotOrientation = EAST; break;
        case EAST:  robotOrientation = SOUTH; break;
        case SOUTH: robotOrientation = WEST; break;
        case WEST:  robotOrientation = NORTH; break;
      }
      break;
    case BLOQUE_CONTROL: // Esto no debería ser llamado directamente desde ejecutarSecuencia si se maneja allí
      Serial.println("BLOQUE DE CONTROL (Ya manejado en ejecutarSecuencia)");
      break;
    case NEGACION: // Esto no debería ser llamado directamente desde ejecutarSecuencia si se maneja allí
      Serial.println("NEGACION (Ya manejado en ejecutarSecuencia)");
      break;
    case MELODIA_1:
      Serial.println("MELODIA 1 (Reproducir melodia 1)");
      playMelody1();
      break;
    case MELODIA_2:
      Serial.println("MELODIA 2 (Reproducir melodia 2)");
      playMelody2();
      break;
    case ERROR:
      Serial.println("CORTO INVALIDO (No ejecutar accion)");
      break;
    case ACTION_NONE:
    default:
      Serial.println("NINGUNA / DESCONOCIDA (No ejecutar accion)");
      break;
  }
  printRobotState(); // Imprime el estado del robot DESPUÉS de la acción
}

// Obtiene la acción invertida para una acción dada.
ActionType getInvertedAction(ActionType originalAction) {
  switch (originalAction) {
    case AVANZADA: return RETROCESO;
    case RETROCESO: return AVANZADA;
    case GIRAR_IZQUIERDA: return GIRAR_DERECHA;
    case GIRAR_DERECHA: return GIRAR_IZQUIERDA;
    default:
      return originalAction; // Esto solo se alcanzaría si se llama con un tipo no invertible,
                             // lo cual el flujo de ejecutarSecuencia ya previene.
  }
}

// Ejecuta la lógica del bloque de control, procesando las 4 resistencias de bloqueControl.
void executeBlockControlLogicInternal() {
  Serial.println("  -> Ejecutando Bloque de Control Interno...");
  for (int i = 0; i < 4; i++) {
    float controlResistencia = bloqueControl[i];
    ActionType controlAction = mapResistanceToAction(controlResistencia);

    Serial.print("    Control Bloque "); Serial.print(i+1); Serial.print(": ");
    if (controlResistencia >= 1000.0) {
      Serial.print(controlResistencia / 1000.0, 2);
      Serial.print(" kOhms -> ");
    } else {
      Serial.print(controlResistencia, 2);
      Serial.print(" Ohms -> ");
    }

    // Si la instrucción del bloque de control es BLOQUE_CONTROL, la omitimos para evitar un bucle infinito.
    if (controlAction == BLOQUE_CONTROL) {
      Serial.println("BLOQUE DE CONTROL (Omitido para evitar recursión)");
    } else {
      // Ejecutamos la acción del bloque de control (sin manejar negación aquí)
      performAction(controlAction, i, controlResistencia); // Reutilizamos performAction
    }
    delay(300); // Pausa entre instrucciones de control
  }
  Serial.println("  -> Fin de Bloque de Control Interno.");
}

// --- FUNCIONES DE MOVIMIENTO DEL ROBOT ---

// Verifica si una posición (x, y) está dentro de los límites de la cuadrícula.
bool isValidPosition(int x, int y) {
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

// Imprime el estado actual del robot (posición y orientación) en el Monitor Serial.
void printRobotState() {
  Serial.print("  Posicion actual: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print(") Orientacion: ");
  switch (robotOrientation) {
    case NORTH: Serial.println("NORTE"); break;
    case EAST:  Serial.println("ESTE");  break;
    case SOUTH: Serial.println("SUR");   break;
    case WEST:  Serial.println("OESTE"); break;
  }
}

// Placeholder para el movimiento CoreXY.
// DEBES IMPLEMENTAR LA CINEMÁTICA REAL DE COREXY AQUÍ.
// deltaX_steps y deltaY_steps son los pasos netos que el robot debería moverse en X e Y.
void moveXY_steps(int deltaX_steps, int deltaY_steps) {
  Serial.print("    (CoreXY) Moviendo en X: "); Serial.print(deltaX_steps);
  Serial.print(", Y: "); Serial.print(deltaY_steps); Serial.println(" pasos.");
  // --- IMPLEMENTACIÓN DE LA CINEMÁTICA COREXY ---
  // Para un sistema CoreXY, un movimiento en X o Y requiere que AMBOS motores (motorX y motorY)
  // se muevan una cantidad específica de pasos, a menudo combinando sus movimientos.
  // Ejemplo conceptual (NO ES LA IMPLEMENTACIÓN COMPLETA DE COREXY):
  // int motor1_actual_steps = deltaX_steps + deltaY_steps;
  // int motor2_actual_steps = deltaX_steps - deltaY_steps;
  // motorX.step(motor1_actual_steps);
  // motorY.step(motor2_actual_steps);
  // Por ahora, solo para simulación, movemos un motor si el delta es significativo.
  // ELIMINA ESTAS LÍNEAS Y REEMPLAZALAS CON TU LÓGICA COREXY REAL.
  if (deltaX_steps != 0) motorX.step(deltaX_steps);
  if (deltaY_steps != 0) motorY.step(deltaY_steps);
  // ---------------------------------------------
}

// Mueve el motor del eje Z.
void moveZ_steps(int deltaZ_steps) {
  Serial.print("    Moviendo eje Z: ");
  Serial.print(deltaZ_steps);
  Serial.println(" pasos.");
  motorZ.step(deltaZ_steps); // Mueve el motor del eje Z el número de pasos especificado
}

// Placeholder para la rotación del robot.
// DEBES IMPLEMENTAR LA LÓGICA REAL DE ROTACIÓN AQUÍ.
// 'direction' puede ser +1 para derecha, -1 para izquierda.
void rotateRobot(int direction) {
  Serial.print("    Rotando robot ");
  if (direction == 1) Serial.println("a la DERECHA.");
  else Serial.println("a la IZQUIERDA.");
  // --- IMPLEMENTACIÓN DE LA ROTACIÓN FÍSICA DEL ROBOT ---
  // Esto dependerá de cómo tu robot CoreXY maneje la rotación.
  // Podría ser un movimiento combinado de motorX y motorY si el cabezal rota,
  // o si el robot tiene un mecanismo de giro independiente.
  // Por ahora, solo una impresión.
  // -----------------------------------------------------
}

// --- FUNCIONES DE ACCIÓN (PLACEHOLDERS) ---
// DEBES IMPLEMENTAR LA LÓGICA REAL PARA CADA UNA DE ESTAS FUNCIONES EN TU TESIS.

void playMelody1() {
  Serial.println("    (Placeholder) Reproduciendo Melodia 1...");
  // Aquí iría el código para reproducir la Melodia 1 (ej. con un buzzer, Tone.h)
}

void playMelody2() {
  Serial.println("    (Placeholder) Reproduciendo Melodia 2...");
  // Aquí iría el código para reproducir la Melodia 2
}

void handleNegationAction() {
  Serial.println("    (Placeholder) Realizando accion de Negacion (si aplica)...");
  // Aquí iría el código para la acción de Negación en sí misma,
  // si la "negación" no solo invierte la siguiente, sino que también tiene una acción propia.
}
