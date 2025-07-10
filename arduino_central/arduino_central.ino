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

// Definición de los pines del motor paso a paso
// Asegúrate de que estos pines coincidan con la conexión de tu motor
#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_PIN3 10
#define MOTOR_PIN4 11

// Objeto del motor paso a paso. Se inicializa aquí globalmente.
Stepper motor(2048, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4); // Definición de motor 1

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

// --- DECLARACIÓN DE FUNCIONES (PROTOTIPOS) ---
// Se declaran aquí porque las funciones se llaman entre sí antes de ser definidas.
void leerTodasColumnas();
void copiarArrays();
void ejecutarSecuencia();
ActionType mapResistanceToAction(float resistanceValue);
void performAction(ActionType action, int globalIndex, float resistanceValue);
ActionType getInvertedAction(ActionType originalAction);
void executeBlockControlLogicInternal();
void moveMotor(int steps);
void playMelody1();
void playMelody2();
void handleNegationAction();


// --- FUNCIÓN setup() ---
// Se ejecuta una sola vez al encender o reiniciar el Arduino.
void setup() {
  Wire.begin(); // Inicia la comunicación I2C como Maestro
  Serial.begin(4800); // Para comunicación con el Monitor Serial del PC

  pinMode(BOTON_INICIO, INPUT_PULLUP); // Configura el pin del botón con resistencia pull-up interna

  // Configura la velocidad del motor paso a paso (pasos por minuto)
  motor.setSpeed(60); // Ejemplo: 60 RPM (ajustar según tu motor y necesidades)

  Serial.println("----------------------------------");
  Serial.println("   Arduino Central (Maestro I2C)  ");
  Serial.println("----------------------------------");
  Serial.println("Iniciando. Esperando lecturas de columnas...");
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

      if (Wire.available() >= sizeof(float)) { // Asegúrate de que haya suficientes bytes disponibles para un float (4 bytes)
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
        Serial.println("  -> Negacion: Instruccion no invertible. Ejecutando normalmente.");
        performAction(action, i, resistenciaActual);
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

  // Ejecución de la acción basada en el tipo de acción
  switch (action) {
    case AVANZADA:
      Serial.println("AVANZADA (Mover motor hacia adelante)");
      moveMotor(500); // Ejemplo: mover 500 pasos
      break;
    case RETROCESO:
      Serial.println("RETROCESO (Mover motor hacia atras)");
      moveMotor(-500); // Ejemplo: mover -500 pasos
      break;
    case GIRAR_IZQUIERDA:
      Serial.println("GIRAR IZQUIERDA (Mover motor para girar izquierda)");
      moveMotor(200); // Ejemplo: mover 200 pasos para girar
      break;
    case GIRAR_DERECHA:
      Serial.println("GIRAR DERECHA (Mover motor para girar derecha)");
      moveMotor(-200); // Ejemplo: mover -200 pasos para girar
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
}

// Obtiene la acción invertida para una acción dada.
ActionType getInvertedAction(ActionType originalAction) {
  // esta función solo será llamada con acciones invertibles.
  // Por lo tanto, los casos para NEGACION, BLOQUE_CONTROL, MELODIA_1, MELODIA_2  son redundantes aquí.
  switch (originalAction) {
    case AVANZADA: return RETROCESO;
    case RETROCESO: return AVANZADA;
    case GIRAR_IZQUIERDA: return GIRAR_DERECHA;
    case GIRAR_DERECHA: return GIRAR_IZQUIERDA;
    default:
      return originalAction; 
                             
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

// --- FUNCIONES DE ACCIÓN (PLACEHOLDERS) ---
// DEBES IMPLEMENTAR LA LÓGICA REAL PARA CADA UNA DE ESTAS FUNCIONES EN TU TESIS.

void moveMotor(int steps) {
  Serial.print("    Moviendo motor: ");
  Serial.print(steps);
  Serial.println(" pasos.");
  motor.step(steps); // Mueve el motor el número de pasos especificado
}

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
