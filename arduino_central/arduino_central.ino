// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C)
// Solicita datos de 4 Arduinos de columna (Esclavos I2C) y consolida los valores
// de resistencia en un array de 16 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion IC2 con cada columna .
// ---------------------------------------------------------------------------------------

#include <Wire.h> // Librería para comunicación I2C
// #include <Servo.h> // Comentado, ya que se pidió no usar servos por ahora.

#define BOTON_INICIO 2 // Pin del botón (con resistencia pull-up)

// Direcciones I2C de los Arduinos de las columnas
const int COLUMN_ADDRESSES[] = {0x01, 0x02, 0x03, 0x04};
const int NUM_COLUMNS = 4;
const int RESISTANCES_PER_COLUMN = 4;
const int TOTAL_RESISTANCES = NUM_COLUMNS * RESISTANCES_PER_COLUMN; // 4 columnas * 4 resistencias = 16

float allResistances[TOTAL_RESISTANCES]; // Array para almacenar todas las resistencias leídas (16 posiciones)
float instruccionesColumnas[12];         // Array para almacenar las instrucciones de 3 columnas (12 posiciones)
float bloqueControl[4];                  // Array para almacenar el bloque de control (4 posiciones)

// Variables de control de secuencia 
bool secuenciaLista = false;

// Servo servoX, servoY; 

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
// Esto es necesario porque Wire.write() y Wire.read() operan con bytes.
// Un float (4 bytes) se descompone en 4 bytes individuales para la transmisión I2C.
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- FUNCIÓN setup() ---
// Se ejecuta una sola vez al encender o reiniciar el Arduino.
void setup() {
  Wire.begin(); // Inicia la comunicación I2C como Maestro
  Serial.begin(4800); // Para comunicación con el Monitor Serial del PC

  pinMode(BOTON_INICIO, INPUT_PULLUP); // Configura el pin del botón con resistencia pull-up interna

  // servoX.attach(9); // Comentado: Adjunta el servo al pin 9
  // servoY.attach(10); // Comentado: Adjunta el servo al pin 10

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
    copiarArrays(); // Copia los valores leídos a los arrays de instrucciones y bloque de control
    Serial.println("Instrucciones cargadas. Esperando botón...");
    // Imprimir todos los valores leídos para verificación en el Monitor Serial
    Serial.println("Valores de todas las resistencias leídas:");
    for (int i = 0; i < TOTAL_RESISTANCES; i++) {
      Serial.print("Resistencia ");
      Serial.print(i + 1);
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
  if (/*digitalRead(BOTON_INICIO) == LOW &&*/ !secuenciaLista) {
    secuenciaLista = true; // Marca que la secuencia está lista para ejecutarse
    ejecutarSecuencia();   // Llama a la función que contiene la lógica de tu tesis
    secuenciaLista = false; // Permite reiniciar la secuencia después de completarla
    Serial.println("Secuencia completada.");
  }
}

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
        allResistances[globalIndex] = -9999.0; // Valor de error para indicar fallo en la lectura I2C
      }
    }
    delay(50); // Pequeña pausa entre solicitudes a diferentes esclavos para evitar saturación
  }
}

// Copiar los valores después de leer todas las columnas
void copiarArrays() {
  // Copia las primeras 12 posiciones
  for (int i = 0; i < 12; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  // Copia las últimas 4 posiciones
  for (int i = 0; i < 4; i++) {
    bloqueControl[i] = allResistances[12 + i];
  }
}

//ejecuta una instruccion determinada
void ejecutarInstruccion(float resistenciaActual){
    Serial.println(resistenciaActual);
}


// Ejecuta las instrucciones del array 'instruccionesColumnas'
void ejecutarSecuencia() {

  Serial.println("Iniciando ejecucion de instrucciones...");

  for (int i = 0; i < 12; i++) { // Recorre las primeras 3 columnas (12 resistencias si las hubiese)
    float resistenciaActual = instruccionesColumnas[i];

    Serial.print("Procesando Resistencia ");
    Serial.print(i + 1); // Número de resistencia (1 a 16)
    Serial.print(" (Columna ");
    Serial.print((i / RESISTANCES_PER_COLUMN) + 1); // Calcula la columna
    Serial.print(", Rx ");
    Serial.print((i % RESISTANCES_PER_COLUMN) + 1); // Calcula el Rx dentro de la columna
    Serial.print("): ");


    // Imprime el valor de la resistencia actual
    if (resistenciaActual >= 1000.0) {
      Serial.print(resistenciaActual / 1000.0, 2);
      Serial.println(" kOhms");
    } else {
      Serial.print(resistenciaActual, 2);
      Serial.println(" Ohms");
    }

    //ejecutamos cada instruccion solo si es valida
    if(resistenciaActual>0){
      ejecutarInstruccion(resistenciaActual); 
    }

    delay(500); // Pequeña pausa entre el procesamiento de cada resistencia
  }

}
