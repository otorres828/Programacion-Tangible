// ---------------------------------------------------------------------------------------
// Código para Medidor de 4 Resistencias con Arduino Nano
// Utiliza A0 para medir Vin y A1, A2, A3, A6 para medir Vout de cada divisor.
// Muestra los valores de las resistencias en el Monitor Serial.
// Nota: Utilizamos A4 y A5 para la comunicacion IC2 con el arduino central.
// ---------------------------------------------------------------------------------------

#include <Wire.h> // Librería para comunicación I2C


// Dirección I2C ÚNICA para cada Arduino de columna.
#define I2C_SLAVE_ADDRESS 0x01 // <---  COLUMNA 1

// --- DEFINICIONES DE PINES Y VALORES DE REFERENCIA ---

// Pin para medir el voltaje de entrada (Vin) del divisor. Conectar a 3V3 del Arduino.
#define inputVoltaje A0

// Pines para medir el voltaje de salida (Vout) de cada divisor.
// Conectar cada uno al punto medio de su respectivo divisor (entre R1 y Rx).
#define dividerInput1 A1
#define dividerInput2 A2
#define dividerInput3 A3
#define dividerInput4 A6

// Valor de la resistencia de referencia conocida (R1) en Ohmios (Ω).
#define RC 1000  // Resistencia de Referencia = 1 kOhmio (1000 Ohms)

const float ADC_REFERENCE_VOLTAGE = 3.3; //voltaje del arduino
float measuredResistances[4]; // Array para guardar Rx1, Rx2, Rx3, Rx4

// Pines digitales para los LEDs de cada resistencia
#define ledPin1 2 // D2 para Rx1
#define ledPin2 3 // D3 para Rx2
#define ledPin3 4 // D4 para Rx3
#define ledPin4 5 // D5 para Rx4

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
// Esto es necesario porque Wire.write() y Wire.read() operan con bytes.
union FloatBytes {
  float f;
  byte b[4]; // Un float ocupa 4 bytes
};


void setup() {

  Serial.begin(9600);

  // Inicia la comunicación I2C como ESCLAVO con la dirección asignada.
  Wire.begin(I2C_SLAVE_ADDRESS);
  // Registra la función que se llamará cuando el Maestro solicite datos.
  Wire.onRequest(requestEvent);

  // Configurar los pines de los LEDs como SALIDA
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);

  // Asegurarse de que los LEDs estén apagados al inicio
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);

  // Mensajes iniciales informativos en el Monitor Serial.
  Serial.println("----------------------------------");
  Serial.println("   Medidor de 4 Resistencias Activo ");
  Serial.println("----------------------------------");
  Serial.print("Voltaje de referencia ADC: ");
  Serial.print(ADC_REFERENCE_VOLTAGE, 1);
  Serial.println(" V");
  Serial.print("Resistencia de referencia (R1): ");
  Serial.print(RC);
  Serial.println(" Ohmios");

}

//Convierte el valor analogico en voltaje
float getVoltage(int analogPin){
  int bits = analogRead(analogPin);
  // delay(100); // Un pequeño delay para estabilización, pero 100ms es mucho si se repite 10 veces.
  float dividerVoltaje = (bits * ADC_REFERENCE_VOLTAGE) /1023; //0-1023
  return dividerVoltaje;
}

//Obtiene un promedio del voltaje obtenido
float getAverageVoltage(int analogPinToMeasure) {
  float sumVoltage = 0;
  for (int i = 0; i < 10; i++) {
    sumVoltage += getVoltage(analogPinToMeasure);
  }
  return sumVoltage / 10.0;
}

//Obtiene el valor de la resustencia
float getResistanceValue(float v2, float v1) {

  // Manejo de casos especiales para evitar divisiones por cero o resultados erróneos.
  if (v1 < 0.01) { // Si Vin es muy bajo, la fuente no está conectada o hay un error.
    return -1.0; // Código de error: Vin inválido
  }

  if ((v1 - v2) < 0.001) { // Si (Vin - Vout) es casi cero, significa que Vout ~ Vin,
                           // lo que implica que Rx es muy pequeña o un cortocircuito.
    return -2.0; // Código de error: Corto Circuito / Valor Inválido
  }

  if (v2 < 0.001) { // Si Vout es casi cero, Rx es extremadamente grande (circuito abierto).
    return -999.0; // Código de error: Circuito Abierto
  }

  // Fórmula para calcular Rx: Rx = R1 * (Vout / (Vin - Vout))
  float resistance = (v2 * RC) / (v1 - v2);

  if(resistance > 60000.0) { // Si la resistencia calculada es mayor a 60kOhm, se considera un error.
    return -3.0; // Código de error: Corto Circuito / Valor Inválido
  }

  return resistance;
}

//Imprime el valor de las resistencias
void printResistance(float resistance, int rxNumber,int ledPin) {

  Serial.print("Rx");
  Serial.print(rxNumber);
  Serial.print(": ");

  // Apagar el LED al inicio de cada evaluación
  digitalWrite(ledPin, LOW);

  if (resistance == -999.0) {
    Serial.println("ABIERTO / Muy Alta");
  } else if (resistance == -1.0) {
    Serial.println("ERROR - Vin no detectado");
  } else if (resistance == -2.0 || resistance < -3.0 || resistance < 0) { // Incluye negativos por ruido
    Serial.println("CORTO / Valor Invalido");
  } else {

      // Valor válido, encender el LED
      digitalWrite(ledPin, HIGH);

      // Formatear la resistencia para mostrar en Ohms, kOhms o MOhms
      if (resistance >= 1000.0) { // Mayor o igual a 1 kOhms
        Serial.print(resistance / 1000.0, 2);
        Serial.println(" kOhms");
      } else { // Menor de 1 kOhms
        Serial.print(resistance, 2);
        Serial.println(" Ohms");
      }
    
  }
}

// Esta función se llama automáticamente cuando el Arduino Maestro solicita datos.
void requestEvent() {
  // Convertir cada float en el array a 4 bytes y enviarlos.
  // El Maestro espera 4 floats * 4 bytes/float = 16 bytes.
  for (int i = 0; i < 4; i++) {
    FloatBytes fb;

    //nos aseguramos que solo mandemos isntancias válidas
    // Si la resistencia medida es válida (mayor que 0), enviamos su valor
    if(measuredResistances[i]>0){
      fb.f = measuredResistances[i];
    }else{
      // Si es -999.0, -1.0, -2.0 o -3.0,  Enviamos -1 como valor de error si la resistencia no es válida  
      fb.f = -1; 
    }
    Wire.write(fb.b, 4); // Envía los 4 bytes de cada float
  }
}

void loop() {

  Serial.println("--------------------------------");
  float avgInputVoltage = getAverageVoltage(inputVoltaje);

  // Asegurarse de que los LEDs estén apagados antes de la nueva ronda de mediciones,
  // para que solo se enciendan si la lectura es válida en esta iteración.
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);


  // Medir y calcular cada resistencia Rx, y controlar su LED correspondiente.
  float avgDividerVoltage_Rx1 = getAverageVoltage(dividerInput1);
  measuredResistances[0] = getResistanceValue(avgDividerVoltage_Rx1, avgInputVoltage);
  printResistance(measuredResistances[0], 1, ledPin1);

  float avgDividerVoltage_Rx2 = getAverageVoltage(dividerInput2);
  measuredResistances[1] = getResistanceValue(avgDividerVoltage_Rx2, avgInputVoltage);
  printResistance(measuredResistances[1], 2, ledPin2);

  float avgDividerVoltage_Rx3 = getAverageVoltage(dividerInput3);
  measuredResistances[2] = getResistanceValue(avgDividerVoltage_Rx3, avgInputVoltage);
  printResistance(measuredResistances[2], 3, ledPin3);

  float avgDividerVoltage_Rx4 = getAverageVoltage(dividerInput4);
  measuredResistances[3] = getResistanceValue(avgDividerVoltage_Rx4, avgInputVoltage);
  printResistance(measuredResistances[3], 4, ledPin4);


  delay(1000);
}