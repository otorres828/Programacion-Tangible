// Prueba de botón: inicio/pausa/reanudar/reinicio
const int BOTON_INICIO = 2;

enum SystemState { ESTADO_LEER, ESTADO_CORRER, ESTADO_PAUSA, ESTADO_REINICIO };
SystemState estadoSistemaActual = ESTADO_LEER;

unsigned long lastButtonStateChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool lastButtonReading = HIGH;
bool currentButtonReading;
bool longPressTriggered = false;

const unsigned long BUTTON_DEBOUNCE_DELAY = 50; // ms
const unsigned long LONG_PRESS_THRESHOLD = 3000; // ms (3s)

void setup() {
  Serial.begin(9600);
  pinMode(BOTON_INICIO, INPUT_PULLUP);
  Serial.println("Prueba boton: listo. Usa el boton conectado a pin 2 (INPUT_PULLUP).");
  Serial.println("Pulsacion corta: INICIO / PAUSA / REANUDAR. Pulsacion larga (3s): REINICIO.");
}

void loop() {
  currentButtonReading = digitalRead(BOTON_INICIO);

  // Debounce: detectar cambio de estado
  if (currentButtonReading != lastButtonReading) {
    lastButtonStateChangeTime = millis();
  }

  if ((millis() - lastButtonStateChangeTime) > BUTTON_DEBOUNCE_DELAY) {
    // Flanco de bajada (presionado)
    if (currentButtonReading == LOW && lastButtonReading == HIGH) {
      buttonPressStartTime = millis();
      longPressTriggered = false;
    }
    // Flanco de subida (liberado)
    else if (currentButtonReading == HIGH && lastButtonReading == LOW) {
      if (!longPressTriggered) {
        // Pulsación corta: alterna estados INICIO/PAUSA/REANUDAR
        if (estadoSistemaActual == ESTADO_LEER) {
          Serial.println("Boton: INICIO de secuencia.");
          estadoSistemaActual = ESTADO_CORRER;
        } else if (estadoSistemaActual == ESTADO_CORRER) {
          Serial.println("Boton: PAUSA de secuencia.");
          estadoSistemaActual = ESTADO_PAUSA;
        } else if (estadoSistemaActual == ESTADO_PAUSA) {
          Serial.println("Boton: REANUDAR secuencia.");
          estadoSistemaActual = ESTADO_CORRER;
        } else if (estadoSistemaActual == ESTADO_REINICIO) {
          Serial.println("Estado: ya en REINICIO. Volviendo a LEER.");
          estadoSistemaActual = ESTADO_LEER;
        }
      }
    }
  }

  // Detectar pulsación larga mientras se mantiene presionado
  if (currentButtonReading == LOW && !longPressTriggered) {
    if ((millis() - buttonPressStartTime) >= LONG_PRESS_THRESHOLD) {
      Serial.println("Boton: REINICIO COMPLETO (pulsacion larga).");
      longPressTriggered = true;
      estadoSistemaActual = ESTADO_REINICIO;
      // Comportamiento de reinicio: regresar a estado de lectura después de un pequeño retardo
      delay(200);
      estadoSistemaActual = ESTADO_LEER;
      Serial.println("Estado: REINICIO completado. Volviendo a LEER.");
    }
  }

  lastButtonReading = currentButtonReading;
  delay(10); // pequeña pausa para evitar saturar el loop
}
