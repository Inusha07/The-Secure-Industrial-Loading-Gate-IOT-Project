/**
 * Scenario 1 - The Secure Industrial Loading Gate
 *
 * Hardware:
 * - ESP32
 * - PIR motion sensor
 * - HC-SR04 ultrasonic sensor
 * - Potentiometer
 * - Servo motor
 * - Buzzer
 * - I2C OLED display
 *
 * Operational modes:
 * - AUTONOMOUS
 * - MANUAL
 * - SAFETY
 *
 * Safety design:
 * If an obstruction closer than 20 cm is detected while the
 * gate is closing, the gate immediately stops and retreats
 * to the fully open position. The system remains in SAFETY
 * mode until the obstruction has been clear for 3 seconds.
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define PIR_PIN             27
#define TRIG_PIN             5
#define ECHO_PIN            18
#define POT_PIN             34
#define SERVO_PIN           13
#define BUZZER_PIN          25

#define OLED_SDA_PIN        21
#define OLED_SCL_PIN        22

// ============================================================
// OLED CONFIGURATION
// ============================================================

#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT        64
#define OLED_RESET           -1
#define OLED_ADDRESS       0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// ============================================================
// SERVO CONFIGURATION
// ============================================================

Servo gateServo;

const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE   = 90;

int currentGateAngle = GATE_CLOSED_ANGLE;
int targetGateAngle  = GATE_CLOSED_ANGLE;

// ============================================================
// SAFETY CONFIGURATION
// ============================================================

const float SAFETY_DISTANCE_CM = 20.0;

const unsigned long SAFETY_CLEAR_TIME = 3000;

// ============================================================
// TIMING CONFIGURATION
// ============================================================

const unsigned long SENSOR_INTERVAL = 100;
const unsigned long DISPLAY_INTERVAL = 250;
const unsigned long SERIAL_INTERVAL = 1000;
const unsigned long BUZZER_INTERVAL = 500;

// ============================================================
// SYSTEM STATES
// ============================================================

enum SystemState
{
  AUTONOMOUS,
  MANUAL,
  SAFETY
};

SystemState currentState = AUTONOMOUS;

// ============================================================
// SENSOR VARIABLES
// ============================================================

bool pirDetected = false;

float distanceCM = 999.0;

int potValue = 0;

int potPercent = 0;

// ============================================================
// GATE VARIABLES
// ============================================================

bool gateOpening = false;
bool gateClosing = false;

unsigned long gateHoldStartTime = 0;

unsigned long lastServoUpdate = 0;

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long lastBuzzerUpdate = 0;

unsigned long obstructionClearStart = 0;

// ============================================================
// USER CONFIGURATION
// ============================================================

/*
 * Potentiometer controls the automatic hold-open delay.
 *
 * Minimum: 2 seconds
 * Maximum: 10 seconds
 */
unsigned long holdOpenDelay = 5000;

// ============================================================
// MANUAL MODE VARIABLES
// ============================================================

bool manualGateOpen = false;

// ============================================================
// BUZZER
// ============================================================

bool buzzerState = false;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void readSensors();
float readUltrasonicDistance();

void processSystem();

void processAutonomousMode();
void processManualMode();
void processSafetyMode();

void updateGateMovement();

void openGate();
void closeGate();
void stopGate();

void enterSafetyMode();

void updateBuzzer();
void buzzerOn();
void buzzerOff();

void updateOLED();

void printSerialStatus();

void processSerialCommands();

void setGateAngle(int angle);

const char* getStateName();


// ============================================================
// SETUP
// ============================================================

/**
 * @brief Initialises GPIO, OLED, servo and serial communication.
 *
 * @return void
 */
void setup()
{
  Serial.begin(115200);

  delay(100);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" Secure Industrial Loading Gate");
  Serial.println(" ESP32 Embedded Control System");
  Serial.println(" Scenario 1");
  Serial.println("========================================");

  // ----------------------------------------------------------
  // GPIO CONFIGURATION
  // ----------------------------------------------------------

  pinMode(PIR_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ----------------------------------------------------------
  // ADC CONFIGURATION
  // ----------------------------------------------------------

  analogReadResolution(12);

  // ----------------------------------------------------------
  // I2C CONFIGURATION
  // ----------------------------------------------------------

  Wire.begin(
    OLED_SDA_PIN,
    OLED_SCL_PIN
  );

  // ----------------------------------------------------------
  // OLED INITIALISATION
  // ----------------------------------------------------------

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS))
  {
    Serial.println("ERROR: OLED initialisation failed!");

    while (true)
    {
      // Fatal hardware error.
      // No normal operation is possible.
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("Industrial Gate");

  display.setCursor(0, 15);
  display.println("System Starting...");

  display.display();

  // ----------------------------------------------------------
  // SERVO INITIALISATION
  // ----------------------------------------------------------

  gateServo.setPeriodHertz(50);

  gateServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  setGateAngle(GATE_CLOSED_ANGLE);

  currentState = AUTONOMOUS;

  Serial.println();
  Serial.println("System ready.");
  Serial.println();
  Serial.println("UART COMMANDS:");
  Serial.println("A = Autonomous mode");
  Serial.println("M = Manual mode");
  Serial.println("O = Open gate");
  Serial.println("C = Close gate");
  Serial.println("S = Stop gate");
  Serial.println("R = Safety reset");
  Serial.println("? = Show commands");
  Serial.println();

  Serial.println("Starting in AUTONOMOUS mode.");

  delay(1000);
}


// ============================================================
// MAIN LOOP
// ============================================================

/**
 * @brief Main non-blocking control loop.
 *
 * The system is implemented using elapsed-time scheduling
 * instead of delay-based control.
 *
 * @return void
 */
void loop()
{
  unsigned long currentTime = millis();

  // ----------------------------------------------------------
  // PERIODIC SENSOR SAMPLING
  // ----------------------------------------------------------

  if (currentTime - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = currentTime;

    readSensors();
  }

  // ----------------------------------------------------------
  // SERIAL COMMAND PROCESSING
  // ----------------------------------------------------------

  processSerialCommands();

  // ----------------------------------------------------------
  // SYSTEM STATE PROCESSING
  // ----------------------------------------------------------

  processSystem();

  // ----------------------------------------------------------
  // SERVO MOVEMENT
  // ----------------------------------------------------------

  updateGateMovement();

  // ----------------------------------------------------------
  // BUZZER
  // ----------------------------------------------------------

  updateBuzzer();

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL)
  {
    lastDisplayUpdate = currentTime;

    updateOLED();
  }

  // ----------------------------------------------------------
  // SERIAL STATUS
  // ----------------------------------------------------------

  if (currentTime - lastSerialUpdate >= SERIAL_INTERVAL)
  {
    lastSerialUpdate = currentTime;

    printSerialStatus();
  }
}


// ============================================================
// SENSOR FUNCTIONS
// ============================================================

/**
 * @brief Reads all sensors used by the loading gate.
 *
 * Reads the PIR sensor, potentiometer and ultrasonic distance.
 *
 * @return void
 */
void readSensors()
{
  // PIR motion detection
  pirDetected = digitalRead(PIR_PIN);

  // Potentiometer ADC
  potValue = analogRead(POT_PIN);

  // Convert ADC value to percentage
  potPercent = map(
    potValue,
    0,
    4095,
    0,
    100
  );

  // Convert potentiometer to hold-open delay.
  //
  // 0%   = 2 seconds
  // 100% = 10 seconds
  holdOpenDelay = map(
    potValue,
    0,
    4095,
    2000,
    10000
  );

  // Ultrasonic sensor
  distanceCM = readUltrasonicDistance();
}


/**
 * @brief Measures distance using the HC-SR04 ultrasonic sensor.
 *
 * @return Distance in centimetres.
 */
float readUltrasonicDistance()
{
  /*
   * Trigger pulse.
   */
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(TRIG_PIN, HIGH);

  /*
   * Short trigger pulse.
   *
   * This is a hardware pulse rather than a system delay.
   */
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  /*
   * Read echo pulse.
   *
   * Timeout prevents the system from waiting indefinitely
   * if the sensor does not return an echo.
   */
  unsigned long duration =
    pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration == 0)
  {
    return 999.0;
  }

  /*
   * Speed of sound:
   *
   * distance = time * speed / 2
   *
   * Using 0.0343 cm/us.
   */
  float distance =
    (duration * 0.0343) / 2.0;

  return distance;
}


// ============================================================
// SYSTEM STATE PROCESSING
// ============================================================

/**
 * @brief Processes the currently selected operational state.
 *
 * @return void
 */
void processSystem()
{
  switch (currentState)
  {
    case AUTONOMOUS:
      processAutonomousMode();
      break;

    case MANUAL:
      processManualMode();
      break;

    case SAFETY:
      processSafetyMode();
      break;
  }
}


// ============================================================
// AUTONOMOUS MODE
// ============================================================

/**
 * @brief Executes autonomous gate operation.
 *
 * PIR motion opens the gate.
 * After motion disappears, the configured hold-open delay
 * starts. The gate closes only when the path is clear.
 *
 * @return void
 */
void processAutonomousMode()
{
  /*
   * Vehicle/person detected.
   */
  if (pirDetected)
  {
    openGate();

    gateHoldStartTime = millis();

    return;
  }

  /*
   * If the gate is currently open and PIR is no longer
   * detecting motion, start/continue the hold timer.
   */
  if (currentGateAngle >= GATE_OPEN_ANGLE)
  {
    if (gateHoldStartTime == 0)
    {
      gateHoldStartTime = millis();
    }

    /*
     * Close only after the configured hold-open time.
     */
    if (millis() - gateHoldStartTime >= holdOpenDelay)
    {
      /*
       * The ultrasonic sensor must show a clear path.
       */
      if (distanceCM >= SAFETY_DISTANCE_CM)
      {
        closeGate();
      }
      else
      {
        /*
         * Obstruction exists.
         * Keep gate open.
         */
        openGate();
      }
    }
  }

  /*
   * If PIR becomes active while closing,
   * reopen the gate.
   */
  if (gateClosing && pirDetected)
  {
    openGate();
  }

  /*
   * Safety condition:
   *
   * Only activate safety logic when the gate is physically
   * closing.
   */
  if (gateClosing &&
      distanceCM < SAFETY_DISTANCE_CM)
  {
    enterSafetyMode();
  }
}


// ============================================================
// MANUAL MODE
// ============================================================

/**
 * @brief Executes manual gate control mode.
 *
 * Manual mode is controlled through the UART serial terminal.
 *
 * @return void
 */
void processManualMode()
{
  /*
   * Safety monitoring remains active in manual mode.
   *
   * If a worker is closing the gate manually and an
   * obstruction appears, safety mode takes priority.
   */
  if (gateClosing &&
      distanceCM < SAFETY_DISTANCE_CM)
  {
    enterSafetyMode();
  }
}


// ============================================================
// SAFETY MODE
// ============================================================

/**
 * @brief Executes the safety recovery routine.
 *
 * When an obstruction is detected:
 * 1. Stop normal closing operation.
 * 2. Retreat the gate to the fully open position.
 * 3. Wait until the obstruction is removed.
 * 4. Require the path to remain clear for 3 seconds.
 * 5. Return automatically to Autonomous mode.
 *
 * @return void
 */
void processSafetyMode()
{
  /*
   * Safety priority:
   * Keep the gate open.
   */
  openGate();

  /*
   * Obstruction still present.
   */
  if (distanceCM < SAFETY_DISTANCE_CM)
  {
    obstructionClearStart = 0;

    return;
  }

  /*
   * Path has become clear.
   */
  if (obstructionClearStart == 0)
  {
    obstructionClearStart = millis();
  }

  /*
   * Require clear path for 3 seconds.
   */
  if (millis() - obstructionClearStart >= SAFETY_CLEAR_TIME)
  {
    obstructionClearStart = 0;

    currentState = AUTONOMOUS;

    gateHoldStartTime = millis();

    Serial.println();
    Serial.println("SAFETY CLEARED.");
    Serial.println("Returning to AUTONOMOUS mode.");
  }
}


// ============================================================
// GATE CONTROL
// ============================================================

/**
 * @brief Commands the gate to open.
 *
 * @return void
 */
void openGate()
{
  targetGateAngle = GATE_OPEN_ANGLE;

  gateOpening = true;
  gateClosing = false;
}


/**
 * @brief Commands the gate to close.
 *
 * @return void
 */
void closeGate()
{
  targetGateAngle = GATE_CLOSED_ANGLE;

  gateOpening = false;
  gateClosing = true;
}


/**
 * @brief Stops gate movement at the current position.
 *
 * @return void
 */
void stopGate()
{
  targetGateAngle = currentGateAngle;

  gateOpening = false;
  gateClosing = false;
}


/**
 * @brief Updates the servo position without using delay().
 *
 * Servo movement is calculated from elapsed time and the
 * potentiometer-derived closing speed.
 *
 * @return void
 */
void updateGateMovement()
{
  unsigned long now = millis();

  /*
   * Update servo approximately every 50 ms.
   */
  if (now - lastServoUpdate < 50)
  {
    return;
  }

  lastServoUpdate = now;

  /*
   * Calculate movement speed from potentiometer.
   *
   * Potentiometer controls the gate closing speed:
   *
   * Low value  = slower
   * High value = faster
   *
   * 1 to 8 degrees per update.
   */
  int stepSize = map(
    potValue,
    0,
    4095,
    1,
    8
  );

  /*
   * Opening.
   */
  if (currentGateAngle < targetGateAngle)
  {
    currentGateAngle += stepSize;

    if (currentGateAngle > targetGateAngle)
    {
      currentGateAngle = targetGateAngle;
    }

    gateServo.write(currentGateAngle);
  }

  /*
   * Closing.
   */
  else if (currentGateAngle > targetGateAngle)
  {
    currentGateAngle -= stepSize;

    if (currentGateAngle < targetGateAngle)
    {
      currentGateAngle = targetGateAngle;
    }

    gateServo.write(currentGateAngle);
  }

  /*
   * Stop movement when target reached.
   */
  if (currentGateAngle == targetGateAngle)
  {
    gateOpening = false;
    gateClosing = false;
  }
}


/**
 * @brief Writes an angle to the servo and updates the internal state.
 *
 * @param angle Desired servo angle.
 * @return void
 */
void setGateAngle(int angle)
{
  currentGateAngle = constrain(
    angle,
    GATE_CLOSED_ANGLE,
    GATE_OPEN_ANGLE
  );

  targetGateAngle = currentGateAngle;

  gateServo.write(currentGateAngle);
}


// ============================================================
// SAFETY CONTROL
// ============================================================

/**
 * @brief Enters the safety state after detecting an obstruction.
 *
 * The gate immediately stops its normal closing command and
 * the recovery routine moves it back toward the open position.
 *
 * @return void
 */
void enterSafetyMode()
{
  if (currentState == SAFETY)
  {
    return;
  }

  currentState = SAFETY;

  obstructionClearStart = 0;

  Serial.println();
  Serial.println("!!! SAFETY BLOCK !!!");
  Serial.println("Obstruction detected below 20 cm.");
  Serial.println("Gate retreating to OPEN position.");

  /*
   * Immediately command the gate open.
   */
  openGate();
}


// ============================================================
// BUZZER
// ============================================================

/**
 * @brief Updates the buzzer without blocking the main loop.
 *
 * The buzzer periodically chirps while the system is in
 * safety mode.
 *
 * @return void
 */
void updateBuzzer()
{
  if (currentState != SAFETY)
  {
    buzzerOff();

    return;
  }

  unsigned long now = millis();

  if (now - lastBuzzerUpdate >= BUZZER_INTERVAL)
  {
    lastBuzzerUpdate = now;

    buzzerState = !buzzerState;

    if (buzzerState)
    {
      buzzerOn();
    }
    else
    {
      buzzerOff();
    }
  }
}


/**
 * @brief Activates the warning buzzer.
 *
 * @return void
 */
void buzzerOn()
{
  tone(BUZZER_PIN, 2200);
}


/**
 * @brief Deactivates the warning buzzer.
 *
 * @return void
 */
void buzzerOff()
{
  noTone(BUZZER_PIN);

  buzzerState = false;
}


// ============================================================
// OLED DISPLAY
// ============================================================

/**
 * @brief Updates the OLED with live system information.
 *
 * @return void
 */
void updateOLED()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // ----------------------------------------------------------
  // Line 1 - System state
  // ----------------------------------------------------------

  display.setCursor(0, 0);

  display.print("MODE: ");
  display.println(getStateName());

  // ----------------------------------------------------------
  // Line 2 - PIR
  // ----------------------------------------------------------

  display.setCursor(0, 10);

  display.print("PIR: ");

  if (pirDetected)
  {
    display.println("MOTION");
  }
  else
  {
    display.println("CLEAR");
  }

  // ----------------------------------------------------------
  // Line 3 - Ultrasonic
  // ----------------------------------------------------------

  display.setCursor(0, 20);

  display.print("DIST: ");

  if (distanceCM >= 999)
  {
    display.println("---");
  }
  else
  {
    display.print(distanceCM, 1);
    display.println("cm");
  }

  // ----------------------------------------------------------
  // Line 4 - Potentiometer
  // ----------------------------------------------------------

  display.setCursor(0, 30);

  display.print("POT: ");
  display.print(potPercent);
  display.println("%");

  // ----------------------------------------------------------
  // Line 5 - Gate position
  // ----------------------------------------------------------

  display.setCursor(0, 40);

  display.print("GATE: ");
  display.print(currentGateAngle);
  display.println(" deg");

  // ----------------------------------------------------------
  // Line 6 - Safety / timing information
  // ----------------------------------------------------------

  display.setCursor(0, 50);

  if (currentState == SAFETY)
  {
    display.println("!! SAFETY BLOCK !!");
  }
  else
  {
    display.print("HOLD: ");
    display.print(holdOpenDelay / 1000);
    display.println("s");
  }

  display.display();
}


// ============================================================
// SERIAL STATUS
// ============================================================

/**
 * @brief Prints live system information to the UART terminal.
 *
 * @return void
 */
void printSerialStatus()
{
  Serial.println("----------------------------------------");

  Serial.print("MODE: ");
  Serial.println(getStateName());

  Serial.print("PIR: ");
  Serial.println(
    pirDetected ? "MOTION" : "CLEAR"
  );

  Serial.print("Distance: ");
  Serial.print(distanceCM, 1);
  Serial.println(" cm");

  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print(" / 4095 (");
  Serial.print(potPercent);
  Serial.println("%)");

  Serial.print("Hold-open delay: ");
  Serial.print(holdOpenDelay / 1000);
  Serial.println(" seconds");

  Serial.print("Gate angle: ");
  Serial.print(currentGateAngle);
  Serial.println(" degrees");

  Serial.print("Target angle: ");
  Serial.print(targetGateAngle);
  Serial.println(" degrees");

  Serial.println("----------------------------------------");
}


// ============================================================
// SERIAL COMMANDS
// ============================================================

/**
 * @brief Processes commands received from the UART terminal.
 *
 * Commands:
 * A = Autonomous mode
 * M = Manual mode
 * O = Open gate
 * C = Close gate
 * S = Stop gate
 * R = Safety reset
 * ? = Display commands
 *
 * @return void
 */
void processSerialCommands()
{
  if (!Serial.available())
  {
    return;
  }

  char command = Serial.read();

  /*
   * Convert lowercase to uppercase.
   */
  if (command >= 'a' && command <= 'z')
  {
    command -= 32;
  }

  switch (command)
  {
    // --------------------------------------------------------
    // AUTONOMOUS MODE
    // --------------------------------------------------------

    case 'A':

      currentState = AUTONOMOUS;

      gateHoldStartTime = millis();

      Serial.println("Mode changed to AUTONOMOUS.");

      break;


    // --------------------------------------------------------
    // MANUAL MODE
    // --------------------------------------------------------

    case 'M':

      currentState = MANUAL;

      stopGate();

      Serial.println("Mode changed to MANUAL.");

      Serial.println(
        "Use O = Open, C = Close, S = Stop."
      );

      break;


    // --------------------------------------------------------
    // OPEN
    // --------------------------------------------------------

    case 'O':

      /*
       * Manual command automatically enters Manual mode.
       */
      currentState = MANUAL;

      openGate();

      Serial.println("Manual OPEN command.");

      break;


    // --------------------------------------------------------
    // CLOSE
    // --------------------------------------------------------

    case 'C':

      /*
       * Manual command automatically enters Manual mode.
       */
      currentState = MANUAL;

      closeGate();

      Serial.println("Manual CLOSE command.");

      break;


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    case 'S':

      stopGate();

      Serial.println("Gate STOP command.");

      break;


    // --------------------------------------------------------
    // SAFETY RESET
    // --------------------------------------------------------

    case 'R':

      if (currentState == SAFETY)
      {
        /*
         * Reset is only allowed if the path is clear.
         */
        if (distanceCM >= SAFETY_DISTANCE_CM)
        {
          obstructionClearStart = 0;

          currentState = AUTONOMOUS;

          gateHoldStartTime = millis();

          Serial.println(
            "Safety reset accepted."
          );

          Serial.println(
            "Returning to AUTONOMOUS."
          );
        }
        else
        {
          Serial.println(
            "RESET DENIED: obstruction remains."
          );
        }
      }
      else
      {
        Serial.println(
          "System is not in SAFETY mode."
        );
      }

      break;


    // --------------------------------------------------------
    // HELP
    // --------------------------------------------------------

    case '?':

      Serial.println();
      Serial.println("========== COMMANDS ==========");
      Serial.println("A - Autonomous mode");
      Serial.println("M - Manual mode");
      Serial.println("O - Open gate");
      Serial.println("C - Close gate");
      Serial.println("S - Stop gate");
      Serial.println("R - Safety reset");
      Serial.println("? - Show commands");
      Serial.println("==============================");
      Serial.println();

      break;


    default:

      /*
       * Ignore newline and carriage-return characters.
       */
      if (command != '\n' &&
          command != '\r')
      {
        Serial.println(
          "Unknown command. Type ? for help."
        );
      }

      break;
  }
}


// ============================================================
// STATE NAME
// ============================================================

/**
 * @brief Returns the human-readable name of the current state.
 *
 * @return Pointer to state name string.
 */
const char* getStateName()
{
  switch (currentState)
  {
    case AUTONOMOUS:
      return "AUTO";

    case MANUAL:
      return "MANUAL";

    case SAFETY:
      return "SAFETY";

    default:
      return "UNKNOWN";
  }
}