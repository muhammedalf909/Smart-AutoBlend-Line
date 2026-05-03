#include <AccelStepper.h>
// ArduinoJson library removed to save Flash/RAM (JSON is built manually)
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h> // CRITICAL: Ninja protocol for local hostname resolution
#include <Preferences.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// --- Hardware Mapping ---
#define RELAY_PUMP1 18 // Blue Liquid
#define RELAY_PUMP2 19 // Yellow Liquid
#define RELAY_PUMP3 21 // Final Green Liquid (Dispense)
#define STEPPER_MIXER_STEP 25
#define STEPPER_MIXER_DIR 26
#define STEPPER_CONVEYOR_STEP 14
#define STEPPER_CONVEYOR_DIR 27
#define STEPPER_ENABLE_PIN                                                     \
  22 // CRITICAL FIX: Moved to safe GPIO to prevent MTDI boot-loop
#define TRIG_PIN 5
#define ECHO_PIN 17
#define IR_SENSOR 34 // Cup detection
#define LED_GREEN 2  // System Ready/Running
#define LED_RED 4    // Error/Low Level/Stopped

// --- Macros ---
#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define DRIVER_ENABLED LOW
#define DRIVER_DISABLED HIGH
#define WDT_TIMEOUT 10

// --- Parameters ---
const float TARGET_EMPTY_CM = 25.0; // Level indicating tank needs refill
const float TARGET_FULL_CM = 5.0;   // Level indicating tank is full
const unsigned long MIX_DURATION = 30000;
const unsigned long DISPENSE_TIMER = 4500; // Early shutoff for gravity drip
const unsigned long CONVEYOR_CENTER_DELAY = 200; // Delay to center cup
const unsigned long FILL_TIMEOUT = 60000;

// --- WiFi & Network Credentials ---
const char *ap_ssid = "Mastermind_Line";
const char *ap_password = "12345678";
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);

// --- Steppers ---
AccelStepper mixerStepper(AccelStepper::DRIVER, STEPPER_MIXER_STEP,
                          STEPPER_MIXER_DIR);
AccelStepper conveyorStepper(AccelStepper::DRIVER, STEPPER_CONVEYOR_STEP,
                             STEPPER_CONVEYOR_DIR);

// --- State Machine ---
enum SystemState {
  STATE_IDLE,
  STATE_CHECK_LEVEL,
  STATE_FILLING,
  STATE_MIXING,
  STATE_CONVEYING,
  STATE_DISPENSING,
  STATE_ERROR
};

SystemState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// System Data
int cupCounter = 0;
float currentDistance = 0.0;
unsigned long lastUsPingTime = 0;

// Smart Cup Indexing State Variables
int indexPhase = 0;
unsigned long indexTimer = 0;

// Mixer Phase Tracking
bool mixerRampingDown = false;
unsigned long mixerRampDownStartTime = 0;

Preferences preferences;

// HTML Dashboard
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Liquid Mixing Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; text-align: center; margin:0; padding:20px; overflow-x: hidden; }
    .card { background-color: #1e1e1e; padding: 20px; border-radius: 10px; display: inline-block; width: 90%; max-width: 400px; box-shadow: 0px 4px 10px rgba(0,0,0,0.5); position: relative; z-index: 1; transition: opacity 0.3s; }
    h1 { color: #00d2ff; }
    button { width: 80%; padding: 15px; font-size: 1.2em; margin: 10px auto; border: none; border-radius: 5px; cursor: pointer; color: white; font-weight: bold; transition: transform 0.2s, background-color 0.3s; display: block; }
    button:active { transform: scale(0.95); }
    .btn-start { background-color: #28a745; }
    .btn-start:hover { background-color: #218838; }
    .btn-stop { background-color: #dc3545; }
    .btn-stop:hover { background-color: #c82333; }
    .btn-details { background-color: #007bff; }
    .btn-details:hover { background-color: #0069d9; }
    
    /* Bottom Sheet */
    .bottom-sheet { position: fixed; bottom: -100%; left: 0; width: 100%; background-color: #2c2c2c; border-top-left-radius: 20px; border-top-right-radius: 20px; padding: 20px; box-sizing: border-box; transition: bottom 0.4s ease-in-out; z-index: 10; box-shadow: 0px -5px 15px rgba(0,0,0,0.5); text-align: left; }
    .bottom-sheet.open { bottom: 0; }
    .close-btn { position: absolute; top: 15px; right: 20px; background: none; border: none; color: #aaaaaa; font-size: 1.5em; cursor: pointer; width: auto; padding: 0; margin: 0; }
    .close-btn:hover { color: #ffffff; }
    .sheet-title { color: #00d2ff; margin-top: 0; margin-bottom: 20px; font-size: 1.5em; text-align: center; }
    .data-row { display: flex; justify-content: space-between; margin-bottom: 15px; font-size: 1.2em; border-bottom: 1px solid #444; padding-bottom: 5px; }
    .data-label { color: #cccccc; }
    .data-val { color: #00d2ff; font-weight: bold; }
    
    /* Overlay */
    .overlay { position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.6); opacity: 0; pointer-events: none; transition: opacity 0.4s; z-index: 5; }
    .overlay.open { opacity: 1; pointer-events: auto; }
  </style>
</head>
<body>
  <div class="overlay" id="overlay" onclick="closeDetails()"></div>

  <div class="card" id="mainCard">
    <h1>Mixing Line</h1>
    <button class="btn-start" onclick="sendCommand('start')">START</button>
    <button class="btn-stop" onclick="sendCommand('stop')">E-STOP</button>
    <button class="btn-details" onclick="openDetails()">FULL DETAILS</button>
  </div>

  <div class="bottom-sheet" id="bottomSheet">
    <button class="close-btn" onclick="closeDetails()">&#x2715;</button>
    <h2 class="sheet-title">System Details</h2>
    <div class="data-row"><span class="data-label">State</span><span class="data-val" id="state">--</span></div>
    <div class="data-row"><span class="data-label">Total Cups</span><span class="data-val" id="cups">0</span></div>
    <div class="data-row"><span class="data-label">Tank Level</span><span class="data-val"><span id="level">0.0</span> cm</span></div>
    <div class="data-row"><span class="data-label">Run Time</span><span class="data-val" id="runTime">0s</span></div>
    <div class="data-row"><span class="data-label">Avg Cup Time</span><span class="data-val" id="avgTime">0.0s</span></div>
  </div>

  <script>
    let running = false;
    let runStartTime = 0;
    let totalRunTime = 0;
    let currentCups = 0;

    function openDetails() {
      document.getElementById('bottomSheet').classList.add('open');
      document.getElementById('overlay').classList.add('open');
      document.getElementById('mainCard').style.opacity = '0.3';
    }
    
    function closeDetails() {
      document.getElementById('bottomSheet').classList.remove('open');
      document.getElementById('overlay').classList.remove('open');
      document.getElementById('mainCard').style.opacity = '1';
    }

    setInterval(function() {
      fetch('/status').then(res => res.json()).then(data => {
        document.getElementById('state').innerText = data.state;
        document.getElementById('cups').innerText = data.cups;
        document.getElementById('level').innerText = data.level.toFixed(1);
        
        currentCups = data.cups;

        if (data.state !== 'IDLE' && data.state !== 'ERROR') {
          if (!running) {
            running = true;
            runStartTime = Date.now() - (totalRunTime * 1000);
          }
          totalRunTime = Math.floor((Date.now() - runStartTime) / 1000);
        } else {
          running = false;
        }

        document.getElementById('runTime').innerText = totalRunTime + 's';
        
        if (currentCups > 0) {
          let avg = (totalRunTime / currentCups).toFixed(1);
          document.getElementById('avgTime').innerText = avg + 's';
        } else {
          document.getElementById('avgTime').innerText = '0.0s';
        }
      }).catch(err => console.log(err));
    }, 1000);

    function sendCommand(cmd) {
      fetch('/' + cmd);
    }
  </script>
</body>
</html>
)rawliteral";

// Function Prototypes
void stopAllOutputs();
float readUltrasonic();
void setState(SystemState newState);

void setup() {
  Serial.begin(115200);

  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, DRIVER_DISABLED);

  // CRITICAL: Boot Glitch Prevention
  // Write HIGH to the relay pins BEFORE setting them as OUTPUT
  // This prevents brief misfires during boot since the relays are active-LOW.
  digitalWrite(RELAY_PUMP1, RELAY_OFF);
  pinMode(RELAY_PUMP1, OUTPUT);
  digitalWrite(RELAY_PUMP2, RELAY_OFF);
  pinMode(RELAY_PUMP2, OUTPUT);
  digitalWrite(RELAY_PUMP3, RELAY_OFF);
  pinMode(RELAY_PUMP3, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_SENSOR, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // Stepper Setup
  mixerStepper.setMaxSpeed(3000.0);
  mixerStepper.setAcceleration(500.0); // Smooth acceleration to prevent whip

  conveyorStepper.setMaxSpeed(800.0);
  conveyorStepper.setAcceleration(400.0);

  // ==========================================
  // Standalone Industrial Control Unit Setup (Anti-Brownout Mode)
  // ==========================================
  WiFi.mode(WIFI_AP);
  
  // CRITICAL FIX: Limit WiFi TX power to flatten the inrush current spike and prevent silent freezing
  WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  
  WiFi.setSleep(false); // Keep radio awake to prevent Windows/Chrome dropped TCP packets
  WiFi.softAPConfig(local_ip, gateway, subnet);
  // param: SSID, Password, Channel, Hidden(0), Max connections(4)
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);

  Serial.print("Mastermind AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start mDNS Responder
  if (!MDNS.begin("mastermind")) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println(
        "mDNS started! Access Dashboard at: http://mastermind.local");
  }
  // ==========================================

  // Async Web Server Endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"state\":\"";
    switch (currentState) {
    case STATE_IDLE:
      json += "IDLE";
      break;
    case STATE_CHECK_LEVEL:
      json += "CHECK_LEVEL";
      break;
    case STATE_FILLING:
      json += "FILLING";
      break;
    case STATE_MIXING:
      json += "MIXING";
      break;
    case STATE_CONVEYING:
      json += "CONVEYING";
      break;
    case STATE_DISPENSING:
      json += "DISPENSING";
      break;
    case STATE_ERROR:
      json += "ERROR";
      break;
    }
    json += "\",\"cups\":";
    json += String(cupCounter);
    json += ",\"level\":";
    json += String(currentDistance);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentState == STATE_IDLE || currentState == STATE_ERROR) {
      setState(STATE_CHECK_LEVEL);
    }
    request->send(200, "text/plain", "Started");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    setState(STATE_IDLE);
    request->send(200, "text/plain", "Stopped");
  });

  server.begin();

#if defined(CONFIG_IDF_TARGET_ESP32)
  // ESP-IDF v5+ Compatible WDT Configuration
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000, // Convert seconds to milliseconds
      .idle_core_mask =
          (1 << portNUM_PROCESSORS) - 1, // Monitor all available cores
      .trigger_panic = true              // Panic/Reset on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Subscribe current task to WDT
#endif

  preferences.begin("machine_data", false);
  cupCounter = preferences.getInt("cups", 0);

  // ==========================================
  // MASTERMIND HARD AUTO-START FALLBACK
  // ==========================================
  Serial.println("Boot Complete. Auto-Starting mechanism in 3 seconds...");
  delay(3000); // CRITICAL: Safety delay to clear hands from moving parts
  setState(STATE_CHECK_LEVEL); // Force system to bypass IDLE and start production
}

void loop() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  esp_task_wdt_reset();
#endif

  // Stepper Execution Rule: Both MUST continuously call run()
  mixerStepper.run();
  conveyorStepper.run();

  // Non-blocking Ultrasonic Ping
  if (millis() - lastUsPingTime > 500) {
    currentDistance = readUltrasonic();
    lastUsPingTime = millis();
  }

  // State Machine Logic
  switch (currentState) {
  case STATE_IDLE:
    // Awaiting start command via Web
    break;

  case STATE_CHECK_LEVEL:
    if (currentDistance > TARGET_EMPTY_CM) {
      setState(STATE_FILLING);
    } else {
      setState(STATE_CONVEYING);
    }
    break;

  case STATE_FILLING:
    digitalWrite(RELAY_PUMP1, RELAY_ON);
    digitalWrite(RELAY_PUMP2, RELAY_ON);

    if (currentDistance <= TARGET_FULL_CM && currentDistance > 2.0) {
      digitalWrite(RELAY_PUMP1, RELAY_OFF);
      digitalWrite(RELAY_PUMP2, RELAY_OFF);
      setState(STATE_MIXING);
    }

    if (millis() - stateStartTime > FILL_TIMEOUT) {
      setState(STATE_ERROR);
    }
    break;

  case STATE_MIXING:
    if (!mixerRampingDown) {
      // Massive relative distance to spin continuously
      if (mixerStepper.distanceToGo() == 0) {
        mixerStepper.move(1000000);
      }

      if (millis() - stateStartTime > MIX_DURATION) {
        mixerStepper.stop(); // Ramps down safely using acceleration parameters
        mixerRampingDown = true;
        mixerRampDownStartTime = millis();
      }
    } else {
      // Wait for stepper to physically stop + 2 seconds for liquid inertia
      if (mixerStepper.distanceToGo() == 0 &&
          (millis() - mixerRampDownStartTime > 2000)) {
        setState(STATE_CONVEYING);
      }
    }
    break;

  case STATE_CONVEYING:
    /* Quirks Smart Indexing:
       1) Start Conveyor
       2) Wait IR == HIGH (passed previous cup)
       3) Wait IR == LOW (new cup arrived)
       4) Non-blocking Center Delay while still moving
       5) Ramp down
    */
    if (indexPhase == 0) {
      conveyorStepper.move(1000000); // Start moving indefinitely
      if (digitalRead(IR_SENSOR) == HIGH) {
        indexPhase = 1; // Passed previous cup or empty space
      }
    } else if (indexPhase == 1) {
      if (digitalRead(IR_SENSOR) == LOW) {
        indexPhase = 2; // New cup edge detected
        indexTimer = millis();
      }
    } else if (indexPhase == 2) {
      if (millis() - indexTimer > CONVEYOR_CENTER_DELAY) {
        conveyorStepper.stop(); // Ramp down to stop perfectly under nozzle
        indexPhase = 3;
      }
    } else if (indexPhase == 3) {
      if (conveyorStepper.distanceToGo() == 0) {
        setState(STATE_DISPENSING);
      }
    }
    break;

  case STATE_DISPENSING:
    digitalWrite(RELAY_PUMP3, RELAY_ON);

    // Early shutoff to let remaining liquid drop naturally (Gravity Drip
    // Offset)
    if (millis() - stateStartTime > DISPENSE_TIMER) {
      digitalWrite(RELAY_PUMP3, RELAY_OFF);
      cupCounter++;
      preferences.putInt("cups", cupCounter);
      setState(STATE_CHECK_LEVEL);
    }
    break;

  case STATE_ERROR:
    // Halt logic already executed by setState
    break;
  }

  // LED Update Logic
  if (currentState == STATE_ERROR) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else if (currentState == STATE_IDLE) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, (millis() / 500) % 2); // Blink non-blocking
  } else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
}

void setState(SystemState newState) {
  if (newState == STATE_IDLE || newState == STATE_ERROR) {
    digitalWrite(STEPPER_ENABLE_PIN, DRIVER_DISABLED);
  } else {
    digitalWrite(STEPPER_ENABLE_PIN, DRIVER_ENABLED);
    delayMicroseconds(
        5); // Critical: Hardware stabilization delay before first step
  }

  currentState = newState;
  stateStartTime = millis();

  // Stop outputs by default on critical state changes
  if (newState == STATE_IDLE || newState == STATE_ERROR) {
    stopAllOutputs();
  }

  // Initialize state specific tracking variables
  if (newState == STATE_MIXING) {
    mixerRampingDown = false;
  }
  if (newState == STATE_CONVEYING) {
    indexPhase = 0;
  }
}

void stopAllOutputs() {
  digitalWrite(RELAY_PUMP1, RELAY_OFF);
  digitalWrite(RELAY_PUMP2, RELAY_OFF);
  digitalWrite(RELAY_PUMP3, RELAY_OFF);

  // Non-blocking stop for steppers
  mixerStepper.stop();
  conveyorStepper.stop();
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Strict 20000us timeout prevents blocking main loop
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  if (duration == 0)
    return 999.0;
  return (float)duration * 0.0343 / 2.0;
}
