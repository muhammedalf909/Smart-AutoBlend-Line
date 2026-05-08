#include <AccelStepper.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// --- Hardware Mapping ---
#define RELAY_PUMP1 21 // Blue Liquid
#define RELAY_PUMP2 19 // Yellow Liquid
#define RELAY_PUMP3 18 // Final Green Liquid (Dispense)

#define STEPPER_MIXER_STEP 25
#define STEPPER_MIXER_DIR 26
#define STEPPER_CONVEYOR_STEP 14
#define STEPPER_CONVEYOR_DIR 27

#define STEPPER_ENABLE_PIN 22
#define TRIG_PIN 5
#define ECHO_PIN 17
#define IR_SENSOR 34 // Cup detection

#define LED_GREEN 2 // System Ready/Running
#define LED_RED 4   // Error/Low Level/Stopped

// --- Macros ---
#define RELAY_ON LOW
#define RELAY_OFF HIGH
#define DRIVER_ENABLED LOW
#define DRIVER_DISABLED HIGH
#define WDT_TIMEOUT 10

// --- Parameters ---
const float TARGET_EMPTY_CM = 22.0;  // Used for UI progress bar math
const float TARGET_FULL_CM = 11.0;   // Stop filling point
const float REFILL_THRESHOLD_CM = 17.0; // [Zatona] Hysteresis: Only start filling if level drops below 17cm

const unsigned long MIX_DURATION = 30000;
const unsigned long DISPENSE_TIMER = 5000;       // [Zatona] 5 seconds dispense
const unsigned long CONVEYOR_CENTER_DELAY = 200; // Delay to center cup
const unsigned long FILL_SAFETY_TIMER = 125000;  // 125 seconds
const unsigned long CLEAR_CUP_TIMER = 1500;      // [Zatona] Blind time: 1.5s to ensure cup clears IR sensor
const unsigned long US_PING_INTERVAL = 600;      // [Zatona] 5 pings * 600ms = 3000ms

// --- Network & WebSockets ---
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastBroadcastTime = 0;

// --- FreeRTOS & Dual Core ---
TaskHandle_t Core0TaskHandle;

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
  STATE_CLEAR_CUP,
  STATE_ERROR,
  STATE_PAUSED
};

volatile SystemState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// System Data
volatile int cupCounter = 0;
volatile float currentDistance = 999.0;
unsigned long lastUsPingTime = 0;
int indexPhase = 0;
unsigned long indexTimer = 0;
bool mixerRampingDown = false;
unsigned long mixerRampDownStartTime = 0;
int ultrasonicDebounceCount = 0;
volatile bool newPing = false;
volatile bool cmdStart = false;
volatile bool cmdStop = false;
bool conveyorRampingDown = false;
volatile SystemState previousState = STATE_IDLE;
volatile unsigned long timeSpentInState = 0;
volatile bool cmdPause = false;
volatile bool cmdContinue = false;

Preferences preferences;

// HTML Dashboard (Cyberpunk UI/UX)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Mastermind SCADA</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
      background-color: #121212; 
      color: #ffffff; 
      text-align: center; 
      margin:0; 
      padding:20px; 
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }
    .card { 
      background: rgba(255, 255, 255, 0.05); 
      backdrop-filter: blur(10px);
      -webkit-backdrop-filter: blur(10px);
      border: 1px solid rgba(255, 255, 255, 0.1);
      padding: 30px; 
      border-radius: 15px; 
      width: 90%; 
      max-width: 450px; 
      box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37); 
      margin-top: 20px;
    }
    h1 { 
      color: #00f3ff; 
      text-shadow: 0 0 10px rgba(0,243,255,0.5);
      margin-bottom: 30px;
      letter-spacing: 2px;
      text-transform: uppercase;
    }
    .status-box {
      margin-bottom: 20px;
    }
    .status-text {
      font-size: 1.5em;
      font-weight: bold;
      color: #00f3ff;
      text-shadow: 0 0 8px rgba(0,243,255,0.8);
    }
    .counter {
      font-size: 3em;
      font-weight: bold;
      color: #0077ff;
      margin: 15px 0;
      text-shadow: 0 0 15px rgba(0,119,255,0.5);
    }
    .label {
      color: #aaaaaa;
      font-size: 0.9em;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    .progress-container {
      width: 100%;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 10px;
      height: 30px;
      margin-top: 10px;
      position: relative;
      overflow: hidden;
      border: 1px solid rgba(255, 255, 255, 0.2);
    }
    .progress-bar {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, #0077ff, #00f3ff);
      transition: width 0.3s ease;
      box-shadow: 0 0 15px rgba(0, 243, 255, 0.6);
    }
    .progress-text {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      font-weight: bold;
      color: #fff;
      text-shadow: 1px 1px 2px #000;
      pointer-events: none;
    }
    .ctrl-btn { 
      width: 100%; 
      padding: 20px; 
      font-size: 1.5em; 
      margin-top: 30px; 
      border: none; 
      border-radius: 10px; 
      cursor: pointer; 
      color: white; 
      font-weight: bold; 
      transition: transform 0.1s, background 0.3s, box-shadow 0.3s; 
      letter-spacing: 2px;
    }
    .ctrl-btn:active { transform: scale(0.95); }
    
    .btn-start { 
      background: linear-gradient(45deg, #00f3ff, #00ff88); 
      box-shadow: 0 0 20px rgba(0,255,136,0.4);
    }
    
    .btn-stop { 
      background: linear-gradient(45deg, #ff0055, #ff0000); 
      box-shadow: 0 0 20px rgba(255,0,0,0.4);
    }
    .btn-pause { 
      background: linear-gradient(45deg, #00bfff, #0077ff); 
      box-shadow: 0 0 20px rgba(0, 191, 255, 0.4); 
    }
    .btn-continue { 
      background: linear-gradient(45deg, #ffffff, #e0e5ec); 
      color: #121212; 
      box-shadow: 0 0 20px rgba(255, 255, 255, 0.6); 
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>Mastermind SCADA</h1>
    
    <div class="status-box">
      <div class="label">System State</div>
      <div class="status-text" id="state">CONNECTING...</div>
    </div>
    
    <div>
      <div class="label">Cup Counter</div>
      <div class="counter" id="cups">0</div>
    </div>
    
    <div style="margin-top: 20px;">
      <div class="label">Tank Level Progress</div>
      <div class="progress-container">
        <div class="progress-bar" id="levelBar"></div>
        <div class="progress-text"><span id="level">0.0</span> cm</div>
      </div>
    </div>

    <button id="mainBtn" class="ctrl-btn btn-start" onclick="toggleState()">START SEQUENCE</button>
    <button id="pauseBtn" class="ctrl-btn btn-pause" onclick="togglePause()" style="display:none;">PAUSE</button>
  </div>

  <script>
    let ws;
    function connect() {
      ws = new WebSocket('ws://' + window.location.hostname + ':81/');
      
      ws.onmessage = function(event) {
        try {
          let data = JSON.parse(event.data);
          let currentState = data.state;
          
          document.getElementById('state').innerText = data.state;
          document.getElementById('cups').innerText = data.cups;
          document.getElementById('level').innerText = data.level.toFixed(1);
          
          let stateEl = document.getElementById('state');
          if(data.state === 'ERROR') {
            stateEl.style.color = '#ff0055';
            stateEl.style.textShadow = '0 0 8px rgba(255,0,85,0.8)';
          } else if(data.state === 'IDLE') {
            stateEl.style.color = '#aaaaaa';
            stateEl.style.textShadow = 'none';
          } else {
            stateEl.style.color = '#00f3ff';
            stateEl.style.textShadow = '0 0 8px rgba(0,243,255,0.8)';
          }

          let emptyCm = 22.0;
          let fullCm = 11.0;
          let p = 0;
          if(data.level <= fullCm) p = 100;
          else if(data.level >= emptyCm) p = 0;
          else {
            p = 100 - ((data.level - fullCm) / (emptyCm - fullCm)) * 100;
          }
          document.getElementById('levelBar').style.width = p + '%';

          let btn = document.getElementById('mainBtn');
          if (data.state === 'IDLE') {
            btn.className = 'ctrl-btn btn-start';
            btn.innerText = 'START SEQUENCE';
          } else {
            btn.className = 'ctrl-btn btn-stop';
            btn.innerText = 'E-STOP';
          }

          let pauseBtn = document.getElementById('pauseBtn');
          if (data.state === 'IDLE' || data.state === 'ERROR') {
              pauseBtn.style.display = 'none';
          } else {
              pauseBtn.style.display = 'block';
              if (data.state === 'PAUSED') {
                  pauseBtn.className = 'ctrl-btn btn-continue';
                  pauseBtn.innerText = 'CONTINUE';
              } else {
                  pauseBtn.className = 'ctrl-btn btn-pause';
                  pauseBtn.innerText = 'PAUSE';
              }
          }
        } catch (e) {
          console.error("Parse error", e);
        }
      };
      
      ws.onclose = function() {
        document.getElementById('state').innerText = 'DISCONNECTED';
        setTimeout(connect, 1000);
      };
    }
    
    connect();

    function toggleState() {
      let btnText = document.getElementById('mainBtn').innerText;
      if(btnText === 'START SEQUENCE') {
        ws.send("START");
      } else {
        ws.send("STOP");
      }
    }

    function togglePause() {
        let btnText = document.getElementById('pauseBtn').innerText;
        if(btnText === 'PAUSE') {
            ws.send("PAUSE");
        } else {
            ws.send("CONTINUE");
        }
    }
  </script>
</body>
</html>
)rawliteral";

// Function Prototypes
void stopAllOutputs();
float readUltrasonic();
void setState(SystemState newState);
void resumeState(SystemState stateToResume);
void handleRoot();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void core0TaskCode(void *pvParameters);

void handleRoot() { server.send_P(200, "text/html", index_html); }

void core0TaskCode(void *pvParameters) {
  for (;;) {
    server.handleClient();
    webSocket.loop();

    if (millis() - lastBroadcastTime > 200) {
      char jsonBuffer[128];
      const char *stateStr = "IDLE";
      switch (currentState) {
      case STATE_IDLE:
        stateStr = "IDLE";
        break;
      case STATE_CHECK_LEVEL:
        stateStr = "CHECK_LEVEL";
        break;
      case STATE_FILLING:
        stateStr = "FILLING";
        break;
      case STATE_MIXING:
        stateStr = "MIXING";
        break;
      case STATE_CONVEYING:
        stateStr = "CONVEYING";
        break;
      case STATE_DISPENSING:
        stateStr = "DISPENSING";
        break;
      case STATE_CLEAR_CUP:
        stateStr = "CLEAR_CUP";
        break;
      case STATE_ERROR:
        stateStr = "ERROR";
        break;
      case STATE_PAUSED:
        stateStr = "PAUSED";
        break;
      }
      snprintf(jsonBuffer, sizeof(jsonBuffer),
               "{\"state\":\"%s\",\"cups\":%d,\"level\":%.1f}", stateStr,
               cupCounter, currentDistance);

      webSocket.broadcastTXT(jsonBuffer);
      lastBroadcastTime = millis();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, DRIVER_DISABLED);

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

  // [Ninja] Modified Dynamics: Mixer speed lowered for stability, Conveyor setup for max torque & snapping
  mixerStepper.setMaxSpeed(4000.0);
  mixerStepper.setAcceleration(3000.0);

  conveyorStepper.setMaxSpeed(1000.0);
  conveyorStepper.setAcceleration(4000.0);
  conveyorStepper.setPinsInverted(true, false, false);

  // ==========================================
  // Industrial LAN Setup: Air-Gapped AP Mode
  // ==========================================
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Mastermind_Line", "12345678", 1, 0, 4);
  WiFi.setTxPower(WIFI_POWER_15dBm);

  Serial.println("\nAP Started. SSID: Mastermind_Line");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  if (MDNS.begin("mastermind")) {
    Serial.println("mDNS responder started: mastermind.local");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

#if defined(CONFIG_IDF_TARGET_ESP32)
  esp_task_wdt_config_t wdt_config = {.timeout_ms = WDT_TIMEOUT * 1000,
                                      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
                                      .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
#endif

  // [Zatona] Mastermind Fix: Protected NVS flash lifespan
  preferences.begin("machine_data", false);
  // preferences.putInt("cups", 0); // UNCOMMENT ONLY ONCE TO RESET, THEN RE-COMMENT TO PROTECT FLASH MEMORY
  cupCounter = preferences.getInt("cups", 0);

  xTaskCreatePinnedToCore(core0TaskCode, "Core0Task", 10000, NULL, 1,
                          &Core0TaskHandle, 0);

  setState(STATE_IDLE);
}

void loop() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  esp_task_wdt_reset();
#endif

  if (cmdStart) {
    cmdStart = false;
    if (currentState == STATE_IDLE || currentState == STATE_ERROR) {
      setState(STATE_CHECK_LEVEL);
    }
  }

  if (cmdStop) {
    cmdStop = false;
    setState(STATE_IDLE);
  }

  if (cmdPause) {
    cmdPause = false;
    if (currentState != STATE_IDLE && currentState != STATE_ERROR &&
        currentState != STATE_PAUSED) {
      timeSpentInState = millis() - stateStartTime;
      previousState = currentState;
      setState(STATE_PAUSED);
    }
  }

  if (cmdContinue) {
    cmdContinue = false;
    if (currentState == STATE_PAUSED) {
      resumeState(previousState);
      stateStartTime = millis() - timeSpentInState; // Restore exact timer safely
    }
  }

  mixerStepper.run();
  conveyorStepper.run();

  // [Zatona] US_PING_INTERVAL is now 600ms. 5 pings = exactly 3 seconds for strict debounce.
  if (millis() - lastUsPingTime > US_PING_INTERVAL) {
    currentDistance = readUltrasonic();
    lastUsPingTime = millis();
    newPing = true;
  }

  switch (currentState) {
  case STATE_IDLE:
    break;

  case STATE_CHECK_LEVEL:
    // [Zatona] Mastermind Fix: If distance is >= 17cm OR it's 999.0 (timeout/boot), FORCE filling to prevent dry runs.
    if (currentDistance >= REFILL_THRESHOLD_CM || currentDistance == 999.0) {
      setState(STATE_FILLING);
    } else {
      setState(STATE_CONVEYING);
    }
    break;

  case STATE_FILLING:
    digitalWrite(RELAY_PUMP1, RELAY_ON);
    digitalWrite(RELAY_PUMP2, RELAY_ON);

    if (newPing) {
      newPing = false;
      if (currentDistance <= TARGET_FULL_CM) {
        ultrasonicDebounceCount++;
        if (ultrasonicDebounceCount >= 5) {
          ultrasonicDebounceCount = 0;
          digitalWrite(RELAY_PUMP1, RELAY_OFF);
          digitalWrite(RELAY_PUMP2, RELAY_OFF);
          setState(STATE_MIXING);
        }
      } else if (currentDistance > TARGET_FULL_CM && currentDistance != 999.0) {
        // Only reset debounce if we explicitly read a valid distance ABOVE the target
        ultrasonicDebounceCount = 0;
      }
    }

    if (millis() - stateStartTime > FILL_SAFETY_TIMER) {
      // Safety override: Move to mixing even if target not reached
      ultrasonicDebounceCount = 0;
      digitalWrite(RELAY_PUMP1, RELAY_OFF);
      digitalWrite(RELAY_PUMP2, RELAY_OFF);
      setState(STATE_MIXING);
    }
    break;

  case STATE_MIXING:
    if (!mixerRampingDown) {
      if (mixerStepper.distanceToGo() == 0) {
        mixerStepper.move(1000000);
      }
      if (millis() - stateStartTime > MIX_DURATION) {
        mixerStepper.stop();
        mixerRampingDown = true;
        mixerRampDownStartTime = millis();
      }
    } else {
      if (mixerStepper.distanceToGo() == 0 &&
          (millis() - mixerRampDownStartTime > 2000)) {
        setState(STATE_CONVEYING);
      }
    }
    break;

  case STATE_CONVEYING:
    // [Zatona] Mastermind Fix: Ensure the motor gets movement commands in BOTH seeking phases
    // This prevents Deadlock if PAUSE clears the target distance during indexPhase 1
    if (indexPhase == 0 || indexPhase == 1) {
      if (conveyorStepper.distanceToGo() == 0) {
        conveyorStepper.move(1000000);
      }
    }

    if (indexPhase == 0) {
      if (digitalRead(IR_SENSOR) == HIGH) {
        indexPhase = 1;
      }
    } else if (indexPhase == 1) {
      if (digitalRead(IR_SENSOR) == LOW) {
        indexPhase = 2;
        indexTimer = millis();
      }
    } else if (indexPhase == 2) {
      if (millis() - indexTimer > CONVEYOR_CENTER_DELAY) {
        conveyorStepper.stop();
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
    if (millis() - stateStartTime > DISPENSE_TIMER) {
      digitalWrite(RELAY_PUMP3, RELAY_OFF);
      cupCounter++;
      preferences.putInt("cups", cupCounter);
      setState(STATE_CLEAR_CUP);
    }
    break;

  case STATE_CLEAR_CUP:
    // [Zatona] This phase acts as the blind time. The IR sensor is completely ignored here.
    if (!conveyorRampingDown) {
      if (conveyorStepper.distanceToGo() == 0) {
        conveyorStepper.move(1000000);
      }
      if (millis() - stateStartTime > CLEAR_CUP_TIMER) {
        conveyorStepper.stop();
        conveyorRampingDown = true;
      }
    } else {
      if (conveyorStepper.distanceToGo() == 0) {
        setState(STATE_CHECK_LEVEL);
      }
    }
    break;

  case STATE_ERROR:
    break;

  case STATE_PAUSED:
    break;
  }

  if (currentState == STATE_ERROR) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else if (currentState == STATE_IDLE) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, (millis() / 500) % 2);
  } else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
}

void setState(SystemState newState) {
  // Only enable steppers when physical movement is required to prevent thermal overload
  if (newState == STATE_MIXING || newState == STATE_CONVEYING ||
      newState == STATE_CLEAR_CUP) {
    digitalWrite(STEPPER_ENABLE_PIN, DRIVER_ENABLED);
    delayMicroseconds(5);
  } else {
    digitalWrite(STEPPER_ENABLE_PIN, DRIVER_DISABLED);
  }

  currentState = newState;
  stateStartTime = millis();

  if (newState == STATE_IDLE || newState == STATE_ERROR ||
      newState == STATE_PAUSED) {
    stopAllOutputs();
  }

  if (newState == STATE_MIXING) {
    mixerRampingDown = false;
  }

  if (newState == STATE_CONVEYING) {
    indexPhase = 0;
  }

  if (newState == STATE_FILLING) {
    ultrasonicDebounceCount = 0;
  }

  if (newState == STATE_CONVEYING || newState == STATE_CLEAR_CUP) {
    conveyorRampingDown = false;
  }
}

void resumeState(SystemState stateToResume) {
  // 1. Re-enable steppers ONLY if the resumed state requires physical movement
  if (stateToResume == STATE_MIXING || stateToResume == STATE_CONVEYING ||
      stateToResume == STATE_CLEAR_CUP) {
    digitalWrite(STEPPER_ENABLE_PIN, DRIVER_ENABLED);
    delayMicroseconds(5);
  }

  // 2. Safely restore the state pointer WITHOUT resetting indexPhase, debounce
  // count, or ramp flags.
  currentState = stateToResume;
}

void stopAllOutputs() {
  digitalWrite(RELAY_PUMP1, RELAY_OFF);
  digitalWrite(RELAY_PUMP2, RELAY_OFF);
  digitalWrite(RELAY_PUMP3, RELAY_OFF);
  mixerStepper.stop();
  conveyorStepper.stop();
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 4000);
  if (duration == 0)
    return 999.0;
  return (float)duration * 0.0343 / 2.0;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    break;
  case WStype_CONNECTED:
    break;
  case WStype_TEXT:
    if (length == 5 && memcmp(payload, "START", 5) == 0) {
      cmdStart = true;
    } else if (length == 4 && memcmp(payload, "STOP", 4) == 0) {
      cmdStop = true;
    } else if (length == 5 && memcmp(payload, "PAUSE", 5) == 0) {
      cmdPause = true;
    } else if (length == 8 && memcmp(payload, "CONTINUE", 8) == 0) {
      cmdContinue = true;
    }
    break;
  }
}
