#include <Wire.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NeoPixel.h>
#include "FluxGarage_RoboEyes.h"
#include "images01.h"

// ================================================================================================
// ADJUSTABLE CONSTANTS & PIN DEFINITIONS
// ================================================================================================
// Push Button
const int BUTTON_PIN = 3;

// --- HC-SR04 Ultrasonic Sensor ---
const int US_TRIG_PIN = 9;
const int US_ECHO_PIN = 10;

// --- Distance zones for navigation ---
// 上一步区：d < 4cm
const float PREV_CM = 4.0;

// 下一步区：9~15cm
const float NEXT_MIN_CM = 9.0;
const float NEXT_MAX_CM = 15.0;

// 防连发（避免一次动作跳很多步）
const unsigned long NAV_COOLDOWN_MS = 1200;

// 用于“边沿触发”：只有进入某个区间才触发一次
enum NavZone { Z_FAR, Z_PREV, Z_NEXT, Z_DEAD };
NavZone lastZone = Z_FAR;
unsigned long lastNavMs = 0;

const int SCREEN_WIDTH = 128; // OLED display width, in pixels
const int SCREEN_HEIGHT = 64; // OLED display height, in pixels
const int OLED_RESET = -1; //   QT-PY / XIAO


// NeoPixel LED Strip
const int LED_PIN = 6;
const int LED_COUNT = 10;

// DFPlayer Mini
static const uint8_t PIN_MP3_TX = 1;
static const uint8_t PIN_MP3_RX = 0;

// MQ-2 Air Quality Sensor
const int MQ2_PIN = A0;
const int MQ2_BAD_THRESHOLD = 700; // <<< ADJUSTABLE: Sensor value that triggers the alarm (0-1023)

// LED Brightness Levels (0-255)
const int BRIGHTNESS_HOME = 40;
const int BRIGHTNESS_COUNTDOWN_3 = 100;
const int BRIGHTNESS_COUNTDOWN_2 = 80;
const int BRIGHTNESS_COUNTDOWN_1 = 60;
const int BRIGHTNESS_FLASH = 100;
const int BRIGHTNESS_ALARM = 100;
const int BRIGHTNESS_RECHECK = 60;

// DFPlayer Audio File Numbers
const int ALARM_SOUND = 1;

// ================================================================================================
// SYSTEM STATES (ENUM)
// ================================================================================================
enum SystemState {
  BOOT_SPLASH,
  HOME_EYES,
  RATING_SCREEN,
  PORTION_SCREEN,
  COUNTDOWN,
  STEP1, STEP2, STEP3, STEP4, STEP5, STEP6, STEP7, STEP8,
  CONGRATS,
  AIR_ALARM,
  AIR_RECHECK
};

// ---------- Function prototypes ----------
void changeState(SystemState newState);
void restoreLEDsForState(SystemState state);
void handleButton();
void handleGesture();
void handleAirQuality();
void renderTextScreen(String text, int size);
void setStripColorBrightness(uint8_t r, uint8_t g, uint8_t b, int brightness);
void flashStrip();

// Unified Step UI (top-centered title + optional image below, no overlap)
void showStepUI(int step, const unsigned char* img);


// ================================================================================================
// GLOBAL OBJECTS
// ================================================================================================
Adafruit_SH1106G display(128, 64, &Wire, -1);
RoboEyes<Adafruit_SH1106G> roboEyes(display);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
DFRobotDFPlayerMini player;

// ================================================================================================
// GLOBAL VARIABLES
// ================================================================================================
SystemState currentState = BOOT_SPLASH;
SystemState previousState = HOME_EYES; // To store state before an alarm

// Non-blocking timers
unsigned long lastStateChangeTime = 0;
unsigned long lastActionTime = 0;
unsigned long lastSerialPrintTime = 0;
unsigned long lastAirQualityCheckTime = 0;
unsigned long lastFlashTime = 0;
unsigned long timerStartTime = 0;

// State variables
int countdownNumber = 3;
int currentStep = 1;
bool buttonPressed = false;
bool gestureLeft = false; // true when a RIGHT->LEFT swipe is detected (APDS9960 DIR_LEFT)
bool gestureRight = false; // true when a LEFT->RIGHT swipe is detected (APDS9960 DIR_RIGHT)
bool flashActive = false;

// ---------- Congrats LED animation ----------
unsigned long congratsLastLedMs = 0;
int congratsLedIndex = 0;
bool bootSoundPlayed = false;

bool congratsEyesBack = false;
// ---------------- STEP4 timer sub-state ----------------
enum Step4Phase { STEP4_SHOW_IMAGE, STEP4_TIMING, STEP4_DONE_WAIT };
Step4Phase step4Phase = STEP4_SHOW_IMAGE;

unsigned long step4StartMs = 0;
const unsigned long STEP4_DURATION_MS = 5700;
bool step4DoneSoundPlayed = false;

// Air quality
int mq2Value = 0;
bool airIsBad = false;
const unsigned long AIR_RECHECK_DURATION_MS = 10000; // 10秒检测窗口
bool airImprovedDuringRecheck = false;               // 10秒内是否曾变好


// ================================================================================================
// SETUP
// ================================================================================================
void setup() {
  Serial.begin(9600);

  // --- OLED Display ---
  if (!display.begin(0x3C, true)) {
    Serial.println(F("SH1106 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100); 
  roboEyes.setAutoblinker(ON, 3, 2); 
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setWidth(40, 40); // byte leftEye, byte rightEye
  roboEyes.setHeight(40, 40); // byte leftEye, byte rightEye
  //roboEyes.setBorderradius(8, 8); // byte leftEye, byte rightEye
  roboEyes.setSpacebetween(20); // int space -> can also be negative

  // --- Push Button ---
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // --- HC-SR04 ---
pinMode(US_TRIG_PIN, OUTPUT);
pinMode(US_ECHO_PIN, INPUT);
digitalWrite(US_TRIG_PIN, LOW);


  // --- HC-SR04 ---
pinMode(US_TRIG_PIN, OUTPUT);
pinMode(US_ECHO_PIN, INPUT);
digitalWrite(US_TRIG_PIN, LOW);


  // --- NeoPixel Strip ---
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(BRIGHTNESS_HOME);
  // --- DFPlayer (UNO R4 WiFi: use Serial1) ---
  Serial1.begin(9600);
  delay(800);

  // IMPORTANT: isACK=false avoids blocking if DFPlayer doesn't respond
  // doReset=true to re-init module on boot
  if (player.begin(Serial1, false, true)) {
    Serial.println("DFPlayer OK (ACK off)");
  } else {
    Serial.println("DFPlayer FAIL");
  }

  player.volume(15); // 0~30
  
  // --- CORRECTED: Start in BOOT_SPLASH state explicitly ---
  currentState = BOOT_SPLASH;
  renderTextScreen("MICO", 3);
  setStripColorBrightness(0, 255, 0, BRIGHTNESS_FLASH);
  // player.play(3);
  lastStateChangeTime = millis(); // Start the timer for the splash screen
}


// ================================================================================================
// MAIN LOOP
// ================================================================================================
void loop() {
  unsigned long currentTime = millis();

  // --- ALWAYS-ACTIVE HANDLERS ---
  handleAirQuality(); // Highest priority: check air and potentially trigger alarm
    
  // Allow button presses in all states except during the splash screen
  if (currentState != BOOT_SPLASH) {
     handleButton();
     handleGesture();
  }

if (Serial.available() > 0) {
  // ProtoPie -> Arduino commands (send as plain text, ideally ending with newline)
  String msg = Serial.readString(); // you can also use readStringUntil('
  msg.trim();
  msg.toUpperCase();

  if (msg == "START") {
    changeState(COUNTDOWN);

  } else if (msg == "RATING") {
    changeState(RATING_SCREEN);

  } else if (msg == "PORTION") {
    changeState(PORTION_SCREEN);

  } else if (msg == "RATING OFF" || msg == "RATING_OFF" ||
             msg == "PORTION OFF" || msg == "PORTION_OFF") {
    changeState(HOME_EYES);
  }
}


  // --- MAIN STATE MACHINE ---
  switch (currentState) {
    case BOOT_SPLASH:
    // play boot sound once (0003)
      if (!bootSoundPlayed) {
      bootSoundPlayed = true;
      player.play(3);   
}
      // CORRECTED: The setup is now done in setup(). Here we just wait for the timer.
      if (currentTime - lastStateChangeTime >= 2000) {
        changeState(HOME_EYES);
      }
      break;

    case HOME_EYES:
      roboEyes.update();
      roboEyes.setSpacebetween(22);
      roboEyes.setWidth(40, 40); // byte leftEye, byte rightEye
      roboEyes.setHeight(40, 40); // byte leftEye, byte rightEye
      roboEyes.setAutoblinker(ON, 3, 2);
      roboEyes.setIdleMode(ON, 2, 2);
      if (buttonPressed) {
        changeState(COUNTDOWN);
      }
      break;

    case RATING_SCREEN:
      // Static text already rendered on entry; keep screen as-is
      break;

    case PORTION_SCREEN:
      // Static text already rendered on entry; keep screen as-is
      break;

    case COUNTDOWN:
      //if (currentTime - lastStateChangeTime < 50) { // First entry
      if (currentTime - lastStateChangeTime < 50) { // First entry
        countdownNumber = 3;
        lastActionTime = currentTime;
        renderTextScreen(String(countdownNumber), 4);
        player.play(7); 
        setStripColorBrightness(0, 255, 0, BRIGHTNESS_COUNTDOWN_3);
      }

      if (currentTime - lastActionTime >= 1000) {
        lastActionTime = currentTime;
        countdownNumber--;
        if (countdownNumber > 0) {
          renderTextScreen(String(countdownNumber), 4);
          if(countdownNumber == 2) setStripColorBrightness(50, 255, 50, BRIGHTNESS_COUNTDOWN_2);
          if(countdownNumber == 1) setStripColorBrightness(144, 239, 144, BRIGHTNESS_COUNTDOWN_1);
        } else {
          currentStep = 1;
          changeState(STEP1);
        }
      }
      break;

    case STEP1:
      //if (millis() - lastStateChangeTime < 50) { // First entry
      if (currentTime - lastStateChangeTime < 200) { // First entry
        showStepUI(1, IMG_STEP1); // Step1 title top-centered + image below
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }
      if (gestureLeft) {
        currentStep = 2;
        flashStrip();
        changeState(STEP2);
        Serial.println("NEXT_STEP2");
      }
      break;

    case STEP2:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(2, IMG_STEP2);
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }

      if (gestureLeft) {
        currentStep = 3;
        flashStrip();
        changeState(STEP3);
        Serial.println("NEXT_STEP3");
      } else if (gestureRight) {
        currentStep = 1;
        flashStrip();
        changeState(STEP1);
        Serial.println("BACK_STEP1");
      }
      break;

    case STEP3:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(3, IMG_STEP3);
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }

      if (gestureLeft) {
        currentStep = 4;
        flashStrip();
        changeState(STEP4);
        Serial.println("NEXT_STEP4");
      } else if (gestureRight) {
        currentStep = 2;
        flashStrip();
        changeState(STEP2);
        Serial.println("BACK_STEP2");
      }
      break;

    case STEP4:
    {
      // ----- 第一次进入 STEP4：显示原来的“STEP4 + 图片” -----
      if (millis() - lastStateChangeTime < 50) { // First entry
        step4Phase = STEP4_SHOW_IMAGE;          // 进STEP4先回到图片界面
        step4DoneSoundPlayed = false;   
        showStepUI(4, IMG_STEP4);
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }

      // Swipe back: STEP4 -> STEP3 (only before starting the timer)
      if (step4Phase == STEP4_SHOW_IMAGE && gestureRight) {
        currentStep = 3;
        flashStrip();
        changeState(STEP3);
        Serial.println("BACK_STEP3");
        break;
      }
      // ----- 如果处于“计时中”，每帧刷新进度条 -----
      if (step4Phase == STEP4_TIMING) {
        unsigned long elapsed = millis() - step4StartMs;

        float progress = (float)elapsed / (float)STEP4_DURATION_MS;
        if (progress > 1.0f) progress = 1.0f;
        Serial.println("TIMER_START");
        showStep4ProgressUI(progress);
                // ----- LED color transition: white -> red -----
        float p = progress;  // 0.0 ~ 1.0

        int r = 255;
        int g = (int)(255 * (1.0f - p));
        int b = (int)(255 * (1.0f - p));

        setStripColorBrightness(r, g, b, BRIGHTNESS_HOME);
        if (elapsed >= STEP4_DURATION_MS) {
          step4Phase = STEP4_DONE_WAIT; 
          Serial.println("TIMER_UP"); // 计时结束，等待下一次按钮进入 STEP5
          if (!step4DoneSoundPlayed) {
          step4DoneSoundPlayed = true;
          player.stop();     // 停掉正在 loop 的 0004
          player.play(6);    // 播放完成音 0005（一次）
  }
        }
        
               // ----- STEP4 done: keep LED red -----
       if (step4Phase == STEP4_DONE_WAIT) {
         setStripColorBrightness(255, 0, 0, BRIGHTNESS_HOME);
       }

      }
      // ----- 按钮逻辑：分三段 -----
      if (buttonPressed) {
        // STEP4里第一次按：从图片切到进度条并开始计时
        if (step4Phase == STEP4_SHOW_IMAGE) {
          step4Phase = STEP4_TIMING;
          step4StartMs = millis();
          showStep4ProgressUI(0.0f); // 立刻显示进度条
          player.loop(4);  
        }

        // 计时结束后再按一次：进入 STEP5
        else if (step4Phase == STEP4_DONE_WAIT) {
          currentStep = 5;
          player.stop(); 
          flashStrip();  
          changeState(STEP5);
          Serial.println("NEXT_STEP5");
        }

        // 如果正在计时，再按按钮：先忽略（不让它跳 step）
      }

      break;
    }
    case STEP5:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(5, IMG_STEP2);
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }

      if (gestureLeft) {
        currentStep = 6;
        flashStrip();
        changeState(STEP6);
        Serial.println("NEXT_STEP6");
      } else if (gestureRight) {
        currentStep = 4;
        flashStrip();
        changeState(STEP4);
        Serial.println("BACK_STEP4");
      }
      break;

    case STEP6:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(6, IMG_STEP3);
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }

      if (gestureLeft) {
        currentStep = 7;
        flashStrip();
        changeState(STEP7);
        Serial.println("NEXT_STEP7");
      } else if (gestureRight) {
        currentStep = 5;
        flashStrip();
        changeState(STEP5);
        Serial.println("BACK_STEP5");
      }
      break;

    case STEP7:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(7, IMG_STEP3); // Title only (top-centered)
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }
      if (gestureLeft) {
        currentStep = 8;
        flashStrip();
        changeState(STEP8);
        Serial.println("NEXT_STEP8");
      } else if (gestureRight) {
        currentStep = 6;
        flashStrip();
        changeState(STEP6);
        Serial.println("BACK_STEP6");
      }
      break;

    case STEP8:
      if (millis() - lastStateChangeTime < 50) { // First entry
        showStepUI(8, IMG_STEP8); // Title only (top-centered)
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
      }
      if (gestureLeft) {
        flashStrip();
        changeState(CONGRATS);
        Serial.println("NEXT_STEP9");
      } else if (gestureRight) {
        currentStep = 7;
        flashStrip();
        changeState(STEP7);
        Serial.println("BACK_STEP7");
      }
      break;
    case CONGRATS: {
      unsigned long elapsed = currentTime - lastStateChangeTime;

      // First entry: show text for the first 2s and start looping audio
      if (elapsed < 50) {
        congratsEyesBack = false;
        renderTextScreen("Congrats!", 2);
        // Keep playing during the whole CONGRATS state
        player.loop(5);
      }

      // After 2 seconds, restore RoboEyes (your 'happyeyes' expression)
      if (elapsed >= 2000 && !congratsEyesBack) {
        congratsEyesBack = true;
        roboEyes.setMood(HAPPY);
        roboEyes.setPosition(DEFAULT);
        roboEyes.update();
      }

      // Total duration: 8 seconds
      if (elapsed >= 8000) {
        player.disableLoop();
        player.stop();
        changeState(HOME_EYES);
      }
      break;
    }
    case AIR_ALARM:
      {
  unsigned long elapsed = currentTime - lastStateChangeTime;

  // first entry: bad air for 3s
  if (elapsed < 50) {
    renderTextScreen("BAD AIR", 2);
    Serial.println("AIR_ALARM");
  }

  // 3s after: angry eyes
  if (elapsed >= 3000) {
    roboEyes.setMood(ANGRY);
    roboEyes.update();
  }

  // red led + allarme and play0002
  if (currentTime - lastFlashTime >= 1000) {
    lastFlashTime = currentTime;
    flashActive = !flashActive;
    if (flashActive) {
      setStripColorBrightness(255, 0, 0, BRIGHTNESS_ALARM);
      player.play(2);
    } else {
      strip.clear();
      strip.show();
    }
  }

  //  改成“按一下按钮进入检测状态”，并且吃掉这次按钮事件，防止回HOME后跳COUNTDOWN
  if (buttonPressed) {
    buttonPressed = false;
    player.stop();              // 停掉报警音（如果正在响）
    changeState(AIR_RECHECK);
    Serial.println("RECHECK");
  }

  break;
}

    case AIR_RECHECK:
     {
  // 第一次进入：初始化检测
  if (currentTime - lastStateChangeTime < 50) {
    timerStartTime = currentTime;
    airImprovedDuringRecheck = false;

    player.stop(); // 检测阶段无音效
    setStripColorBrightness(255, 0, 0, BRIGHTNESS_RECHECK);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 0);
      display.print("Checking Air");
      renderProgressBar(0.0f);
  }

  // 红灯常亮
  setStripColorBrightness(255, 0, 0, BRIGHTNESS_RECHECK);

  // 10秒进度条
  unsigned long elapsed = currentTime - timerStartTime;
  float progress = (float)elapsed / (float)AIR_RECHECK_DURATION_MS;
  if (progress > 1.0f) progress = 1.0f;
  renderProgressBar(progress);

  // 10秒内任何时刻空气变好，就算成功
  if (!airIsBad) {
    airImprovedDuringRecheck = true;
  }

  // 10秒结束：成功回到报警前一步；失败回到报警
  if (elapsed >= AIR_RECHECK_DURATION_MS) {
    buttonPressed = false;
    if (airImprovedDuringRecheck) {
      changeState(previousState);
      Serial.println("BETTER");
    } else {
      changeState(AIR_ALARM);
      Serial.println("BAD");
    }
  }

  break;
}
  }
  
  // --- Non-blocking flash handler ---
  if(flashActive) {
      if(currentTime - lastFlashTime > 100) { // momentary flash
          flashActive = false;
          // Restore previous LED state based on current state
          restoreLEDsForState(currentState);
      }
  }
// ---------- LED celebration during CONGRATS ----------
if (currentState == CONGRATS) {
  if (millis() - congratsLastLedMs > 200) {  // 每 200ms 换一次颜色
    congratsLastLedMs = millis();

    switch (congratsLedIndex) {
      case 0: setStripColorBrightness(255, 0, 0, BRIGHTNESS_HOME); break;
      case 1: setStripColorBrightness(0, 255, 0, BRIGHTNESS_HOME); break;
      case 2: setStripColorBrightness(0, 0, 255, BRIGHTNESS_HOME); break;
      case 3: setStripColorBrightness(255, 255, 0, BRIGHTNESS_HOME); break;
      case 4: setStripColorBrightness(255, 0, 255, BRIGHTNESS_HOME); break;
      case 5: setStripColorBrightness(0, 255, 255, BRIGHTNESS_HOME); break;
    }

    congratsLedIndex++;
    if (congratsLedIndex > 5) congratsLedIndex = 0;
  }
}

  // --- Serial Debug Print ---
  if (currentTime - lastSerialPrintTime > 1000) {
    lastSerialPrintTime = currentTime;
    Serial.print("MQ-2: " + String(mq2Value));
    Serial.println(airIsBad ? " | Status: BAD" : " | Status: OK");
    Serial.println("Current State: " + String(currentState));
  }
  
  buttonPressed = false; // Reset button flag at the end of the loop
  gestureLeft = false;   // Reset gesture flag at the end of the loop
  gestureRight = false;  // Reset gesture flag at the end of the loop
}


// ================================================================================================
// HELPER FUNCTIONS
// ================================================================================================

void changeState(SystemState newState) {
  if (currentState == newState) return;

  // Store previous state if entering an alarm, but not if already in one
  if (newState == AIR_ALARM && currentState != AIR_RECHECK) {
      previousState = currentState;
  }

  currentState = newState;
  lastStateChangeTime = millis();
  flashActive = false; // Reset flash on any state change
  roboEyes.setIdleMode(ON, 2, 2); // Reset mood by default

    /*--- Step transition sound (0001.mp3) ---
  if (newState >= STEP1 && newState <= STEP8) {
    player.play(1);   // 播放 0001.mp3（和你开机用的 player.play(3) 一致）
  } */

  // --- Actions on entering a new state ---
  switch(newState) {
    case HOME_EYES:
        currentStep = 1;
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
        display.clearDisplay();
        roboEyes.setMood(DEFAULT);
        break;
    case RATING_SCREEN:
        // Show rating text pushed from ProtoPie
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
        renderTextScreen("4.9 stars", 2);
        break;

    case PORTION_SCREEN:
        // Show portion text pushed from ProtoPie
        setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
        renderTextScreen("4 Portions", 2);
        break;

    case STEP4:
        // 每次进入 STEP4 都回到“显示图片”的初始阶段
        step4Phase = STEP4_SHOW_IMAGE;
        step4StartMs = 0;
        break;

    case CONGRATS:
        //setStripColorBrightness(0,0,0,0); // Turn off LEDs
        congratsLedIndex = 0;
        congratsLastLedMs = 0;

        break;

    case AIR_ALARM:
        display.clearDisplay();
        break;
    case AIR_RECHECK:
        display.clearDisplay();
        break;
    default:
      // most states handle their own setup
      break;
  }
}

void restoreLEDsForState(SystemState state) {
    switch(state) {
        case HOME_EYES:
        case STEP1: case STEP2: case STEP3: case STEP4: case STEP5: case STEP6: case STEP7: case STEP8:
            setStripColorBrightness(255, 255, 255, BRIGHTNESS_HOME);
            break;
        default:
             strip.clear();
             strip.show();
             break;
    }
}

void handleButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  lastReading = reading;

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState == LOW) {
        buttonPressed = true;
      }
    }
  }
}

// 读取一次超声波距离（cm），失败返回 -1
float readUltrasonicCM() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  // 30ms 超时（约 5m），防止卡住
  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;

  return duration * 0.0343f / 2.0f;
}

// 把距离映射到“区域”
NavZone getZone(float d) {
  if (d <= 0) return Z_FAR;

  // 上一步：d < 5
  if (d < PREV_CM) return Z_PREV;

  // 下一步：9~15
  if (d >= NEXT_MIN_CM && d <= NEXT_MAX_CM) return Z_NEXT;

  // 空档：5~9
  if (d >= PREV_CM && d < NEXT_MIN_CM) return Z_DEAD;

  // 其他（>15）：不触发
  return Z_FAR;
}

void handleGesture() {
  // 只在 STEP 页面响应（保持你原设计）
  bool onStep = (currentState == STEP1 || currentState == STEP2 || currentState == STEP3 ||
                 currentState == STEP4 || currentState == STEP5 || currentState == STEP6 ||
                 currentState == STEP7 || currentState == STEP8);
  if (!onStep) return;

  float d = readUltrasonicCM();
  if (d < 0) return;

  NavZone zone = getZone(d);
  unsigned long now = millis();

  bool entering = (zone != lastZone);                    // 进入新区域
  bool cooledDown = (now - lastNavMs > NAV_COOLDOWN_MS); // 冷却结束

  if (entering && cooledDown) {

    // --- 上一步区：d < 5cm ---
    if (zone == Z_PREV) {
      gestureRight = true; // 上一步
      lastNavMs = now;
    }

    // --- 下一步区：9~15cm ---
    if (zone == Z_NEXT) {
      // 如果你仍希望 STEP4 “只能按钮前进”，就保留这行；不需要就删掉 if
      if (currentState != STEP4) {
        gestureLeft = true; // 下一步
        lastNavMs = now;
      }
    }
  }

  lastZone = zone;
}


void handleAirQuality() {
  if (millis() - lastAirQualityCheckTime > 200) {
    lastAirQualityCheckTime = millis();
    mq2Value = analogRead(MQ2_PIN);
    bool previousAirStatus = airIsBad;
    airIsBad = (mq2Value > MQ2_BAD_THRESHOLD);

    if (airIsBad && !previousAirStatus && currentState != AIR_ALARM && currentState != AIR_RECHECK) {
      changeState(AIR_ALARM);
    }
  }
}

void renderTextScreen(String text, int size) {
  display.clearDisplay();
  display.setTextSize(size);
  display.setTextColor(SH110X_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((display.width() - w) / 2, (display.height() - h) / 2);
  display.println(text);
  display.display();
}

void showStepUI(int step, const unsigned char* img) {
  display.clearDisplay();

  // ---------- Top centered title ----------
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);

  String title = "Step" + String(step);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);

  display.setCursor((SCREEN_WIDTH - (int)w) / 2, 0);
  display.print(title);

  // ---------- Bottom image (no overlap) ----------
  // 16px reserved for title
  display.drawBitmap(0, 16, img, IMG_W, IMG_H, SH110X_WHITE);

  display.display();
}

void showStep4ProgressUI(float progress) {
  progress = constrain(progress, 0.0, 1.0);

  display.clearDisplay();

  // ---------- Top centered title ----------
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  String title = "STEP4";

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, 0);
  display.print(title);

  // ---------- Progress bar in the "image area" ----------
  // 16px reserved for title, so bar goes below
  int x = 10;
  int y = 34;
  int barW = 108;
  int barH = 10;

  display.drawRect(x, y, barW, barH, SH110X_WHITE);
  display.fillRect(x + 2, y + 2, (int)((barW - 4) * progress), barH - 4, SH110X_WHITE);

  display.display();
}


void renderProgressBar(float progress) {
  progress = constrain(progress, 0.0, 1.0);

  display.clearDisplay();

  // 标题：Checking Air（在上方）
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Checking Air");

  // 进度条放在下面（避免和标题重叠）
  display.drawRect(10, 28, 108, 10, SH110X_WHITE);
  display.fillRect(12, 30, (int)(104 * progress), 6, SH110X_WHITE);

  display.display();
}

void setStripColorBrightness(uint8_t r, uint8_t g, uint8_t b, int brightness) {
  strip.setBrightness(brightness);
  strip.fill(strip.Color(r, g, b));
  strip.show();
}

void flashStrip() {
  setStripColorBrightness(0, 255, 0, BRIGHTNESS_FLASH);
  flashActive = true;
  lastFlashTime = millis();
  player.disableLoop();   // 关键：取消 DFPlayer 的循环模式（防止 play(1) 也变循环）
  //player.stop();          // 可选但推荐：打断上一个循环音
  player.play(1);   
}