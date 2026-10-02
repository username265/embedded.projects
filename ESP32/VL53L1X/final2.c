#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

constexpr int LED_PIN = 4;
constexpr int BUZZER_PIN = 5;
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;

// HIGH 신호로 울리는 능동형 부저/구동 회로: true
// LOW 신호로 울리는 부저 모듈: false
constexpr bool BUZZER_ACTIVE_HIGH = true;
constexpr uint32_t PRINT_MS = 500;
constexpr uint32_t STALE_MS = 2000;
constexpr uint32_t STARTUP_MS = 600;

enum Stage { BOOTING, STARTUP, NORMAL, WARN30, WARN40, ALARM50,
             SENSOR_ERROR, SYSTEM_ERROR };

struct Reading {
  float maxC;
  uint32_t at;
  bool valid;
};

Adafruit_MLX90640 mlx;
float frame[32 * 24];
QueueHandle_t readings = nullptr;
Reading latest = {NAN, 0, false};
Stage stage = BOOTING;
uint32_t stageAt = 0, lastPrint = 0, bootAt = 0;
bool haveReading = false, startupPlayed = false;
bool alarmRunning = false, systemFailed = false;

void setBuzzer(bool on) {
  digitalWrite(BUZZER_PIN, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

Stage temperatureStage(float t) {
  // 하강할 때도 같은 기준을 사용합니다. 추가 온도 여유는 없습니다.
  if (t >= 50.0f) return ALARM50;
  if (t >= 40.0f) return WARN40;
  if (t >= 30.0f) return WARN30;
  return NORMAL;
}

void enterStage(Stage next, uint32_t now) {
  if (next == stage) return;  // 같은 구간이면 경보를 다시 시작하지 않음
  stage = next;
  stageAt = now;
  alarmRunning = (next == WARN30 || next == WARN40 || next == ALARM50);
}

const char *stageName(Stage s) {
  switch (s) {
    case BOOTING:      return "BOOTING";
    case STARTUP:      return "STARTUP";
    case NORMAL:       return "NORMAL";
    case WARN30:       return "WARN30";
    case WARN40:       return "WARN40";
    case ALARM50:      return "ALARM50";
    case SENSOR_ERROR: return "SENSOR_ERROR";
    default:          return "SYSTEM_ERROR";
  }
}

// 센서 읽기는 별도 작업에서 실행: 부저/LED/시리얼의 시간을 방해하지 않음.
void sensorTask(void *parameter) {
  if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
    Reading error = {NAN, millis(), false};
    xQueueOverwrite(readings, &error);
    vTaskDelete(nullptr);
    return;
  }
  Wire.setTimeOut(50);
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Reading error = {NAN, millis(), false};
    xQueueOverwrite(readings, &error);
    // 초기화 실패는 배선을 확인한 뒤 RESET. 실행 중 읽기 오류는 아래에서 재시도.
    vTaskDelete(nullptr);
    return;
  }
  Wire.setClock(400000);  // 라이브러리 초기화 후 I2C 속도 재설정
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_8_HZ);

  for (;;) {
    // 이전 프레임의 값이 남아 정상 데이터처럼 쓰이지 않게 합니다.
    for (float &t : frame) t = NAN;
    bool valid = (mlx.getFrame(frame) == 0);
    float hottest = -INFINITY;
    if (valid) {
      for (float t : frame) {
        if (!isfinite(t)) {
          valid = false;
          break;
        }
        if (t > hottest) hottest = t;
      }
    }
    Reading result = {valid ? hottest : NAN, millis(), valid};
    xQueueOverwrite(readings, &result);
    vTaskDelay(pdMS_TO_TICKS(valid ? 10 : 100));
  }
}

void updateOutputs(uint32_t now) {
  const uint32_t elapsed = now - stageAt;
  bool ledOn = false, buzzerOn = false;
  uint32_t alarmLength = 0;

  switch (stage) {
    case STARTUP:
      ledOn = true;
      // 능동형 부저: 음높이 변경 대신 0.1초씩 3번 울림.
      buzzerOn = (elapsed < STARTUP_MS && elapsed % 200 < 100);
      break;
    case NORMAL:
      ledOn = true;
      break;
    case WARN30:
      ledOn = (elapsed % 1000 < 500);  // 0.5초 ON / 0.5초 OFF
      alarmLength = 2500;             // 0, 1, 2초에 시작하는 0.5초 소리 3회
      break;
    case WARN40:
      ledOn = (elapsed % 250 < 125);   // 0.125초 ON / 0.125초 OFF
      alarmLength = 4500;             // 0, 1, 2, 3, 4초에 시작하는 소리 5회
      break;
    case ALARM50:
      alarmLength = 3000;             // LED OFF, 3초 연속 소리
      break;
    default:
      break;                         // 부팅 대기/오류: 둘 다 OFF
  }

  if (alarmRunning) {
    if (elapsed >= alarmLength) {
      alarmRunning = false;          // 같은 구간에서는 한 번만 실행
    } else {
      buzzerOn = (stage == ALARM50 || elapsed % 1000 < 500);
    }
  }
  digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  setBuzzer(buzzerOn);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  Serial.begin(115200);
  bootAt = lastPrint = millis();
  readings = xQueueCreate(1, sizeof(Reading));
  if (readings == nullptr ||
      xTaskCreate(sensorTask, "MLX90640", 8192, nullptr, 1, nullptr) != pdPASS) {
    systemFailed = true;
  }
}

void loop() {
  Reading incoming;
  if (readings != nullptr && xQueueReceive(readings, &incoming, 0) == pdTRUE) {
    latest = incoming;
    haveReading = true;
  }
  const uint32_t now = millis();
  const bool fresh = haveReading && latest.valid &&
                     uint32_t(now - latest.at) < STALE_MS;

  if (systemFailed) {
    enterStage(SYSTEM_ERROR, now);
  } else if (!fresh) {
    if (haveReading || uint32_t(now - bootAt) >= STALE_MS) {
      enterStage(SENSOR_ERROR, now);
    }
  } else if (!startupPlayed) {
    startupPlayed = true;
    enterStage(STARTUP, now);        // 첫 정상 프레임을 받은 뒤 시작 알림
  } else if (stage != STARTUP || uint32_t(now - stageAt) >= STARTUP_MS) {
    // 구간이 바뀌면 이전 경보 중단 후 새 구간 패턴을 한 번 실행합니다.
    enterStage(temperatureStage(latest.maxC), now);
  }

  updateOutputs(now);

  if (uint32_t(now - lastPrint) >= PRINT_MS) {
    lastPrint += PRINT_MS;
    if (uint32_t(now - lastPrint) >= PRINT_MS) lastPrint = now;
    if (Serial) {                   // 모니터를 열지 않아도 경보는 동작
      if (fresh) Serial.printf("Tmax=%.2f C | State=%s\n", latest.maxC, stageName(stage));
      else Serial.printf("Tmax=-- C | State=%s\n", stageName(stage));
    }
  }
  delay(5);
}
