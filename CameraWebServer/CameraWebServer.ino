//优化控制指令机械臂一直动
//根因不是识别错，而是发送策略：A 板每帧（约 6.5 秒）只要识别到就发一次指令，B 板冷却只有 2.5 秒 → 永远拦不住 → 机械臂就一直动。

已改成边沿触发（L252-L272）：

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_heap_caps.h"

 

// 覆盖 Edge Impulse 的 weak 分配函数：让 TFLite 的 tensor arena(约 327KB) 落在 PSRAM
// 注意：EI 未定义 EI_C_LINKAGE，这两个符号是 C++ 链接，所以这里不要加 extern "C"
void *ei_calloc(size_t nitems, size_t size) {
  size_t bytes = nitems * size;
  void *p = heap_caps_aligned_calloc(16, 1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr) {
    p = heap_caps_aligned_calloc(16, 1, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  return p;
}

void *ei_malloc(size_t size) {
  void *p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr) {
    p = heap_caps_aligned_alloc(16, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  return p;
}
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **//
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#define LED_GPIO_PIN              2

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "荣耀400";
const char *password = "20061031";

// ============ UDP 指令发送（与机械臂板 B 约定）============
#define ARM_BOARD_IP     "10.191.178.220"  // B板静态IP【按和队友约定的实际IP填】
#define ARM_BOARD_PORT   8888
#define LOCAL_UDP_PORT   8889              // 本机只发不收
#define COOL_DOWN_MS     2500              // 指令冷却：2.5s 内不重复发送

// 发送策略：只在“识别结果发生变化”时发一次，机械臂不会一直动
#define ALLOW_SAME_CMD_REPEAT  0           // 0=同一表情只触发一次；1=识别失效(0)后允许再次触发
// 置信度上限（可选兜底）：>0 时，置信度 >= 该值视为“无人脸/不可信”，不发指令
// 注意：真的 sleepy 表情也可能 >0.9，设之前先测数据。0 = 关闭
#define SCORE_MAX_VALID  0.00f

WiFiUDP udp;

extern volatile int   g_emotion_cmd;       // 来自 edge_impulse.cpp：0=无有效识别 1=开心 2=不开心
extern volatile float g_emotion_score;

int  lastSentCmd  = 0;      // 上一次实际发出的指令（边沿触发用）
unsigned long lastSendMs = 0;

void sendEmotionCmd(int cmd) {
  udp.beginPacket(ARM_BOARD_IP, ARM_BOARD_PORT);
  udp.write((uint8_t)cmd);
  udp.endPacket();
  Serial.printf("UDP发送指令: %d\n", cmd);
}

bool bOn = false;

extern void ei_edge_impulse(camera_fb_t *fb);
extern void startCameraServer();
extern void setupLedFlash(int pin);

void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("step1 ok");
  pinMode(LED_GPIO_PIN, OUTPUT);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  //config.frame_size = FRAMESIZE_96X96;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    bOn = !bOn;
    delay(250);
    Serial.print(".");
    digitalWrite(LED_GPIO_PIN, bOn ? LOW : HIGH);
  }
  Serial.println("");
  Serial.println("WiFi connected");

  udp.begin(LOCAL_UDP_PORT);
  Serial.printf("视觉板A IP: %s  -> 目标 %s:%d\n",
                WiFi.localIP().toString().c_str(), ARM_BOARD_IP, ARM_BOARD_PORT);

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Serial.printf("HeapSize: %d bytes\n", ESP.getHeapSize());
  Serial.printf("FreeHeap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("MinFreeHeap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.printf("MaxAllocHeap: %d bytes\n", ESP.getMaxAllocHeap());
  
  // 如果有PSRAM
  if (ESP.getPsramSize() > 0) {
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
    Serial.printf("FreePSRAM: %d bytes\n", ESP.getFreePsram());
  }
  
  Serial.printf("SketchSize: %d bytes\n", ESP.getSketchSize());
  Serial.printf("FreeSketchSpace: %d bytes\n", ESP.getFreeSketchSpace());
}



void loop() {
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("获取摄像头帧失败");
    digitalWrite(LED_GPIO_PIN, bOn ? LOW : HIGH);
    bOn = !bOn;
    delay(500);
    return;
  }

  // 诊断：期望 fmt=4(JPEG) width=320 height=240，len 约 5~30KB
  Serial.printf("fb buf=%p len=%u fmt=%d %ux%u\n",
                fb->buf, (unsigned)fb->len, (int)fb->format, fb->width, fb->height);

  ei_edge_impulse(fb);            // 原有推理，阻塞约 6.3 秒

  esp_camera_fb_return(fb);   // 只释放一次

  // ---- 发送策略：只在“识别结果发生变化”时发一次，避免机械臂一直运动 ----
  int   cmd   = g_emotion_cmd;
  float score = g_emotion_score;

  // 可选兜底：置信度过高时按“无人脸/不可信”处理（SCORE_MAX_VALID=0 表示关闭）
  if (SCORE_MAX_VALID > 0.0f && cmd != 0 && score >= SCORE_MAX_VALID) {
    Serial.printf("置信度 %.3f >= %.2f，按无人脸处理，不发送\n",
                  (double)score, (double)SCORE_MAX_VALID);
    cmd = 0;
  }

  // 边沿触发：只有指令相对“上次已发”发生变化时才发；搭配冷却防止刷屏
  unsigned long now = millis();
  if (cmd != 0 && cmd != lastSentCmd && (now - lastSendMs >= COOL_DOWN_MS)) {
    sendEmotionCmd(cmd);
    lastSentCmd = cmd;
    lastSendMs  = now;
  } else if (cmd == 0 && ALLOW_SAME_CMD_REPEAT) {
    lastSentCmd = 0;   // 识别失效后复位，同一种表情可以再次触发
  }

  Serial.printf("识别: %s  cmd=%d  score=%.3f  上次已发=%d\n",
                cmd == 1 ? "peaceful(开心)" : (cmd == 2 ? "sleepy(不开心)" : "无有效识别"),
                cmd, (double)score, lastSentCmd);

  // 推理一帧约 6.5 秒，这里不需要额外延时
