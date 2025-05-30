#include <Arduino.h>
#include "driver/i2s.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ArduinoFFT.h>
#include <math.h>
#include <Wire.h>             // <<<<<<<<<<< 新增：包含 Wire 库
#include "RevEng_PAJ7620.h" // <<<<<<<<<<< 新增：包含 PAJ7620 库头文件

// --- WiFi ---
#include <WiFi.h>
#include <HTTPClient.h> // For OpenWeatherMap
#include <ArduinoJson.h> // For OpenWeatherMap
#include "time.h"       // For NTP time

// --- HUB75 LED 屏幕参数配置 ---
#define PANEL_WIDTH   64
#define PANEL_HEIGHT  32
#define PANELS_NUMBER 1

// --- HUB75 ESP32-S3 引脚配置 (保持不变) ---
#define R1_PIN GPIO_NUM_9
#define G1_PIN GPIO_NUM_10
#define B1_PIN GPIO_NUM_11
#define R2_PIN GPIO_NUM_14
#define G2_PIN GPIO_NUM_12
#define B2_PIN GPIO_NUM_13
#define A_PIN  GPIO_NUM_8
#define B_PIN  GPIO_NUM_1
#define C_PIN  GPIO_NUM_5
#define D_PIN  GPIO_NUM_17
#define E_PIN  (gpio_num_t)-1
#define LAT_PIN GPIO_NUM_4
#define OE_PIN  GPIO_NUM_2
#define CLK_PIN GPIO_NUM_16

MatrixPanel_I2S_DMA *dma_display = nullptr;

// --- INMP441 I2S 引脚配置 (保持不变) ---
#define I2S_WS_PIN   39
#define I2S_SCK_PIN  40
#define I2S_SD_PIN   41
#define I2S_PORT_NUM I2S_NUM_0

// --- PAJ7620 手势传感器引脚配置 ---
#define PAJ_SDA_PIN 6 // <<<<<<<<<<< 新增
#define PAJ_SCL_PIN 7 // <<<<<<<<<<< 新增
RevEng_PAJ7620 paj_sensor;  // <<<<<<<<<<< 新增：创建传感器对象

// --- WiFi Credentials ---
const char* WIFI_SSID = "areyouok";
const char* WIFI_PASSWORD = "888888888";

// --- OpenWeatherMap API ---
const String OPENWEATHERMAP_API_KEY = "3ffe513ed138b2f60cfee3c72fb1c90c";
const String OPENWEATHERMAP_CITY = "Tianjin";
const String OPENWEATHERMAP_API_URL_BASE = "http://api.openweathermap.org/data/2.5/weather";

// --- NTP Time ---
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 8 * 3600; // GMT+8 for Tianjin
const int   DAYLIGHT_OFFSET_SEC = 0;
struct tm timeinfo;
char timeStringBuff[10]; // HH:MM:SS\0 or "No Time\0"

// --- Application Modes ---
enum AppMode {
  SPECTRUM_MODE,
  WEATHER_CLOCK_MODE
};
AppMode current_mode = SPECTRUM_MODE;
bool mode_changed = true; // Flag to indicate mode has changed, for initial display update
bool force_redraw_weather_clock = true; // 新增: 天气时钟模式强制刷新标志
String last_displayed_time = ""; // 用于检测时间变化

// --- Weather Data Store ---
String weather_description = "Loading...";
float temperature = 0.0;
String weather_icon_code = ""; // e.g., "01d", "10n"
unsigned long last_weather_update = 0;
// Update weather every 15 minutes (15 * 60 * 1000 milliseconds)
const unsigned long WEATHER_UPDATE_INTERVAL = 900000; 

// --- I2S 和 FFT 配置参数 ---
// ... (你现有的 FFT 和频谱参数保持不变) ...
#define I2S_SAMPLE_RATE     (16000)
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define FFT_SAMPLES_COUNT   (512)

#define I2S_DMA_BUF_COUNT    (8)
#define I2S_DMA_BUF_LEN_SAMPLES (FFT_SAMPLES_COUNT)

float vReal[FFT_SAMPLES_COUNT];
float vImag[FFT_SAMPLES_COUNT];
ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLES_COUNT, (float)I2S_SAMPLE_RATE);

#define NUM_BANDS (PANEL_WIDTH)
#define F_MIN_HZ 40.0f
int band_start_bins[NUM_BANDS];
int band_end_bins[NUM_BANDS];
const float FREQ_RESOLUTION = (float)I2S_SAMPLE_RATE / FFT_SAMPLES_COUNT;
const int USEFUL_FFT_BINS = FFT_SAMPLES_COUNT / 2;

#define NOISE_FLOOR 26000.0f
#define MAX_AMP_LOG 8.8f
#define MIN_AMP_LOG 4.3f
#define BAR_SENSITIVITY 1.0f

float band_heights[NUM_BANDS];
float peak_heights[NUM_BANDS];
unsigned long peak_timers[NUM_BANDS];
float band_velocities[NUM_BANDS];

#define PEAK_FALL_DELAY 200
#define PEAK_FALL_RATE_PIXELS_PER_FRAME 0.3f
#define SMOOTHING_FACTOR_RISE 0.80f
#define GRAVITY_ACCELERATION 0.15f
#define MAX_FALL_SPEED 2.5f
#define INITIAL_FALL_VELOCITY 0.05f
#define NORM_AMP_PIVOT 0.50f
#define HEIGHT_AT_PIVOT_FACTOR 0.60f
#define UPPER_HEIGHT_EXPONENT 0.35f
#define LOWER_HEIGHT_EXPONENT 1.8f
uint16_t precomputed_bar_colors[NUM_BANDS];
uint16_t precomputed_peak_color;
#define POW_LUT_SIZE 256
float pow_lut_lower[POW_LUT_SIZE];
float pow_lut_upper[POW_LUT_SIZE];
// --- 调试开关 ---
// #define DEBUG_SERIAL_LEVEL_1
// #define DEBUG_SERIAL_LEVEL_2
#ifdef DEBUG_SERIAL_LEVEL_2
static unsigned long last_print_time_detail = 0;
#endif

// 函数原型 (如果你的 printGestureName 函数定义在 loop 之后)
void printGestureName(Gesture gesture); // <<<<<<<<<<< 新增
void setup_wifi();
void fetch_weather_data();
void update_time();
void display_spectrum();
void display_weather_clock();
void draw_text_with_outline(int16_t x, int16_t y, const String& text, uint16_t text_color, uint16_t outline_color, uint8_t size, bool centerX);
void handle_gesture_input();

// >>>>>>>>>>>>>>>>>> START: 天气图标与绘制函数 <<<<<<<<<<<<<<<<<<
// --- Icon Data (8x8, using RGB888, will be converted to 565) ---
const uint32_t sun_8x8[] = {
  0xFFFF00, 0x000000, 0x000000, 0xFFFF00, 0x000000, 0x000000, 0x000000, 0xFFFF00,
  0x000000, 0xFFFF00, 0x000000, 0x000000, 0x000000, 0x000000, 0xFFFF00, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFF00, 0xFFFF00, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0x000000, 0xFFFF00,
  0xFFFF00, 0x000000, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0xFFFF00, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFF00, 0xFFFF00, 0x000000, 0x000000, 0x000000,
  0x000000, 0xFFFF00, 0x000000, 0x000000, 0x000000, 0x000000, 0xFFFF00, 0x000000,
  0xFFFF00, 0x000000, 0x000000, 0x000000, 0xFFFF00, 0x000000, 0x000000, 0xFFFF00,
};
// Cloud (using light blue for clouds)
const uint32_t cloud_8x8[] = {
  0x000000, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000, 0x000000, 0x000000, 0x000000,
  0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000,
  0xADD8E6, 0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6,
  0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xADD8E6, 0xADD8E6, 0xADD8E6,
  0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xADD8E6, 0xADD8E6,
  0x000000, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
};
const uint32_t showers_8x8[] = {
  0x000000, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000, 0x000000, 0x000000, 0x000000,
  0xADD8E6, 0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000,
  0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xADD8E6, 0x000000, 0xADD8E6,
  0xADD8E6, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x0000FF, 0x000000, 0xADD8E6,
  0xADD8E6, 0xADD8E6, 0xADD8E6, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0xADD8E6,
  0x000000, 0xADD8E6, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x000000,
  0x000000, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
};
const uint32_t rain_8x8[] = {
  0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF,
  0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000,
  0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF,
  0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000,
  0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF,
  0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000,
  0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF,
  0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000
};
const uint32_t storm_8x8[] = {
  0x000000, 0x696969, 0x696969, 0x696969, 0x000000, 0x000000, 0x000000, 0x000000,
  0x696969, 0xA9A9A9, 0xA9A9A9, 0xA9A9A9, 0x696969, 0x696969, 0x000000, 0x000000,
  0x696969, 0xA9A9A9, 0xA9A9A9, 0x696969, 0x000000, 0xFFFF00, 0x000000, 0x000000,
  0x000000, 0x696969, 0x000000, 0xFFFF00, 0xFFFF00, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0xFFFF00, 0xFFFF00, 0x000000, 0x696969, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFF00, 0x000000, 0x696969, 0x696969, 0x000000,
  0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x000000,
  0x000000, 0x000000, 0x0000FF, 0x000000, 0x0000FF, 0x000000, 0x000000, 0x000000,
};
const uint32_t snow_8x8[] = {
  0x000000, 0xD3D3D3, 0xD3D3D3, 0xD3D3D3, 0x000000, 0x000000, 0x000000, 0x000000,
  0xD3D3D3, 0xF5F5F5, 0xF5F5F5, 0xF5F5F5, 0xD3D3D3, 0xD3D3D3, 0x000000, 0x000000,
  0xD3D3D3, 0xF5F5F5, 0xF5F5F5, 0xD3D3D3, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000,
  0x000000, 0xD3D3D3, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xD3D3D3, 0x000000, 0xFFFFFF,
  0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0x000000,
  0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xD3D3D3, 0xFFFFFF, 0x000000, 0xFFFFFF,
  0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0xFFFFFF, 0x000000,
  0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0xFFFFFF,
};
const uint32_t unknown_weather_8x8[] = {
  0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000,
  0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000,
  0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0x000000, 0x000000,
};
uint16_t color565(uint32_t rgb) {
  return (((rgb >> 16) & 0xF8) << 8) |
         (((rgb >> 8) & 0xFC) << 3) |
         ((rgb & 0xFF) >> 3);
};
void drawWeatherBitmap(int startx, int starty, int width, int height, const uint32_t *bitmap) {
  if (!dma_display) return;
  int counter = 0;
  for (int yy = 0; yy < height; yy++) {
    for (int xx = 0; xx < width; xx++) {
      dma_display->drawPixel(startx + xx, starty + yy, color565(bitmap[counter]));
      counter++;
    }
  }
}
void drawWeatherBitmap(int startx, int starty, int width, int height, const uint32_t *bitmap, bool enlarged) {
  if (!dma_display) return;
  int counter = 0;
  if (enlarged) {
    for (int yy = 0; yy < height; yy++) {
      for (int xx = 0; xx < width; xx++) {
        uint16_t pixel_color = color565(bitmap[counter]);
        dma_display->drawPixel(startx + 2 * xx, starty + 2 * yy, pixel_color);
        dma_display->drawPixel(startx + 2 * xx + 1, starty + 2 * yy, pixel_color);
        dma_display->drawPixel(startx + 2 * xx, starty + 2 * yy + 1, pixel_color);
        dma_display->drawPixel(startx + 2 * xx + 1, starty + 2 * yy + 1, pixel_color);
        counter++;
      }
    }
  } else {
    drawWeatherBitmap(startx, starty, width, height, bitmap);
  }
}
void drawWeatherIcon(int startx, int starty, int width, int height, uint8_t icon_idx, bool enlarged) {
  const uint32_t* icon_data_ptr;
  switch (icon_idx) {
    case 0: icon_data_ptr = sun_8x8; break;
    case 1: icon_data_ptr = cloud_8x8; break;
    case 2: icon_data_ptr = showers_8x8; break;
    case 3: icon_data_ptr = rain_8x8; break;
    case 4: icon_data_ptr = storm_8x8; break;
    case 5: icon_data_ptr = snow_8x8; break;
    default: icon_data_ptr = unknown_weather_8x8; break;
  }
  drawWeatherBitmap(startx, starty, width, height, icon_data_ptr, enlarged);
}
int mapOwmIconToInternal(String owmIcon) {
    if (owmIcon.startsWith("01")) return 0; // 晴天
    if (owmIcon.startsWith("02")) return 1; // 少云
    if (owmIcon.startsWith("03")) return 1; // 散云
    if (owmIcon.startsWith("04")) return 1; // 阴天
    if (owmIcon.startsWith("09")) return 2; // 阵雨
    if (owmIcon.startsWith("10")) return 3; // 雨
    if (owmIcon.startsWith("11")) return 4; // 雷暴
    if (owmIcon.startsWith("13")) return 5; // 雪
    if (owmIcon.startsWith("50")) return 1; // 雾
    return 6; // 未知
}
// >>>>>>>>>>>>>>>>>> END: 天气图标与绘制函数 <<<<<<<<<<<<<<<<<<

void setup_hub75_display() {
  HUB75_I2S_CFG::i2s_pins _pins = {
    R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN,
    A_PIN, B_PIN, C_PIN, D_PIN, E_PIN,
    LAT_PIN, OE_PIN, CLK_PIN
  };
  HUB75_I2S_CFG mxconfig(PANEL_WIDTH, PANEL_HEIGHT, PANELS_NUMBER, _pins);
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  if (!dma_display || !dma_display->begin()) {
    Serial.println("****** HUB75 MatrixPanel_I2S_DMA setup FAILED! ******");
    while (1);
  }
  Serial.println("HUB75 MatrixPanel_I2S_DMA setup successful!");
  dma_display->setBrightness8(30); // 初始亮度
  dma_display->clearScreen();
  dma_display->flipDMABuffer();
}

void setup_i2s_microphone() {
  Serial.println("--- Configuring I2S for FFT_SAMPLES_COUNT samples ---");
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  Serial.printf("Attempting I2S driver install with: SR=%d, BPS=%d, DMA_Count=%d, DMA_Len_Samples=%d\n",
                i2s_config.sample_rate, (int)i2s_config.bits_per_sample, i2s_config.dma_buf_count, i2s_config.dma_buf_len);

  esp_err_t err = i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing I2S microphone driver: %d (%s)\n", err, esp_err_to_name(err));
    while (1);
  }
  Serial.println("I2S microphone driver installed successfully.");

  err = i2s_set_pin(I2S_PORT_NUM, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting I2S microphone pin config: %d (%s)\n", err, esp_err_to_name(err));
    while (1);
  }
  Serial.println("I2S microphone pins configured successfully.");
}

// <<<<<<<<<<< 新增：手势传感器初始化函数
void setup_paj7620_sensor() {
  Serial.println("--- Initializing PAJ7620 Gesture Sensor ---");
  // 初始化 I2C 总线，必须在 paj_sensor.begin() 之前
  // 如果你的项目中其他地方没有 Wire.begin()，或者需要指定特定引脚
  Wire.begin(PAJ_SDA_PIN, PAJ_SCL_PIN);
  Serial.print("I2C bus for PAJ7620 initialized on SDA: "); Serial.print(PAJ_SDA_PIN);
  Serial.print(", SCL: "); Serial.println(PAJ_SCL_PIN);

  if (paj_sensor.begin()) { // 使用默认的 Wire (我们刚刚配置了)
    Serial.println("PAJ7620 initialized successfully!");
  } else {
    Serial.println("PAJ7620 initialization FAILED! Check wiring.");
    // 这里可以选择是否要 while(1); 阻塞，或者允许程序继续但手势功能不可用
  }
}

void setup_wifi() {
  Serial.println("--- Connecting to WiFi ---");
  Serial.printf("SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Try for 10 seconds
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    // Initialize NTP
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    update_time(); // Get initial time
    fetch_weather_data(); // Get initial weather
  } else {
    Serial.println("\nWiFi connection FAILED!");
    strcpy(timeStringBuff, "No WiFi");
    weather_description = "No WiFi";
  }
}

uint16_t hsv_to_panel_color(float h, float s, float v) {
    // ... (你现有的 hsv_to_panel_color 函数保持不变) ...
    float r_f, g_f, b_f;
    h = fmod(h, 360.0f);
    if (h < 0) h += 360.0f;
    int i = floor(h / 60.0f);
    float f = (h / 60.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    switch (i % 6) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        case 5: r_f = v; g_f = p; b_f = q; break;
        default: r_f = 0; g_f = 0; b_f = 0; break;
    }
    uint8_t r4 = (uint8_t)(r_f * 15.99f);
    uint8_t g4 = (uint8_t)(g_f * 15.99f);
    uint8_t b4 = (uint8_t)(b_f * 15.99f);
    if (dma_display) {
        return dma_display->color444(r4, g4, b4);
    }
    return 0;
}

void calculate_logarithmic_bins() {
    // ... (你现有的 calculate_logarithmic_bins 函数保持不变) ...
    Serial.println("--- Calculating Logarithmic Bins (v2) ---");
    float max_freq_hz = (float)I2S_SAMPLE_RATE / 2.0f;
    float min_possible_freq = FREQ_RESOLUTION;
    float f_start_for_calc = max(F_MIN_HZ, min_possible_freq);
    float log_min = log10f(f_start_for_calc);
    float log_max = log10f(max_freq_hz);
    float log_range = log_max - log_min;

    if (log_range <= 0.001f) {
        Serial.println("Warning: Logarithmic frequency range is too small. Applying fallback linear-like binning.");
        int bins_per_display_band = (USEFUL_FFT_BINS -1) / NUM_BANDS;
        if (bins_per_display_band < 1) bins_per_display_band = 1;
        for (int i = 0; i < NUM_BANDS; i++) {
            band_start_bins[i] = max(1, 1 + i * bins_per_display_band);
            band_end_bins[i] = max(1, band_start_bins[i] + bins_per_display_band - 1);
            if (i == NUM_BANDS - 1) band_end_bins[i] = USEFUL_FFT_BINS - 1;
            band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
            band_end_bins[i]   = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);
            if (band_end_bins[i] < band_start_bins[i]) band_end_bins[i] = band_start_bins[i];
        }
        return;
    }
    for (int i = 0; i < NUM_BANDS; i++) {
        float band_log_low = log_min + ( (float)i / NUM_BANDS ) * log_range;
        float band_log_high = log_min + ( (float)(i + 1) / NUM_BANDS ) * log_range;
        float freq_low_ideal = pow10f(band_log_low);
        float freq_high_ideal = pow10f(band_log_high);
        int start_bin_ideal = roundf(freq_low_ideal / FREQ_RESOLUTION);
        int end_bin_ideal = roundf(freq_high_ideal / FREQ_RESOLUTION) -1;
        if (i == 0) { band_start_bins[i] = max(1, start_bin_ideal);
        } else { band_start_bins[i] = max(start_bin_ideal, band_end_bins[i-1] + 1); }
        band_end_bins[i] = max(band_start_bins[i], end_bin_ideal);
        band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
        band_end_bins[i] = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);
        if (band_start_bins[i] > band_end_bins[i]) { band_end_bins[i] = band_start_bins[i]; }
        if (i == NUM_BANDS - 1) {
            band_end_bins[i] = USEFUL_FFT_BINS - 1;
            if (band_start_bins[i] > band_end_bins[i]) { band_start_bins[i] = band_end_bins[i];}}
        if (band_start_bins[i] >= USEFUL_FFT_BINS -1 && i < NUM_BANDS -1) {
            for (int j = i; j < NUM_BANDS; ++j) {
                band_start_bins[j] = USEFUL_FFT_BINS - 1;
                band_end_bins[j] = USEFUL_FFT_BINS - 1;}
            #ifdef DEBUG_SERIAL_LEVEL_2
            Serial.printf("Band %2d onwards mapped to last FFT bin (%d) due to scarcity.\n", i, USEFUL_FFT_BINS - 1);
            #endif
            break;}
        #ifdef DEBUG_SERIAL_LEVEL_2
        if ( (NUM_BANDS <= 10) || (i < 5 || i >= NUM_BANDS - 5 || (i % (NUM_BANDS/10)) == 0 ) ) {
            if (millis() - last_print_time_detail > 10 ) {
                 Serial.printf("Log Band %2d: Freq_ideal %.0f-%.0f Hz => FFT Bins %3d-%3d (Len %d)\n",
                              i, freq_low_ideal, freq_high_ideal,
                              band_start_bins[i], band_end_bins[i], band_end_bins[i] - band_start_bins[i] + 1);
                 last_print_time_detail = millis(); }}}
        #endif
    }
    Serial.println("--- Adjusted Low Frequency Bins (First 5 - Logarithmic): ---");
    for(int i=0; i < min(5, NUM_BANDS); ++i) {
        float f_low_actual = band_start_bins[i] * FREQ_RESOLUTION;
        float f_high_actual = (band_end_bins[i] + 1) * FREQ_RESOLUTION;
         Serial.printf("  Adj LogBand %2d: FFT Bins %3d-%3d (Actual Freq ~%.0f-%.0f Hz)\n",
                      i, band_start_bins[i], band_end_bins[i], f_low_actual, f_high_actual); }
    Serial.println("--- Logarithmic Bins Calculated (v2) ---");
}

void calculate_linear_bins() {
  // ... (你现有的 calculate_linear_bins 函数保持不变) ...
  Serial.println("--- Calculating Linear Frequency Bins ---");
  float min_possible_freq = FREQ_RESOLUTION;
  float min_map_freq = max(F_MIN_HZ, min_possible_freq);
  float max_map_freq = (float)I2S_SAMPLE_RATE / 2.0f;
  float total_freq_span_to_map = max_map_freq - min_map_freq;

  if (total_freq_span_to_map <= 0.001f) {
      Serial.println("Error: Frequency span for binning is too small or invalid.");
      int bins_per_band = (USEFUL_FFT_BINS -1) / NUM_BANDS;
      if (bins_per_band < 1) bins_per_band = 1;
      for (int i = 0; i < NUM_BANDS; i++) {
          band_start_bins[i] = max(1, 1 + i * bins_per_band);
          band_end_bins[i] = max(1, band_start_bins[i] + bins_per_band - 1);
          if (i == NUM_BANDS -1) band_end_bins[i] = USEFUL_FFT_BINS - 1;
          band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
          band_end_bins[i]   = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);
          if (band_end_bins[i] < band_start_bins[i]) band_end_bins[i] = band_start_bins[i];}
      Serial.println("Fallback binning applied.");
      return;}

  float linear_freq_step_per_band = total_freq_span_to_map / NUM_BANDS;

  for (int i = 0; i < NUM_BANDS; i++) {
    float freq_low_ideal = min_map_freq + ((float)i * linear_freq_step_per_band);
    float freq_high_ideal = min_map_freq + ((float)(i + 1) * linear_freq_step_per_band);
    int start_bin_ideal = roundf(freq_low_ideal / FREQ_RESOLUTION);
    int end_bin_ideal = roundf(freq_high_ideal / FREQ_RESOLUTION) - 1;
    if (i == 0) { band_start_bins[i] = max(1, start_bin_ideal);
    } else { band_start_bins[i] = max(start_bin_ideal, band_end_bins[i - 1] + 1); }
    band_end_bins[i] = max(band_start_bins[i], end_bin_ideal);
    band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
    band_end_bins[i] = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);
    if (band_start_bins[i] > band_end_bins[i]) { band_end_bins[i] = band_start_bins[i]; }
    if (i == NUM_BANDS - 1) {
      band_end_bins[i] = USEFUL_FFT_BINS - 1;
      if (band_start_bins[i] > band_end_bins[i]) { band_start_bins[i] = band_end_bins[i]; }}
    if (band_start_bins[i] >= USEFUL_FFT_BINS - 1 && i < NUM_BANDS - 1) {
      for (int j = i; j < NUM_BANDS; ++j) {
        band_start_bins[j] = USEFUL_FFT_BINS - 1;
        band_end_bins[j] = USEFUL_FFT_BINS - 1; }
      #ifdef DEBUG_SERIAL_LEVEL_2
      Serial.printf("Band %2d onwards mapped to last FFT bin (%d).\n", i, USEFUL_FFT_BINS - 1);
      #endif
      break; }
    #ifdef DEBUG_SERIAL_LEVEL_2
    if ( (NUM_BANDS <= 10) || (i < 5 || i >= NUM_BANDS - 5 || (i % (NUM_BANDS/10)) == 0 ) ) {
       if (millis() - last_print_time_detail > 10 ) {
            Serial.printf("Lin. Band %2d: Freq_ideal %.0f-%.0f Hz => FFT Bins %3d-%3d (Len %d)\n",
                          i, freq_low_ideal, freq_high_ideal,
                          band_start_bins[i], band_end_bins[i], band_end_bins[i] - band_start_bins[i] + 1);
            last_print_time_detail = millis(); }}}
    #endif
  }
  Serial.println("--- Adjusted Low Frequency Bins (First 5 - Linear): ---");
  for (int i = 0; i < min(5, NUM_BANDS); ++i) {
    float f_low_actual = band_start_bins[i] * FREQ_RESOLUTION;
    float f_high_actual = (band_end_bins[i] + 1) * FREQ_RESOLUTION;
    Serial.printf("  Adj LinBand %2d: FFT Bins %3d-%3d (Actual Freq ~%.0f-%.0f Hz)\n",
                  i, band_start_bins[i], band_end_bins[i], f_low_actual, f_high_actual); }
  Serial.println("--- Linear Frequency Bins Calculated ---");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // 等待串口连接，增加超时
  delay(1000);
  Serial.println("Starting setup...");

  setup_hub75_display();
  setup_i2s_microphone();
  setup_paj7620_sensor(); // <<<<<<<<<<< 新增：调用手势传感器初始化
  setup_wifi(); // Connect to WiFi and initialize NTP

  calculate_linear_bins();

  precomputed_peak_color = hsv_to_panel_color(0, 0.0f, 1.0f); // White
  for (int i = 0; i < NUM_BANDS; ++i) {
    float hue = ((float)i / (NUM_BANDS - 1)) * 240.0f;
    precomputed_bar_colors[i] = hsv_to_panel_color(hue, 1.0f, 1.0f);
  }

  for (int i = 0; i < POW_LUT_SIZE; ++i) {
    float x = (float)i / (POW_LUT_SIZE - 1);
    pow_lut_lower[i] = powf(x, LOWER_HEIGHT_EXPONENT);
    pow_lut_upper[i] = powf(x, UPPER_HEIGHT_EXPONENT);
  }

  for (int i = 0; i < NUM_BANDS; i++) {
    band_heights[i] = 0.0f;
    peak_heights[i] = 0.0f;
    peak_timers[i] = 0;
    band_velocities[i] = 0.0f;
  }

  Serial.println("Setup complete. Initial mode: Spectrum Display.");
  current_mode = SPECTRUM_MODE; // Explicitly set initial mode
  mode_changed = true; // Trigger initial display update for the mode
}

void handle_gesture_input() {
    Gesture gesture = paj_sensor.readGesture();
    bool mode_switched_this_cycle = false;

    if (gesture == GES_RIGHT) {
        if (current_mode != WEATHER_CLOCK_MODE) {
            Serial.println("Gesture: RIGHT - Switching to Weather/Clock mode");
            current_mode = WEATHER_CLOCK_MODE;
            mode_changed = true; 
            force_redraw_weather_clock = true; 
            mode_switched_this_cycle = true;
        }
    } else if (gesture == GES_LEFT) {
        if (current_mode != SPECTRUM_MODE) {
            Serial.println("Gesture: LEFT - Switching to Spectrum mode");
            current_mode = SPECTRUM_MODE;
            mode_changed = true;
            mode_switched_this_cycle = true;
        }
    } else if (gesture != GES_NONE && gesture != GES_WAVE && gesture != GES_CLOCKWISE && gesture != GES_ANTICLOCKWISE) { 
        Serial.print("Gesture detected: ");
        printGestureName(gesture);
    }

    if (mode_switched_this_cycle && dma_display) {
        dma_display->clearScreen();
        String mode_text = (current_mode == WEATHER_CLOCK_MODE) ? "Weather" : "Spectrum";
        uint16_t text_color = (current_mode == WEATHER_CLOCK_MODE) ? hsv_to_panel_color(120,1,1) : hsv_to_panel_color(200,1,1); 
        dma_display->setTextSize(1);
        dma_display->setTextColor(text_color);
        int16_t x1, y1; uint16_t w, h;
        dma_display->getTextBounds(mode_text, 0, 0, &x1, &y1, &w, &h);
        dma_display->setCursor((PANEL_WIDTH - w) / 2, (PANEL_HEIGHT - h) / 2);
        dma_display->print(mode_text);
        dma_display->flipDMABuffer();
        delay(1000); 
    }
}

void update_time() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      strcpy(timeStringBuff, "No Time");
      return;
    }
    sprintf(timeStringBuff, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    // Avoid overwriting if it was already "No WiFi" from setup
    if (strcmp(timeStringBuff, "No WiFi") != 0) {
        strcpy(timeStringBuff, "No Time");
    }
  }
}

void fetch_weather_data() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = OPENWEATHERMAP_API_URL_BASE + "?q=" + OPENWEATHERMAP_CITY +
                     "&appid=" + OPENWEATHERMAP_API_KEY + "&units=metric&lang=zh_cn"; 
        
        Serial.print("Fetching weather data from: "); Serial.println(url);
        http.begin(url);
        int httpCode = http.GET();

        if (httpCode > 0) {
            if (httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                DynamicJsonDocument doc(1024); 
                DeserializationError error = deserializeJson(doc, payload);

                if (error) {
                    Serial.print(F("deserializeJson() failed: "));
                    Serial.println(error.f_str());
                    weather_description = "JSON Err";
                    return;
                }

                JsonObject weather_0 = doc["weather"][0];
                String raw_desc = weather_0["description"].as<String>(); // 获取原始中文描述
                weather_icon_code = weather_0["icon"].as<String>();
                
                JsonObject main = doc["main"];
                temperature = main["temp"].as<float>();

                Serial.println("Raw weather_description from API: " + raw_desc);

                // --- 中文到英文的映射 --- 
                String display_desc = raw_desc; // 默认为原始描述，以防没有匹配
                if (raw_desc == "晴") {
                    display_desc = "Sunny";
                } else if (raw_desc == "多云") {
                    display_desc = "Cloudy";
                } else if (raw_desc == "少云" || raw_desc == "晴间多云") {
                    display_desc = "P Cloudy"; // Partly Cloudy
                } else if (raw_desc == "阴") {
                    display_desc = "Overcast";
                } else if (raw_desc.indexOf("雨") != -1) { // 包含"雨"字
                    if (raw_desc.indexOf("雷") != -1) display_desc = "T-Storm"; // Thunderstorm
                    else if (raw_desc.indexOf("小") != -1) display_desc = "L Rain"; // Light Rain
                    else if (raw_desc.indexOf("中") != -1) display_desc = "M Rain"; // Moderate Rain
                    else if (raw_desc.indexOf("大") != -1) display_desc = "H Rain"; // Heavy Rain
                    else display_desc = "Rain";
                } else if (raw_desc.indexOf("雪") != -1) { // 包含"雪"字
                    display_desc = "Snow";
                } else if (raw_desc == "薄雾" || raw_desc == "雾") {
                    display_desc = "Mist";
                } else if (raw_desc == "霾") {
                    display_desc = "Haze";
                }
                // 更新全局变量
                weather_description = display_desc;

                Serial.printf("Mapped Weather: %s, Temp: %.1fC, Icon: %s\n", weather_description.c_str(), temperature, weather_icon_code.c_str());
                last_weather_update = millis(); 

            } else {
                Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
                weather_description = "HTTP Err";
            }
        } else {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
            weather_description = "Connect Err";
        }
        http.end();
    } else {
        Serial.println("WiFi not connected. Cannot fetch weather.");
        weather_description = "No WiFi";
    }
}

void display_spectrum() {
  size_t bytes_actually_read = 0;
  int32_t i2s_read_buff[FFT_SAMPLES_COUNT];
  const size_t buffer_size_in_bytes = FFT_SAMPLES_COUNT * sizeof(int32_t);
  esp_err_t result = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buff, buffer_size_in_bytes, &bytes_actually_read, pdMS_TO_TICKS(30));

  if (result == ESP_OK && bytes_actually_read == buffer_size_in_bytes) {
    for (int i = 0; i < FFT_SAMPLES_COUNT; i++) {
      vReal[i] = (float)(i2s_read_buff[i] >> 8); 
      vImag[i] = 0.0f;
    }
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    for (int band_idx = 0; band_idx < NUM_BANDS; band_idx++) {
        float max_amplitude_in_band = 0.0f;
        int start_bin = band_start_bins[band_idx];
        int end_bin = band_end_bins[band_idx];
        if (start_bin > 0 && start_bin < USEFUL_FFT_BINS && end_bin >= start_bin && end_bin < USEFUL_FFT_BINS) { 
            for (int k = start_bin; k <= end_bin; k++) {
                 if (vReal[k] > max_amplitude_in_band) {
                    max_amplitude_in_band = vReal[k]; 
                 }
            }
        }
        float representative_amplitude = max_amplitude_in_band;
        float current_log_amp = 0.0f;
        if (representative_amplitude > NOISE_FLOOR) {
          current_log_amp = log10f(representative_amplitude);
          if (isinf(current_log_amp) || isnan(current_log_amp)) current_log_amp = 0.0f;}
        float normalized_amp = 0.0f;
        if ((MAX_AMP_LOG - MIN_AMP_LOG) > 0.001f) {
           normalized_amp = (current_log_amp - MIN_AMP_LOG) / (MAX_AMP_LOG - MIN_AMP_LOG);}
        normalized_amp = constrain(normalized_amp * BAR_SENSITIVITY, 0.0f, 1.0f);
        float target_height;
        float screen_max_pixels = (float)PANEL_HEIGHT - 1.0f;
        float height_at_pivot = screen_max_pixels * HEIGHT_AT_PIVOT_FACTOR;
        if (normalized_amp <= NORM_AMP_PIVOT) {
            if (NORM_AMP_PIVOT > 0.001f) {
                float normalized_portion_below_pivot = normalized_amp / NORM_AMP_PIVOT;
                int lut_index = constrain((int)(normalized_portion_below_pivot * (POW_LUT_SIZE - 1) + 0.5f), 0, POW_LUT_SIZE - 1);
                float height_factor_below_pivot = pow_lut_lower[lut_index];
                target_height = height_factor_below_pivot * height_at_pivot;
            } else { target_height = (normalized_amp > 0.001f) ? height_at_pivot : 0.0f; }}
        else {
            float remaining_norm_amp_allowance = 1.0f - NORM_AMP_PIVOT;
            float remaining_height_to_fill = screen_max_pixels - height_at_pivot;
            if (remaining_norm_amp_allowance > 0.001f && remaining_height_to_fill > 0.001f) {
                float normalized_excess = (normalized_amp - NORM_AMP_PIVOT) / remaining_norm_amp_allowance;
                normalized_excess = constrain(normalized_excess, 0.0f, 1.0f);
                int lut_index = constrain((int)(normalized_excess * (POW_LUT_SIZE - 1) + 0.5f), 0, POW_LUT_SIZE - 1);
                float additional_height_factor = pow_lut_upper[lut_index];
                target_height = height_at_pivot + additional_height_factor * remaining_height_to_fill;
            } else { 
                target_height = height_at_pivot;
                if (normalized_amp >= 0.99f && NORM_AMP_PIVOT < 0.99f) { 
                     target_height = screen_max_pixels;
                } else if (NORM_AMP_PIVOT >= 0.99f) { 
                     target_height = height_at_pivot; 
                }
            }
        }
        target_height = constrain(target_height, 0.0f, screen_max_pixels);
        float current_actual_bar_h = band_heights[band_idx];
        float current_velocity = band_velocities[band_idx];
        if (target_height >= current_actual_bar_h) {
             current_actual_bar_h = current_actual_bar_h * (1.0f - SMOOTHING_FACTOR_RISE) + target_height * SMOOTHING_FACTOR_RISE;
            current_velocity = 0.0f; }
        else {
            current_velocity += GRAVITY_ACCELERATION;
            if (current_velocity > MAX_FALL_SPEED) current_velocity = MAX_FALL_SPEED;
            if (current_velocity < INITIAL_FALL_VELOCITY && current_actual_bar_h > target_height + INITIAL_FALL_VELOCITY ) {
                 current_velocity = INITIAL_FALL_VELOCITY; }
            current_actual_bar_h -= current_velocity;
            if (current_actual_bar_h < target_height) {
                current_actual_bar_h = target_height;
                current_velocity = 0.0f; }}
        if (current_actual_bar_h <= 0.01f) {
            current_actual_bar_h = 0.0f;
            current_velocity = 0.0f; }
        band_velocities[band_idx] = current_velocity;
        band_heights[band_idx] = constrain(current_actual_bar_h, 0.0f, screen_max_pixels);
        if (band_heights[band_idx] >= peak_heights[band_idx] - 0.01f) {
            peak_heights[band_idx] = band_heights[band_idx];
            peak_timers[band_idx] = millis(); }
        else {
            if (millis() - peak_timers[band_idx] > PEAK_FALL_DELAY) {
                peak_heights[band_idx] = max(0.0f, peak_heights[band_idx] - PEAK_FALL_RATE_PIXELS_PER_FRAME);}}
        if (peak_heights[band_idx] < band_heights[band_idx]) { 
            peak_heights[band_idx] = band_heights[band_idx]; }
    }
  } else if (result == ESP_ERR_TIMEOUT || bytes_actually_read == 0) { 
     for (int i = 0; i < NUM_BANDS; i++) {
        float current_actual_bar_h = band_heights[i];
        float current_velocity = band_velocities[i];
        current_velocity += GRAVITY_ACCELERATION;
        if (current_velocity > MAX_FALL_SPEED) current_velocity = MAX_FALL_SPEED;
        if (current_actual_bar_h > 0.01f && current_velocity < INITIAL_FALL_VELOCITY) current_velocity = INITIAL_FALL_VELOCITY;
        current_actual_bar_h -= current_velocity;
        if (current_actual_bar_h <= 0.01f) { current_actual_bar_h = 0.0f; current_velocity = 0.0f; }
        band_velocities[i] = current_velocity;
        band_heights[i] = constrain(current_actual_bar_h, 0.0f, (float)(PANEL_HEIGHT - 1));
        if (millis() - peak_timers[i] > PEAK_FALL_DELAY) { 
            peak_heights[i] = max(0.0f, peak_heights[i] - PEAK_FALL_RATE_PIXELS_PER_FRAME);
        }
         if (peak_heights[i] < band_heights[i]) peak_heights[i] = band_heights[i]; 
     }
  } else { 
    Serial.printf("I2S Read Error: %d\n", result);
    for (int i = 0; i < NUM_BANDS; i++) { 
        band_heights[i] *= 0.8; peak_heights[i] *= 0.8; 
        if (band_heights[i] < 0.1) band_heights[i] = 0;
        if (peak_heights[i] < 0.1) peak_heights[i] = 0;
        band_velocities[i] = 0.0f;
    }
  }

  if (dma_display) {
    dma_display->clearScreen();

    int bar_width_ideal = PANEL_WIDTH / NUM_BANDS;
    if (bar_width_ideal < 1) bar_width_ideal = 1;

    for (int screen_band_idx = 0; screen_band_idx < NUM_BANDS; screen_band_idx++) {
      int x = screen_band_idx * bar_width_ideal;
      int data_idx = NUM_BANDS - 1 - screen_band_idx; 
      int current_bar_width = bar_width_ideal;
      if (x + current_bar_width > PANEL_WIDTH) {
          current_bar_width = PANEL_WIDTH - x; }
      if (current_bar_width <= 0) continue; 

      int h_int = (int)roundf(band_heights[data_idx]);
      uint16_t bar_color = precomputed_bar_colors[data_idx];
      if (h_int > 0) {
        dma_display->fillRect(x, PANEL_HEIGHT - h_int, current_bar_width, h_int, bar_color); }
      
      int peak_h_int = (int)roundf(peak_heights[data_idx]);
      if (peak_h_int > 0 && peak_h_int >= h_int) { 
         uint16_t peak_color_to_use = precomputed_peak_color;
         int peak_y = PANEL_HEIGHT - 1 - peak_h_int;
         peak_y = max(0, peak_y); 
         peak_y = min(peak_y, PANEL_HEIGHT -1);
         dma_display->drawFastHLine(x, peak_y, current_bar_width, peak_color_to_use); 
      }
    }
    dma_display->flipDMABuffer();
  }
}

// 新增：分区大号时钟显示，主时钟只显示HH:MM，秒数单独小号显示
void draw_bold_clock(int16_t x, int16_t y, const String& time_str, uint16_t text_color, uint16_t outline_color, uint8_t size) {
    for (int8_t dx = -1; dx <= 1; dx++) {
        for (int8_t dy = -1; dy <= 1; dy++) {
            if (dx != 0 || dy != 0) {
                dma_display->setTextColor(outline_color);
                dma_display->setCursor(x + dx, y + dy);
                dma_display->setTextSize(size);
                dma_display->print(time_str);
            }
        }
    }
    dma_display->setTextColor(text_color);
    dma_display->setCursor(x, y);
    dma_display->setTextSize(size);
    dma_display->print(time_str);
}

void display_weather_clock() {
    bool needs_display_update = force_redraw_weather_clock; 
    force_redraw_weather_clock = false; 

    static unsigned long last_ntp_sync_attempt = 0;
    static unsigned long last_weather_api_call_attempt = 0;
    static String last_hhmm = "";
    static String last_ss = "";
    static unsigned long fade_anim_start = 0;
    const unsigned long fade_anim_ms = 400;
    static String last_minute = "";

    // 更新时间字符串
    if (millis() - last_ntp_sync_attempt > 950) { 
        update_time();
        last_ntp_sync_attempt = millis();
    }

    // 获取天气数据
    bool weather_data_seems_stale = (weather_icon_code == "" || weather_icon_code == "Loading...");
    if (WiFi.status() == WL_CONNECTED && (millis() - last_weather_api_call_attempt > WEATHER_UPDATE_INTERVAL || weather_data_seems_stale) ) {
        fetch_weather_data(); 
        last_weather_api_call_attempt = millis(); 
        needs_display_update = true; 
    }

    // 拆分时分秒
    String cur_time = String(timeStringBuff);
    String cur_hhmm = cur_time.substring(0,5); // HH:MM
    String cur_ss = cur_time.length() >= 8 ? cur_time.substring(6,8) : "00";
    String cur_minute = cur_time.substring(3,5); // MM

    // 检测分钟变化，触发动画
    if (last_minute != cur_minute) {
        fade_anim_start = millis();
        last_minute = cur_minute;
    }

    // 只要时分或强制刷新变化就全屏重绘，否则只重绘秒区
    if (last_hhmm != cur_hhmm || needs_display_update) {
        last_hhmm = cur_hhmm;
        last_ss = cur_ss;
    dma_display->clearScreen(); 
    dma_display->setTextWrap(false);

        // 主时钟渐变亮度动画
        float fade_v = 1.0f;
        if (fade_anim_start > 0) {
            unsigned long anim_elapsed = millis() - fade_anim_start;
            if (anim_elapsed < fade_anim_ms) {
                fade_v = 0.8f + 0.2f * (float(anim_elapsed) / fade_anim_ms);
            } else {
                fade_anim_start = 0;
                fade_v = 1.0f;
            }
        }
        uint16_t time_color = hsv_to_panel_color(200, 1, fade_v);
        uint16_t outline_color = hsv_to_panel_color(0, 0, 0.2);
        uint8_t clock_size = 2;
        int16_t x1, y1; uint16_t w, h;
        dma_display->setTextSize(clock_size);
        dma_display->getTextBounds(cur_hhmm, 0, 0, &x1, &y1, &w, &h);
        int16_t clock_x = (PANEL_WIDTH - w) / 2;
        int16_t clock_y = 0 + 3;
        draw_bold_clock(clock_x, clock_y, cur_hhmm, time_color, outline_color, clock_size);
        // 温度
    String temp_num_str = String(temperature, 0);
    uint16_t temp_color = hsv_to_panel_color(60,1,1);
    dma_display->setTextColor(temp_color);
    dma_display->setTextSize(1);
    int16_t x1_temp_num, y1_temp_num; uint16_t w_temp_num, h_temp_num;
    dma_display->getTextBounds(temp_num_str, 0, 0, &x1_temp_num, &y1_temp_num, &w_temp_num, &h_temp_num);
        uint8_t degree_radius = 1;
        uint8_t spacing_after_num = 1;
        uint8_t spacing_after_degree = 1;
    int16_t x1_c, y1_c; uint16_t w_c, h_c;
    dma_display->getTextBounds("C", 0, 0, &x1_c, &y1_c, &w_c, &h_c);
    int16_t temp_y_baseline = PANEL_HEIGHT - h_temp_num - 2;
        int16_t temp_start_x = 4;
    dma_display->setCursor(temp_start_x, temp_y_baseline);
    dma_display->print(temp_num_str);
        int16_t circle_cx = temp_start_x + w_temp_num + spacing_after_num + degree_radius;
        int16_t circle_cy = temp_y_baseline + degree_radius -1;
        dma_display->drawCircle(circle_cx, circle_cy, degree_radius, temp_color);
        dma_display->drawCircle(circle_cx, circle_cy, degree_radius-1, temp_color);
        dma_display->setCursor(circle_cx + degree_radius + spacing_after_degree, temp_y_baseline);
    dma_display->print("C");
        // 秒区先用黑色块覆盖
        dma_display->fillRect(PANEL_WIDTH-18, PANEL_HEIGHT-14, 18, 14, 0);
        // 天气图标 8x8，放在屏幕中下方，右下角往左25格，往上一格，保证完全可见且不与秒区重叠
        int icon_disp_width = 8;
        int icon_disp_height = 8;
        int internal_icon_idx = mapOwmIconToInternal(weather_icon_code);
        int icon_y_start = (PANEL_HEIGHT / 2) + ((PANEL_HEIGHT / 2) - icon_disp_height) - 2 - 1; // 再往上一格
        int icon_x_start = PANEL_WIDTH - icon_disp_width - 22 - 3; // 再往左三格
        drawWeatherIcon(icon_x_start, icon_y_start, 8, 8, internal_icon_idx, false);
    }
    // 秒数变化时只重绘右下角
    if (last_ss != cur_ss) {
        last_ss = cur_ss;
        // 覆盖秒区
        dma_display->fillRect(PANEL_WIDTH-18, PANEL_HEIGHT-14, 18, 14, 0);
        // 小号加粗秒数，整体向上移动1格
        uint16_t sec_color = hsv_to_panel_color(0,0,1);
        uint16_t sec_outline = hsv_to_panel_color(0,0,0.2);
        uint8_t sec_size = 1;
        int16_t x1s, y1s; uint16_t ws, hs;
        dma_display->setTextSize(sec_size);
        dma_display->getTextBounds(cur_ss, 0, 0, &x1s, &y1s, &ws, &hs);
        int16_t sec_x = PANEL_WIDTH - ws - 4;
        int16_t sec_y = PANEL_HEIGHT - hs - 3; // 向上移动1格
        draw_bold_clock(sec_x, sec_y, cur_ss, sec_color, sec_outline, sec_size);
    }
    dma_display->flipDMABuffer();
}

void loop() {
  if (!dma_display) return;

  handle_gesture_input(); 

  if (current_mode == SPECTRUM_MODE) {
    if (mode_changed) {
        Serial.println("Spectrum display active.");
        mode_changed = false;
    }
    display_spectrum(); 

  } else if (current_mode == WEATHER_CLOCK_MODE) {
    display_weather_clock(); 
    dma_display->flipDMABuffer(); // 天气时钟模式仍然在这里翻转，因为它自己按需绘制
  }
}

void printGestureName(Gesture gesture) {
  switch (gesture) {
    case GES_UP: Serial.println("UP (上)"); break;
    case GES_DOWN: Serial.println("DOWN (下)"); break;
    case GES_LEFT: Serial.println("LEFT (左)"); break;
    case GES_RIGHT: Serial.println("RIGHT (右)"); break;
    case GES_FORWARD: Serial.println("FORWARD (前)"); break;
    case GES_BACKWARD: Serial.println("BACKWARD (后)"); break;
    case GES_CLOCKWISE: Serial.println("CLOCKWISE (顺时针)"); break;
    case GES_ANTICLOCKWISE: Serial.println("ANTI-CLOCKWISE (逆时针)"); break;
    case GES_WAVE: Serial.println("WAVE (挥手)"); break;
    default: Serial.println("UNKNOWN GESTURE"); break;
  }
}