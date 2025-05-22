#include <Arduino.h>
#include "driver/i2s.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ArduinoFFT.h>
#include <math.h>

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

// --- I2S 和 FFT 配置参数 ---
#define I2S_SAMPLE_RATE     (16000)
#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_32BIT
#define FFT_SAMPLES_COUNT   (256)

#define I2S_DMA_BUF_COUNT    (8)
#define I2S_DMA_BUF_LEN_SAMPLES (FFT_SAMPLES_COUNT) 

// --- FFT 对象和数据数组 ---
double vReal[FFT_SAMPLES_COUNT];
double vImag[FFT_SAMPLES_COUNT];
ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES_COUNT, (double)I2S_SAMPLE_RATE);

// --- 频谱可视化参数 ---
#define NUM_BANDS (PANEL_WIDTH / 2) // *** 改回32个频段 (两列一个整体) ***

// !! 下面的参数基于 vReal[i] >> 8 和 max_amplitude_in_band，并且 NUM_BANDS = 32
// !! 这些值是你需要根据实际串口输出精细校准的起点 !!
#define NOISE_FLOOR 20000          // **初始猜测 (>>8, 32频段, max_in_band)**
#define MAX_AMP_LOG 6.0f           // **初始猜测**
#define MIN_AMP_LOG 4.2f           // **初始猜测**
#define BAR_SENSITIVITY 1.0f       // 从1.0开始

float band_heights[NUM_BANDS];    // 数组大小现在是 32
float peak_heights[NUM_BANDS];    // 数组大小现在是 32
unsigned long peak_timers[NUM_BANDS]; // 数组大小现在是 32
float band_velocities[NUM_BANDS]; // 数组大小现在是 32

#define PEAK_FALL_DELAY 200         
#define PEAK_FALL_RATE 0.25f        

// --- 主条行为参数 ---
#define SMOOTHING_FACTOR_RISE 0.85f 
#define GRAVITY_ACCELERATION 0.12f   
#define MAX_FALL_SPEED 2.0f        
#define INITIAL_FALL_VELOCITY 0.05f

// --- 非线性高度映射参数 ---
#define NORM_AMP_PIVOT 0.65f          // normalized_amp 的转折点 (0-1)
#define HEIGHT_AT_PIVOT_FACTOR 0.75f // 当 normalized_amp 达到转折点时，屏幕高度达到的比例 (0-1)
#define UPPER_HEIGHT_EXPONENT 0.4f   // 小于1的指数，使 normalized_amp 在后半段的增长对高度的贡献减小 (即后半段需求变大)

// --- 调试开关 ---
#define DEBUG_SERIAL_LEVEL_1 
// #define DEBUG_SERIAL_LEVEL_2 
#ifdef DEBUG_SERIAL_LEVEL_2
static unsigned long last_print_time_detail = 0;
#endif

// ... (setup_hub75_display, setup_i2s_microphone, hsv_to_panel_color 函数与之前版本相同) ...
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
  dma_display->setBrightness8(30);
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

  Serial.printf("Attempting I2S driver install with: SR=%d, BPS=%d, DMA_Count=%d, DMA_Len_Samples=%d, APLL=%s\n",
                i2s_config.sample_rate, (int)i2s_config.bits_per_sample, i2s_config.dma_buf_count, i2s_config.dma_buf_len, i2s_config.use_apll ? "true" : "false");

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
  Serial.println("--- I2S Configured for FFT_SAMPLES_COUNT ---");
}

uint16_t hsv_to_panel_color(float h, float s, float v) {
    float r_f, g_f, b_f;
    int i = floor(h / 60.0);
    float f = (h / 60.0) - i;
    float p = v * (1.0 - s);
    float q = v * (1.0 - s * f);
    float t = v * (1.0 - s * (1.0 - f));
    switch (i % 6) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        case 5: r_f = v; g_f = p; b_f = q; break;
        default: r_f = 0; g_f = 0; b_f = 0; break;
    }
    uint8_t r4 = (uint8_t)(r_f * 15.99);
    uint8_t g4 = (uint8_t)(g_f * 15.99);
    uint8_t b4 = (uint8_t)(b_f * 15.99);
    if (dma_display) {
        return dma_display->color444(r4, g4, b4);
    }
    return 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setup_hub75_display();
  setup_i2s_microphone();

  for (int i = 0; i < NUM_BANDS; i++) { // NUM_BANDS 现在是 32
    band_heights[i] = 0;
    peak_heights[i] = 0;
    peak_timers[i] = 0;
    band_velocities[i] = 0.0f;
  }

  Serial.println("Setup complete. Starting FFT Spectrum Display (32 BANDS).");
  if (dma_display) {
      dma_display->clearScreen();
      dma_display->setCursor(1, 8);
      dma_display->setTextColor(hsv_to_panel_color(120, 1, 1));
      dma_display->print("Spectrum ON");
      dma_display->flipDMABuffer();
      delay(1500);
  }
}


void loop() {
  size_t bytes_actually_read = 0; 
  int32_t i2s_read_buff[FFT_SAMPLES_COUNT]; 
  const size_t buffer_size_in_bytes = FFT_SAMPLES_COUNT * sizeof(int32_t);

  esp_err_t result = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buff, buffer_size_in_bytes, &bytes_actually_read, pdMS_TO_TICKS(200));

#ifdef DEBUG_SERIAL_LEVEL_1
  static unsigned long last_print_time_i2s = 0;
  bool print_this_cycle = (millis() - last_print_time_i2s > 1000); 
  if (print_this_cycle) {
    Serial.printf("I2S Read: result=%s, bytes_read=%u, expected=%u | ", esp_err_to_name(result), bytes_actually_read, buffer_size_in_bytes);
    last_print_time_i2s = millis();
  }
#endif

  if (result == ESP_OK && bytes_actually_read == buffer_size_in_bytes) {
    for (int i = 0; i < FFT_SAMPLES_COUNT; i++) {
      vReal[i] = (double)(i2s_read_buff[i] >> 8); 
      vImag[i] = 0.0;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude(); 

    int useful_fft_bins = FFT_SAMPLES_COUNT / 2; // 128
    // points_per_band_ideal 现在是 128 / 32 = 4
    int points_per_band_ideal = useful_fft_bins / NUM_BANDS; 
    if (points_per_band_ideal == 0) points_per_band_ideal = 1;

    for (int band_idx = 0; band_idx < NUM_BANDS; band_idx++) { // 循环 NUM_BANDS (32) 次
        double max_amplitude_in_band = 0.0; 

        int start_fft_bin = band_idx * points_per_band_ideal + 1;
        int end_fft_bin = start_fft_bin + points_per_band_ideal - 1;

        for (int k = start_fft_bin; k <= end_fft_bin; k++) {
            if (k < useful_fft_bins) {
                if (vReal[k] > max_amplitude_in_band) {
                    max_amplitude_in_band = vReal[k];
                }
            } else {
                break; 
            }
        }
        double representative_amplitude = max_amplitude_in_band;
        
        float current_log_amp = 0;
        if (representative_amplitude > NOISE_FLOOR) { 
          current_log_amp = log10(representative_amplitude);
          if (isinf(current_log_amp) || isnan(current_log_amp)) current_log_amp = 0;
        }

#ifdef DEBUG_SERIAL_LEVEL_2
        // ... (串口打印逻辑保持不变)
#elif defined(DEBUG_SERIAL_LEVEL_1)
        if (print_this_cycle && band_idx == 0 && result == ESP_OK) { 
             Serial.printf("B0: rep_A=%.0f, log_A=%.2f, vel=%.2f\n", representative_amplitude, current_log_amp, band_velocities[band_idx]);
        } else if (print_this_cycle && band_idx == 0 && result != ESP_OK) { 
            Serial.println();
        }
#endif
        float normalized_amp = 0;
        if ((MAX_AMP_LOG - MIN_AMP_LOG) > 0.001f) { 
           normalized_amp = (current_log_amp - MIN_AMP_LOG) / (MAX_AMP_LOG - MIN_AMP_LOG);
        }
        normalized_amp = constrain(normalized_amp * BAR_SENSITIVITY, 0.0, 1.0);
        
        // --- 非线性高度映射 (方法C) ---
        float target_height;
        float screen_max_pixels = (float)PANEL_HEIGHT - 1.0f;
        float height_at_pivot = screen_max_pixels * HEIGHT_AT_PIVOT_FACTOR;
        
        if (normalized_amp <= NORM_AMP_PIVOT) {
            if (NORM_AMP_PIVOT > 0.001f) { //避免除以0
                target_height = (normalized_amp / NORM_AMP_PIVOT) * height_at_pivot;
            } else {
                target_height = (normalized_amp > 0) ? height_at_pivot : 0; // 如果 pivot 是0，直接判断
            }
        } else {
            float remaining_norm_amp_allowance = 1.0f - NORM_AMP_PIVOT;
            float remaining_height_to_fill = screen_max_pixels - height_at_pivot;
            if (remaining_norm_amp_allowance > 0.001f && remaining_height_to_fill >= 0.0f) { // remaining_height_to_fill 可以为0
                float normalized_excess = (normalized_amp - NORM_AMP_PIVOT) / remaining_norm_amp_allowance;
                normalized_excess = constrain(normalized_excess, 0.0f, 1.0f); // 确保在0-1之间

                // 应用幂函数使后半段需求变大 (exponent < 1)
                float additional_height_factor = pow(normalized_excess, UPPER_HEIGHT_EXPONENT); 
                target_height = height_at_pivot + additional_height_factor * remaining_height_to_fill;
            } else { // 如果没有剩余的normalized_amp范围或高度范围去填充
                target_height = height_at_pivot; 
            }
        }
        target_height = constrain(target_height, 0.0f, screen_max_pixels);
        // --- 非线性高度映射结束 ---


        float current_actual_bar_h = band_heights[band_idx];
        float current_velocity = band_velocities[band_idx];

        if (target_height >= current_actual_bar_h) {
             current_actual_bar_h = current_actual_bar_h * (1.0 - SMOOTHING_FACTOR_RISE) + target_height * SMOOTHING_FACTOR_RISE;
            current_velocity = 0; 
        } else { 
            current_velocity += GRAVITY_ACCELERATION;
            if (current_velocity > MAX_FALL_SPEED) {
                current_velocity = MAX_FALL_SPEED;
            }
             if (current_velocity < INITIAL_FALL_VELOCITY && current_actual_bar_h > target_height + INITIAL_FALL_VELOCITY ) { 
                 current_velocity = INITIAL_FALL_VELOCITY;
             }
            current_actual_bar_h -= current_velocity;

            if (current_actual_bar_h < target_height) { 
                current_actual_bar_h = target_height;   
                current_velocity = 0;                 
            }
        }
        if (current_actual_bar_h <= 0) {
            current_actual_bar_h = 0;
            current_velocity = 0;
        }

        band_velocities[band_idx] = current_velocity; 
        band_heights[band_idx] = constrain(current_actual_bar_h, 0.0f, (float)(PANEL_HEIGHT - 1));

        if (band_heights[band_idx] >= peak_heights[band_idx]) {
            peak_heights[band_idx] = band_heights[band_idx];
            peak_timers[band_idx] = millis(); 
        } else {
            if (millis() - peak_timers[band_idx] > PEAK_FALL_DELAY) {
                peak_heights[band_idx] = max(0.0f, peak_heights[band_idx] - PEAK_FALL_RATE * (PANEL_HEIGHT / 10.0f));
            }
        }
        if (peak_heights[band_idx] < band_heights[band_idx]) {
            peak_heights[band_idx] = band_heights[band_idx];
        }
    }
  } else if (result == ESP_ERR_TIMEOUT) { 
    // ... (超时处理逻辑与上一版本相同，应用重力) ...
     for (int i = 0; i < NUM_BANDS; i++) {
        float current_actual_bar_h = band_heights[i];
        float current_velocity = band_velocities[i];

        current_velocity += GRAVITY_ACCELERATION;
        if (current_velocity > MAX_FALL_SPEED) {
            current_velocity = MAX_FALL_SPEED;
        }
         if (current_velocity < INITIAL_FALL_VELOCITY && current_actual_bar_h > INITIAL_FALL_VELOCITY ) { 
             current_velocity = INITIAL_FALL_VELOCITY;
         }
        current_actual_bar_h -= current_velocity;
        
        if (current_actual_bar_h <= 0) {
            current_actual_bar_h = 0;
            current_velocity = 0;
        }
        band_velocities[i] = current_velocity;
        band_heights[i] = constrain(current_actual_bar_h, 0.0f, (float)(PANEL_HEIGHT - 1));

        if (millis() - peak_timers[i] > PEAK_FALL_DELAY) {
            peak_heights[i] = max(0.0f, peak_heights[i] - PEAK_FALL_RATE * (PANEL_HEIGHT/10.0f));
        }
     }
  } else { 
    // ... (错误处理逻辑与上一版本相同) ...
    if (bytes_actually_read > 0 || result != ESP_OK) {
        Serial.printf("I2S Read Error/Incomplete: %s, bytes_read: %u, expected: %u\n", esp_err_to_name(result), bytes_actually_read, buffer_size_in_bytes);
    }
    for (int i = 0; i < NUM_BANDS; i++) {
        band_heights[i] = 0;
        peak_heights[i] = 0;
        band_velocities[i] = 0.0f;
    }
  }

  // --- 屏幕绘制逻辑 (32频段，高音在左) ---
  if (dma_display) {
    dma_display->clearScreen();
    int bar_width_ideal = PANEL_WIDTH / NUM_BANDS; // 64 / 32 = 2
    if (bar_width_ideal < 1) bar_width_ideal = 1;

    for (int screen_band_idx = 0; screen_band_idx < NUM_BANDS; screen_band_idx++) {
      int x = screen_band_idx * bar_width_ideal; 
      int data_idx = NUM_BANDS - 1 - screen_band_idx; 

      int current_bar_width = bar_width_ideal;

      int h_int = (int)round(band_heights[data_idx]);
      float hue = (float)data_idx / NUM_BANDS * 240.0f; 
      uint16_t bar_color = hsv_to_panel_color(hue, 1.0f, 1.0f);

      if (h_int > 0) {
        dma_display->fillRect(x, PANEL_HEIGHT - h_int, current_bar_width, h_int, bar_color);
      }

      int peak_h_int = (int)round(peak_heights[data_idx]);
      if (peak_h_int > 0) {
         uint16_t peak_color = hsv_to_panel_color(hue, 0.7f, 1.0f);
         dma_display->drawFastHLine(x, PANEL_HEIGHT - 1 - peak_h_int, current_bar_width, peak_color);
      }
    }
    dma_display->flipDMABuffer();
  }
  delay(0); 
}