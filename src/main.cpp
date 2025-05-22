#include <Arduino.h>
#include "driver/i2s.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h> // 用于 HUB75 LED 屏幕
#include <ArduinoFFT.h> // 用于频谱分析

// --- HUB75 LED 屏幕参数配置 ---
#define PANEL_WIDTH   64
#define PANEL_HEIGHT  32
#define PANELS_NUMBER 1

// --- HUB75 ESP32-S3 引脚配置 ---
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

// --- INMP441 I2S 引脚配置 ---
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

// --- 频谱可视化参数 (进行了调整) ---
#define NUM_BANDS (PANEL_WIDTH / 2) // 32个频段
#define NOISE_FLOOR 300            // 尝试降低噪声门限 (原1000) - 需要微调
#define MAX_AMP_LOG 7.0f           // 尝试降低对数幅值上限 (原12.0f) - 需要微调
#define MIN_AMP_LOG 1.5f            // 尝试降低对数幅值下限 (原4.0f) - 需要微调
#define BAR_SENSITIVITY 2.5f        // 尝试增加灵敏度因子 (原1.5f) - 需要微调

float band_heights[NUM_BANDS];
float peak_heights[NUM_BANDS];
unsigned long peak_timers[NUM_BANDS];
#define PEAK_FALL_DELAY 500
#define PEAK_FALL_RATE 0.1f
#define SMOOTHING_FACTOR 0.25f       // 可以尝试调整平滑度 (原0.3f)

// --- 调试开关 ---
// #define DEBUG_SERIAL_LEVEL_1 // 取消注释以打印 I2S 读取和 FFT 概况
// #define DEBUG_SERIAL_LEVEL_2 // 取消注释以打印每个频段的详细数据 (信息量大)


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
  dma_display->setBrightness8(30); // 保持较低亮度测试
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

  for (int i = 0; i < NUM_BANDS; i++) {
    band_heights[i] = 0;
    peak_heights[i] = 0;
    peak_timers[i] = 0;
  }

  Serial.println("Setup complete. Starting FFT Spectrum Display.");
  if (dma_display) {
      dma_display->clearScreen();
      dma_display->setCursor(1, 8); // 调整文本位置
      dma_display->setTextColor(hsv_to_panel_color(120, 1, 1)); // 绿色
      dma_display->print("Spectrum ON");
      dma_display->flipDMABuffer();
      delay(1500);
  }
}

void loop() {
  size_t bytes_read = 0;
  int32_t i2s_read_buff[FFT_SAMPLES_COUNT]; 
  const size_t bytes_to_read = FFT_SAMPLES_COUNT * sizeof(int32_t);

  esp_err_t result = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buff, bytes_to_read, &bytes_read, pdMS_TO_TICKS(200));

#ifdef DEBUG_SERIAL_LEVEL_1
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 1000) { // 每秒打印一次
    Serial.printf("I2S Read: result=%s, bytes_read=%u, expected=%u\n", esp_err_to_name(result), bytes_read, bytes_to_read);
    last_print_time = millis();
  }
#endif

  if (result == ESP_OK && bytes_read == bytes_to_read) {
    for (int i = 0; i < FFT_SAMPLES_COUNT; i++) {
      // 尝试不同的右移位数来调整输入信号的幅度
      // vReal[i] = (double)(i2s_read_buff[i] >> 8); // 原始：相当于取高16位，数值缩小256倍
      vReal[i] = (double)(i2s_read_buff[i] >> 6); // 尝试：数值缩小64倍，信号更强
      // vReal[i] = (double)(i2s_read_buff[i] >> 4); // 尝试：数值缩小16倍，信号更强
      // vReal[i] = (double)(i2s_read_buff[i]);      // 尝试：使用原始高24位，可能需要大幅调整NOISE_FLOOR等
      vImag[i] = 0.0;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude(); 

    int useful_fft_bins = FFT_SAMPLES_COUNT / 2;
    int points_per_band_ideal = useful_fft_bins / NUM_BANDS;
    if (points_per_band_ideal == 0) points_per_band_ideal = 1;

    for (int band_idx = 0; band_idx < NUM_BANDS; band_idx++) {
        double band_amplitude_sum = 0;
        int bins_in_this_band = 0;
        int start_fft_bin = band_idx * points_per_band_ideal + 1;
        int end_fft_bin = start_fft_bin + points_per_band_ideal - 1;

        for (int k = start_fft_bin; k <= end_fft_bin; k++) {
            if (k < useful_fft_bins) {
                band_amplitude_sum += vReal[k];
                bins_in_this_band++;
            } else {
                break; 
            }
        }
        double avg_amplitude = (bins_in_this_band > 0) ? (band_amplitude_sum / bins_in_this_band) : 0.0;
        
        float current_log_amp = 0;
        if (avg_amplitude > NOISE_FLOOR) {
          current_log_amp = log10(avg_amplitude);
          if (isinf(current_log_amp) || isnan(current_log_amp)) current_log_amp = 0;
        }

#ifdef DEBUG_SERIAL_LEVEL_2
        if (band_idx == 0 || band_idx == NUM_BANDS / 2 || band_idx == NUM_BANDS -1 ) { // 抽样打印部分频段数据
            if (millis() - last_print_time_detail > 500) { // 控制打印频率
                 Serial.printf("B%d: avg_A=%.0f, log_A=%.2f | ", band_idx, avg_amplitude, current_log_amp);
                 if(band_idx == NUM_BANDS -1) Serial.println();
                 last_print_time_detail = millis();
            }
        }
#elif defined(DEBUG_SERIAL_LEVEL_1)
        if (band_idx == 0 && millis() - last_print_time > 1000) { // 确保与上面的I2S打印错开
             Serial.printf("B0: avg_A=%.0f, log_A=%.2f\n", avg_amplitude, current_log_amp);
        }
#endif
        float normalized_amp = 0;
        if ((MAX_AMP_LOG - MIN_AMP_LOG) > 0.001f) { 
           normalized_amp = (current_log_amp - MIN_AMP_LOG) / (MAX_AMP_LOG - MIN_AMP_LOG);
        }
        normalized_amp = constrain(normalized_amp * BAR_SENSITIVITY, 0.0, 1.0);
        
        float target_height = normalized_amp * (PANEL_HEIGHT - 1); 

        band_heights[band_idx] = band_heights[band_idx] * (1.0 - SMOOTHING_FACTOR) + target_height * SMOOTHING_FACTOR;
        band_heights[band_idx] = constrain(band_heights[band_idx], 0, PANEL_HEIGHT - 1);

        if (band_heights[band_idx] >= peak_heights[band_idx]) {
            peak_heights[band_idx] = band_heights[band_idx];
            peak_timers[band_idx] = millis();
        } else {
            if (millis() - peak_timers[band_idx] > PEAK_FALL_DELAY) {
                peak_heights[band_idx] = max(0.0f, peak_heights[band_idx] - PEAK_FALL_RATE * (PANEL_HEIGHT / 10.0f));
            }
        }
        if (peak_heights[band_idx] < band_heights[band_idx]) peak_heights[band_idx] = band_heights[band_idx];
    }
  } else if (result == ESP_ERR_TIMEOUT) {
     for (int i = 0; i < NUM_BANDS; i++) {
        band_heights[i] *= (1.0 - SMOOTHING_FACTOR * 0.5); 
        band_heights[i] = constrain(band_heights[i], 0, PANEL_HEIGHT -1);
        if (millis() - peak_timers[i] > PEAK_FALL_DELAY) {
            peak_heights[i] = max(0.0f, peak_heights[i] - PEAK_FALL_RATE * (PANEL_HEIGHT/10.0f));
        }
     }
  } else {
    Serial.printf("I2S Read Error/Incomplete: %s, bytes_read: %u, expected: %u\n", esp_err_to_name(result), bytes_read, bytes_to_read);
    for (int i = 0; i < NUM_BANDS; i++) {
        band_heights[i] = 0;
        peak_heights[i] = 0;
    }
  }

  if (dma_display) {
    dma_display->clearScreen();
    int bar_width_ideal = PANEL_WIDTH / NUM_BANDS;
    if (bar_width_ideal < 1) bar_width_ideal = 1;

    for (int i = 0; i < NUM_BANDS; i++) {
      int x = i * bar_width_ideal;
      int current_bar_width = (i == NUM_BANDS - 1) ? (PANEL_WIDTH - x) : bar_width_ideal; 
      if (x + current_bar_width > PANEL_WIDTH) current_bar_width = PANEL_WIDTH - x;

      int h_int = (int)round(band_heights[i]);
      float hue = (float)i / NUM_BANDS * 240.0f; 
      uint16_t bar_color = hsv_to_panel_color(hue, 1.0f, 1.0f);

      if (h_int > 0) {
        dma_display->fillRect(x, PANEL_HEIGHT - h_int, current_bar_width, h_int, bar_color);
      }

      int peak_h_int = (int)round(peak_heights[i]);
      if (peak_h_int > 0) {
         uint16_t peak_color = hsv_to_panel_color(hue, 0.7f, 1.0f);
         dma_display->drawFastHLine(x, PANEL_HEIGHT - 1 - peak_h_int, current_bar_width, peak_color);
      }
    }
    dma_display->flipDMABuffer();
  }
  delay(20);
}