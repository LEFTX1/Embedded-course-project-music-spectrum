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
#define FFT_SAMPLES_COUNT   (256)   // MODIFIED from 512

#define I2S_DMA_BUF_COUNT    (8)
#define I2S_DMA_BUF_LEN_SAMPLES (FFT_SAMPLES_COUNT) // MODIFIED to match FFT_SAMPLES_COUNT

// --- FFT 对象和数据数组 (使用 float 以提高性能) ---
float vReal[FFT_SAMPLES_COUNT];
float vImag[FFT_SAMPLES_COUNT];
ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLES_COUNT, (float)I2S_SAMPLE_RATE);

// --- 频谱可视化参数 ---
#define NUM_BANDS (PANEL_WIDTH) // MODIFIED for 1 pixel width per bar

// --- 分桶参数 (Linear Binning) ---
#define F_MIN_HZ 40.0f
int band_start_bins[NUM_BANDS];
int band_end_bins[NUM_BANDS];
const float FREQ_RESOLUTION = (float)I2S_SAMPLE_RATE / FFT_SAMPLES_COUNT;
const int USEFUL_FFT_BINS = FFT_SAMPLES_COUNT / 2; // Number of magnitude bins from FFT (0 to N/2 - 1)

// !! 校准参数 !!
#define NOISE_FLOOR 26000.0f
#define MAX_AMP_LOG 8.8f
#define MIN_AMP_LOG 4.3f
#define BAR_SENSITIVITY 1.0f

float band_heights[NUM_BANDS];
float peak_heights[NUM_BANDS];
unsigned long peak_timers[NUM_BANDS];
float band_velocities[NUM_BANDS];

#define PEAK_FALL_DELAY 150
#define PEAK_FALL_RATE_PIXELS_PER_FRAME 0.3f

// --- 主条行为参数 ---
#define SMOOTHING_FACTOR_RISE 0.80f
#define GRAVITY_ACCELERATION 0.15f
#define MAX_FALL_SPEED 2.5f
#define INITIAL_FALL_VELOCITY 0.05f

// --- 非线性高度映射参数 ---
#define NORM_AMP_PIVOT 0.50f
#define HEIGHT_AT_PIVOT_FACTOR 0.60f
#define UPPER_HEIGHT_EXPONENT 0.35f
#define LOWER_HEIGHT_EXPONENT 1.8f

// --- Precomputed Colors & LUTs ---
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

uint16_t hsv_to_panel_color(float h, float s, float v) {
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

void calculate_frequency_bins() { // Changed name to be generic, implementing LINEAR binning
  Serial.println("--- Calculating Linear Frequency Bins ---");
  float min_possible_freq = FREQ_RESOLUTION; // Freq of FFT bin 1 (e.g., 16000/256 = 62.5 Hz)
  float min_map_freq = max(F_MIN_HZ, min_possible_freq); // Actual lowest freq to start mapping from
  float max_map_freq = (float)I2S_SAMPLE_RATE / 2.0f;    // Nyquist frequency (e.g., 16000/2 = 8000 Hz)

  float total_freq_span_to_map = max_map_freq - min_map_freq;

  if (total_freq_span_to_map <= 0.001f) { // Check for non-positive or tiny span
      Serial.println("Error: Frequency span for binning is too small or invalid.");
      // Fallback: Distribute FFT bins as evenly as possible among display bands
      int bins_per_band = (USEFUL_FFT_BINS -1) / NUM_BANDS;
      if (bins_per_band < 1) bins_per_band = 1;
      for (int i = 0; i < NUM_BANDS; i++) {
          band_start_bins[i] = max(1, 1 + i * bins_per_band);
          band_end_bins[i] = max(1, band_start_bins[i] + bins_per_band - 1);
          if (i == NUM_BANDS -1) band_end_bins[i] = USEFUL_FFT_BINS - 1; // Last band takes the rest
          
          band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
          band_end_bins[i]   = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);
          if (band_end_bins[i] < band_start_bins[i]) band_end_bins[i] = band_start_bins[i];
      }
      Serial.println("Fallback binning applied.");
      return;
  }

  float linear_freq_step_per_band = total_freq_span_to_map / NUM_BANDS;

  for (int i = 0; i < NUM_BANDS; i++) {
    float freq_low_ideal = min_map_freq + ((float)i * linear_freq_step_per_band);
    float freq_high_ideal = min_map_freq + ((float)(i + 1) * linear_freq_step_per_band);
    
    int start_bin_ideal = roundf(freq_low_ideal / FREQ_RESOLUTION);
    int end_bin_ideal = roundf(freq_high_ideal / FREQ_RESOLUTION) - 1;

    if (i == 0) {
      band_start_bins[i] = max(1, start_bin_ideal);
    } else {
      band_start_bins[i] = max(start_bin_ideal, band_end_bins[i - 1] + 1);
    }
    
    band_end_bins[i] = max(band_start_bins[i], end_bin_ideal);
    
    band_start_bins[i] = constrain(band_start_bins[i], 1, USEFUL_FFT_BINS - 1);
    band_end_bins[i] = constrain(band_end_bins[i], 1, USEFUL_FFT_BINS - 1);

    if (band_start_bins[i] > band_end_bins[i]) {
      band_end_bins[i] = band_start_bins[i];
    }

    if (i == NUM_BANDS - 1) {
      band_end_bins[i] = USEFUL_FFT_BINS - 1;
      if (band_start_bins[i] > band_end_bins[i]) {
        band_start_bins[i] = band_end_bins[i];
      }
    }

    if (band_start_bins[i] >= USEFUL_FFT_BINS - 1 && i < NUM_BANDS - 1) {
      for (int j = i; j < NUM_BANDS; ++j) {
        band_start_bins[j] = USEFUL_FFT_BINS - 1;
        band_end_bins[j] = USEFUL_FFT_BINS - 1;
      }
      #ifdef DEBUG_SERIAL_LEVEL_2
      Serial.printf("Band %2d onwards mapped to last FFT bin (%d).\n", i, USEFUL_FFT_BINS - 1);
      #endif
      break; 
    }
    
    #ifdef DEBUG_SERIAL_LEVEL_2
    if (millis() - last_print_time_detail > 50 || i < 5 || i > NUM_BANDS - 5) { // Print first/last few and occasionally
        Serial.printf("Band %2d: Freq_ideal %.0f-%.0f Hz => FFT Bins %3d-%3d (Len %d)\n",
                      i, freq_low_ideal, freq_high_ideal,
                      band_start_bins[i], band_end_bins[i], band_end_bins[i] - band_start_bins[i] + 1);
        last_print_time_detail = millis();
    }
    #endif
  }

  Serial.println("--- Adjusted Low Frequency Bins (First 5 - Linear): ---");
  for (int i = 0; i < min(5, NUM_BANDS); ++i) {
    float f_low_actual = band_start_bins[i] * FREQ_RESOLUTION;
    float f_high_actual = (band_end_bins[i] + 1) * FREQ_RESOLUTION;
    Serial.printf("  Adj Band %2d: FFT Bins %3d-%3d (Actual Freq ~%.0f-%.0f Hz)\n",
                  i, band_start_bins[i], band_end_bins[i], f_low_actual, f_high_actual);
  }
  Serial.println("--- Linear Frequency Bins Calculated ---");
}


void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor
  Serial.println("Starting setup...");

  setup_hub75_display();
  setup_i2s_microphone();
  calculate_frequency_bins(); // This now calculates LINEAR bins

  // Precompute colors
  precomputed_peak_color = hsv_to_panel_color(0, 0.0f, 1.0f); // White
  for (int i = 0; i < NUM_BANDS; ++i) {
    float hue;
    if (NUM_BANDS > 1) {
        hue = ((float)i / (NUM_BANDS - 1)) * 240.0f; // Hue from Red (0) to Blue (240)
    } else { 
        hue = 0.0f; 
    }
    precomputed_bar_colors[i] = hsv_to_panel_color(hue, 1.0f, 1.0f);
  }

  // Populate LUTs for powf optimization
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

  Serial.println("Setup complete. Starting FFT Spectrum Display (Linear Bins).");
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
  esp_err_t result = i2s_read(I2S_PORT_NUM, (void*)i2s_read_buff, buffer_size_in_bytes, &bytes_actually_read, pdMS_TO_TICKS(100));


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

        // This check is important: only process if the bin range is valid and non-empty
        if (start_bin < USEFUL_FFT_BINS && end_bin >= start_bin && start_bin > 0) { // Ensure start_bin > 0
            for (int k = start_bin; k <= end_bin; k++) {
                 // Additional safety: ensure k is within vReal bounds, though 'constrain' should handle it.
                if (k < FFT_SAMPLES_COUNT) { // Technically k < USEFUL_FFT_BINS is enough here.
                    if (vReal[k] > max_amplitude_in_band) {
                        max_amplitude_in_band = vReal[k];
                    }
                }
            }
        }
        
        float representative_amplitude = max_amplitude_in_band;
        float current_log_amp = 0.0f;
        if (representative_amplitude > NOISE_FLOOR) {
          current_log_amp = log10f(representative_amplitude);
          if (isinf(current_log_amp) || isnan(current_log_amp)) current_log_amp = 0.0f;
        }

        float normalized_amp = 0.0f;
        if ((MAX_AMP_LOG - MIN_AMP_LOG) > 0.001f) { 
           normalized_amp = (current_log_amp - MIN_AMP_LOG) / (MAX_AMP_LOG - MIN_AMP_LOG);
        }
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
            } else { 
                target_height = (normalized_amp > 0) ? height_at_pivot : 0.0f; 
            }
        } else { 
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
            current_velocity = 0.0f; 
        } else { 
            current_velocity += GRAVITY_ACCELERATION;
            if (current_velocity > MAX_FALL_SPEED) current_velocity = MAX_FALL_SPEED;
            if (current_velocity < INITIAL_FALL_VELOCITY && current_actual_bar_h > target_height + INITIAL_FALL_VELOCITY ) { 
                 current_velocity = INITIAL_FALL_VELOCITY;
             }
            current_actual_bar_h -= current_velocity;
            if (current_actual_bar_h < target_height) { 
                current_actual_bar_h = target_height;   
                current_velocity = 0.0f;                 
            }
        }
        if (current_actual_bar_h <= 0.01f) { 
            current_actual_bar_h = 0.0f;
            current_velocity = 0.0f;
        }

        band_velocities[band_idx] = current_velocity;
        band_heights[band_idx] = constrain(current_actual_bar_h, 0.0f, screen_max_pixels);

        if (band_heights[band_idx] >= peak_heights[band_idx] - 0.01f) { 
            peak_heights[band_idx] = band_heights[band_idx];
            peak_timers[band_idx] = millis(); 
        } else { 
            if (millis() - peak_timers[band_idx] > PEAK_FALL_DELAY) {
                peak_heights[band_idx] = max(0.0f, peak_heights[band_idx] - PEAK_FALL_RATE_PIXELS_PER_FRAME);
            }
        }
        if (peak_heights[band_idx] < band_heights[band_idx]) {
            peak_heights[band_idx] = band_heights[band_idx];
        }
    }
  } else if (result == ESP_ERR_TIMEOUT) { 
     for (int i = 0; i < NUM_BANDS; i++) {
        float current_actual_bar_h = band_heights[i];
        float current_velocity = band_velocities[i];
        current_velocity += GRAVITY_ACCELERATION;
        if (current_velocity > MAX_FALL_SPEED) current_velocity = MAX_FALL_SPEED;
        if (current_velocity < INITIAL_FALL_VELOCITY && current_actual_bar_h > INITIAL_FALL_VELOCITY ) current_velocity = INITIAL_FALL_VELOCITY;
        current_actual_bar_h -= current_velocity;
        if (current_actual_bar_h <= 0.01f) { current_actual_bar_h = 0.0f; current_velocity = 0.0f; }
        band_velocities[i] = current_velocity;
        band_heights[i] = constrain(current_actual_bar_h, 0.0f, (float)(PANEL_HEIGHT - 1));
        if (millis() - peak_timers[i] > PEAK_FALL_DELAY) { peak_heights[i] = max(0.0f, peak_heights[i] - PEAK_FALL_RATE_PIXELS_PER_FRAME);}
     }
  } else { 
    for (int i = 0; i < NUM_BANDS; i++) {
        band_heights[i] = 0; peak_heights[i] = 0; band_velocities[i] = 0.0f;
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
          current_bar_width = PANEL_WIDTH - x;
      }
      
      int h_int = (int)roundf(band_heights[data_idx]);
      uint16_t bar_color = precomputed_bar_colors[data_idx]; 
      
      if (h_int > 0) {
        dma_display->fillRect(x, PANEL_HEIGHT - h_int, current_bar_width, h_int, bar_color);
      }

      int peak_h_int = (int)roundf(peak_heights[data_idx]);
      if (peak_h_int > 0) {
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