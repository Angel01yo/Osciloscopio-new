// ============================================================================
// OSCILOSCOPIO PROFESIONAL - RASPBERRY PI PICO (SDK C)
// Version 2.2 - Corregido y optimizado con USB OTG
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

// ============================================================================
// CONFIGURACION DEL CIRCUITO DE ENTRADA
// ============================================================================

#define R1_OHMS           900000.0f
#define R2_OHMS           100000.0f
#define DIVISOR_RATIO     ((R1_OHMS + R2_OHMS) / R2_OHMS)

#define R3_OHMS           100000.0f
#define R4_OHMS           100000.0f
#define VCC_VOLTAGE       3.3f
#define OFFSET_VOLTAGE    (VCC_VOLTAGE * R4_OHMS / (R3_OHMS + R4_OHMS))

#define ZENER_VOLTAGE     3.3f
#define R_SERIE_OHMS      1000.0f

#define VIN_MAX_POSITIVE  30.0f
#define VIN_MAX_NEGATIVE  -30.0f
#define VIN_MAX_PP        60.0f

#define ADC_VREF          3.3f
#define ADC_RESOLUTION    4096.0f
#define ADC_COUNTS_PER_V  (ADC_RESOLUTION / ADC_VREF)

// ============================================================================
// CONFIGURACION DEL SISTEMA
// ============================================================================

#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_I2C_ADDR       0x3C
#define OLED_I2C            i2c0
#define OLED_SCL_PIN        5
#define OLED_SDA_PIN        4
#define OLED_I2C_FREQ       400000

#define ADC_PIN             26
#define ADC_PIN_2           27
#define ADC_CHANNEL         0
#define ADC_CHANNEL_2       1
#define NUM_SAMPLES         256
#define SAMPLE_RATE         500000
#define ADC_CLOCK_DIV       ((48000000 / SAMPLE_RATE) - 1)

#define DMA_CHAN_ADC        0

#define BTN_VSCALE_PIN      14
#define BTN_TSCALE_PIN      15
#define BTN_PAUSE_PIN       16
#define BTN_MODE_PIN        17
#define BTN_TRIGGER_PIN     18
#define BTN_SAVE_PIN        19
#define BTN_OFFSET_PIN      20

#define WIFI_UART           uart1
#define WIFI_TX_PIN         8
#define WIFI_RX_PIN         9
#define WIFI_BAUDRATE       115200

#define USB_BUFFER_SIZE     4096

#define FLASH_TARGET_OFFSET (256 * 1024)
#define MAX_WAVEFORMS       50

#define FFT_SIZE            NUM_SAMPLES
#define FFT_HALF_SIZE       (FFT_SIZE / 2)

#define NUM_V_SCALES        8
#define NUM_T_SCALES        6

// ============================================================================
// USB OTG CONFIGURACION
// ============================================================================
#define USB_OTG_ENABLED     1
#define USB_SERIAL_BAUD     115200
#define USB_UPDATE_MS       100

// ============================================================================
// TIPOS DE DATOS
// ============================================================================

typedef enum {
    COUPLING_DC = 0,
    COUPLING_AC
} CouplingMode;

typedef enum {
    MODE_NORMAL = 0,
    MODE_XY,
    MODE_FFT,
    MODE_MEASUREMENTS
} DisplayMode;

typedef enum {
    TRIGGER_RISING = 0,
    TRIGGER_FALLING,
    TRIGGER_BOTH
} TriggerEdge;

typedef enum {
    TRIGGER_MODE_NORMAL = 0,
    TRIGGER_MODE_AUTO,
    TRIGGER_MODE_SINGLE
} TriggerMode;

typedef struct {
    float vpp;
    float vrms;
    float vdc;
    float vac;
    float frequency;
    float period;
    float duty_cycle;
    float rise_time;
    float fall_time;
    float max_voltage;
    float min_voltage;
} Measurements;

typedef struct {
    float freqs[FFT_HALF_SIZE];
    float mags[FFT_HALF_SIZE];
    float dominant_freq;
    float dominant_mag;
} FFTResult;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

static uint16_t adc_buffer[NUM_SAMPLES];
static uint16_t adc_buffer_ch2[NUM_SAMPLES];
static volatile bool adc_dma_done = false;

static volatile DisplayMode current_mode = MODE_NORMAL;
static volatile CouplingMode coupling_mode = COUPLING_DC;
static volatile uint8_t v_scale_idx = 4;
static volatile uint8_t t_scale_idx = 2;
static volatile bool running = true;

static volatile TriggerEdge trigger_edge = TRIGGER_RISING;
static volatile TriggerMode trigger_mode = TRIGGER_MODE_AUTO;
static volatile float trigger_level = 1.65f;
static volatile float trigger_hysteresis = 0.1f;
static volatile bool single_triggered = false;

static const float v_scales[NUM_V_SCALES] = {
    1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 200.0f
};

static const float t_scales[NUM_T_SCALES] = {0.1f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f};

static Measurements current_measurements = {0};
static FFTResult fft_result = {0};
static float cos_table[FFT_SIZE];
static float sin_table[FFT_SIZE];

static volatile uint32_t last_button_time = 0;
#define DEBOUNCE_MS     200

static char usb_tx_buffer[USB_BUFFER_SIZE];
static char usb_rx_buffer[USB_BUFFER_SIZE];
static volatile uint16_t usb_rx_idx = 0;

// Spinlock para sincronizacion segura entre cores
static spin_lock_t *adc_spin_lock = NULL;
static uint32_t adc_spin_lock_num = 0;

// ============================================================================
// FUNCIONES DE CONVERSION DE VOLTAJE
// ============================================================================

static inline float adc_raw_to_vin(uint16_t raw_adc) {
    float vadc = ((float)raw_adc / ADC_RESOLUTION) * ADC_VREF;
    float vdiv = vadc - OFFSET_VOLTAGE;
    float vin = vdiv * DIVISOR_RATIO;
    return vin;
}

static inline uint16_t vin_to_adc_raw(float vin) {
    float vdiv = vin / DIVISOR_RATIO;
    float vadc = vdiv + OFFSET_VOLTAGE;
    uint16_t raw = (uint16_t)((vadc / ADC_VREF) * ADC_RESOLUTION);
    if (raw > 4095) raw = 4095;
    return raw;
}

static inline bool is_voltage_in_range(float vin) {
    return (vin >= VIN_MAX_NEGATIVE && vin <= VIN_MAX_POSITIVE);
}

static void convert_buffer_to_voltage(uint16_t *raw, float *vin, int count) {
    for (int i = 0; i < count; i++) {
        vin[i] = adc_raw_to_vin(raw[i]);
    }
}

// ============================================================================
// DRIVER SSD1306
// ============================================================================

static uint8_t oled_buffer[(OLED_WIDTH * OLED_HEIGHT) / 8];

static void oled_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(OLED_I2C, OLED_I2C_ADDR, buf, 2, false);
}

static void oled_init(void) {
    sleep_ms(100);
    static const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F,
        0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA,
        0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_write_cmd(init_cmds[i]);
    }
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_show(void) {
    oled_write_cmd(0x21);
    oled_write_cmd(0);
    oled_write_cmd(OLED_WIDTH - 1);
    oled_write_cmd(0x22);
    oled_write_cmd(0);
    oled_write_cmd((OLED_HEIGHT / 8) - 1);
    for (int i = 0; i < sizeof(oled_buffer); i += 16) {
        uint8_t buf[17];
        buf[0] = 0x40;
        memcpy(&buf[1], &oled_buffer[i], 16);
        i2c_write_blocking(OLED_I2C, OLED_I2C_ADDR, buf, 17, false);
    }
}

static void oled_clear(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_pixel(int x, int y, bool color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    if (color) {
        oled_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        oled_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

static void oled_hline(int x, int y, int w, bool color) {
    for (int i = 0; i < w && (x + i) < OLED_WIDTH; i++) {
        oled_pixel(x + i, y, color);
    }
}

static void oled_vline(int x, int y, int h, bool color) {
    for (int i = 0; i < h && (y + i) < OLED_HEIGHT; i++) {
        oled_pixel(x, y + i, color);
    }
}

static void oled_line(int x0, int y0, int x1, int y1, bool color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        oled_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

static void oled_text(int x, int y, const char *str, bool color) {
    while (*str && x < OLED_WIDTH - 6) {
        for (int i = 0; i < 5 && *str; i++) {
            for (int j = 0; j < 7; j++) {
                oled_pixel(x + i, y + j, color);
            }
        }
        x += 6;
        str++;
    }
}

// ============================================================================
// ADC CON DMA
// ============================================================================

static void adc_dma_init(void) {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_gpio_init(ADC_PIN_2);

    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLOCK_DIV);
    adc_select_input(ADC_CHANNEL);

    dma_channel_config cfg = dma_channel_get_default_config(DMA_CHAN_ADC);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(
        DMA_CHAN_ADC,
        &cfg,
        adc_buffer,
        &adc_hw->fifo,
        NUM_SAMPLES,
        false
    );
}

static void adc_dma_start(void) {
    uint32_t irq_state = spin_lock_blocking(adc_spin_lock);
    adc_dma_done = false;
    spin_unlock(adc_spin_lock, irq_state);

    dma_channel_set_write_addr(DMA_CHAN_ADC, adc_buffer, false);
    dma_channel_set_trans_count(DMA_CHAN_ADC, NUM_SAMPLES, false);
    dma_channel_start(DMA_CHAN_ADC);
    adc_run(true);
}

static void adc_dma_wait(void) {
    dma_channel_wait_for_finish_blocking(DMA_CHAN_ADC);
    adc_run(false);
    adc_fifo_drain();

    uint32_t irq_state = spin_lock_blocking(adc_spin_lock);
    adc_dma_done = true;
    spin_unlock(adc_spin_lock, irq_state);
}

static void adc_sample_ch2(void) {
    adc_select_input(ADC_CHANNEL_2);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_buffer_ch2[i] = adc_read();
        sleep_us(2);
    }
    adc_select_input(ADC_CHANNEL);
}

// ============================================================================
// FFT
// ============================================================================

static void fft_init_tables(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        float angle = -2.0f * M_PI * i / FFT_SIZE;
        cos_table[i] = cosf(angle);
        sin_table[i] = sinf(angle);
    }
}

static int bit_reverse(int n, int bits) {
    int reversed = 0;
    for (int i = 0; i < bits; i++) {
        reversed = (reversed << 1) | (n & 1);
        n >>= 1;
    }
    return reversed;
}

static void fft(float *real, float *imag) {
    int n = FFT_SIZE;
    int bits = (int)log2f(n);

    for (int i = 0; i < n; i++) {
        int j = bit_reverse(i, bits);
        if (j > i) {
            float temp = real[i]; real[i] = real[j]; real[j] = temp;
            temp = imag[i]; imag[i] = imag[j]; imag[j] = temp;
        }
    }

    for (int step = 2; step <= n; step *= 2) {
        int half_step = step / 2;
        for (int start = 0; start < n; start += step) {
            for (int k = 0; k < half_step; k++) {
                int idx = (k * n) / step;
                float cos_val = cos_table[idx];
                float sin_val = sin_table[idx];
                int even_idx = start + k;
                int odd_idx = start + k + half_step;
                float even_r = real[even_idx];
                float even_i = imag[even_idx];
                float odd_r = real[odd_idx] * cos_val - imag[odd_idx] * sin_val;
                float odd_i = real[odd_idx] * sin_val + imag[odd_idx] * cos_val;
                real[even_idx] = even_r + odd_r;
                imag[even_idx] = even_i + odd_i;
                real[odd_idx] = even_r - odd_r;
                imag[odd_idx] = even_i - odd_i;
            }
        }
    }
}

static void compute_fft(float *samples) {
    float real[FFT_SIZE];
    float imag[FFT_SIZE];

    float mean = 0;
    for (int i = 0; i < FFT_SIZE; i++) mean += samples[i];
    mean /= FFT_SIZE;

    for (int i = 0; i < FFT_SIZE; i++) {
        float window = 0.5f - 0.5f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
        real[i] = (samples[i] - mean) * window;
        imag[i] = 0.0f;
    }

    fft(real, imag);

    float max_mag = 0;
    int max_idx = 1;

    for (int i = 0; i < FFT_HALF_SIZE; i++) {
        fft_result.mags[i] = sqrtf(real[i] * real[i] + imag[i] * imag[i]);
        fft_result.freqs[i] = (float)i * SAMPLE_RATE / FFT_SIZE;
        if (i > 0 && fft_result.mags[i] > max_mag) {
            max_mag = fft_result.mags[i];
            max_idx = i;
        }
    }

    fft_result.dominant_freq = fft_result.freqs[max_idx];
    fft_result.dominant_mag = max_mag;

    if (max_mag > 0) {
        for (int i = 0; i < FFT_HALF_SIZE; i++) {
            fft_result.mags[i] /= max_mag;
        }
    }
}

// ============================================================================
// MEDICIONES
// ============================================================================

static void calculate_measurements(float *vin_samples, int count) {
    float min_val = vin_samples[0], max_val = vin_samples[0];
    for (int i = 1; i < count; i++) {
        if (vin_samples[i] < min_val) min_val = vin_samples[i];
        if (vin_samples[i] > max_val) max_val = vin_samples[i];
    }
    current_measurements.vpp = max_val - min_val;
    current_measurements.max_voltage = max_val;
    current_measurements.min_voltage = min_val;

    float sum = 0;
    for (int i = 0; i < count; i++) sum += vin_samples[i];
    current_measurements.vdc = sum / count;

    float sum_sq = 0;
    for (int i = 0; i < count; i++) sum_sq += vin_samples[i] * vin_samples[i];
    current_measurements.vrms = sqrtf(sum_sq / count);

    float sum_sq_ac = 0;
    for (int i = 0; i < count; i++) {
        float diff = vin_samples[i] - current_measurements.vdc;
        sum_sq_ac += diff * diff;
    }
    current_measurements.vac = sqrtf(sum_sq_ac / count);

    int crossings = 0;
    bool last_sign = vin_samples[0] >= current_measurements.vdc;
    for (int i = 1; i < count; i++) {
        bool current_sign = vin_samples[i] >= current_measurements.vdc;
        if (current_sign != last_sign) {
            crossings++;
            last_sign = current_sign;
        }
    }

    if (crossings > 2) {
        current_measurements.frequency = (crossings / 2.0f) * SAMPLE_RATE / count;
        current_measurements.period = 1.0f / current_measurements.frequency;
    } else {
        current_measurements.frequency = 0;
        current_measurements.period = 0;
    }

    int above_dc = 0;
    for (int i = 0; i < count; i++) {
        if (vin_samples[i] >= current_measurements.vdc) above_dc++;
    }
    current_measurements.duty_cycle = (100.0f * above_dc) / count;

    float v10 = min_val + 0.1f * current_measurements.vpp;
    float v90 = min_val + 0.9f * current_measurements.vpp;
    int t10 = -1, t90 = -1;
    for (int i = 0; i < count - 1; i++) {
        if (t10 < 0 && vin_samples[i] <= v10 && vin_samples[i+1] >= v10) t10 = i;
        if (t90 < 0 && vin_samples[i] <= v90 && vin_samples[i+1] >= v90) t90 = i;
    }
    if (t10 >= 0 && t90 >= 0 && t90 > t10) {
        current_measurements.rise_time = (t90 - t10) / (float)SAMPLE_RATE;
    } else {
        current_measurements.rise_time = 0;
    }

    t90 = -1; t10 = -1;
    for (int i = 0; i < count - 1; i++) {
        if (t90 < 0 && vin_samples[i] >= v90 && vin_samples[i+1] <= v90) t90 = i;
        if (t10 < 0 && vin_samples[i] >= v10 && vin_samples[i+1] <= v10) t10 = i;
    }
    if (t90 >= 0 && t10 >= 0 && t10 > t90) {
        current_measurements.fall_time = (t10 - t90) / (float)SAMPLE_RATE;
    } else {
        current_measurements.fall_time = 0;
    }
}

// ============================================================================
// TRIGGER
// ============================================================================

static int find_trigger(float *samples, int count) {
    if (trigger_mode == TRIGGER_MODE_SINGLE && single_triggered) {
        return -1;
    }

    float trigger_vin = (trigger_level - OFFSET_VOLTAGE) * DIVISOR_RATIO;

    if (trigger_edge == TRIGGER_RISING) {
        for (int i = 1; i < count; i++) {
            if (samples[i-1] < trigger_vin && samples[i] >= trigger_vin) {
                if (trigger_mode == TRIGGER_MODE_SINGLE) single_triggered = true;
                return i;
            }
        }
    } else if (trigger_edge == TRIGGER_FALLING) {
        for (int i = 1; i < count; i++) {
            if (samples[i-1] > trigger_vin && samples[i] <= trigger_vin) {
                if (trigger_mode == TRIGGER_MODE_SINGLE) single_triggered = true;
                return i;
            }
        }
    } else {
        for (int i = 1; i < count; i++) {
            bool rising = (samples[i-1] < trigger_vin && samples[i] >= trigger_vin);
            bool falling = (samples[i-1] > trigger_vin && samples[i] <= trigger_vin);
            if (rising || falling) {
                if (trigger_mode == TRIGGER_MODE_SINGLE) single_triggered = true;
                return i;
            }
        }
    }

    return (trigger_mode == TRIGGER_MODE_AUTO) ? 0 : -1;
}

// ============================================================================
// DIBUJO EN PANTALLA
// ============================================================================

static void draw_grid(void) {
    for (int y = 0; y < OLED_HEIGHT; y += 16) {
        for (int x = 0; x < OLED_WIDTH; x += 2) {
            oled_pixel(x, y, true);
        }
    }
    for (int x = 0; x < OLED_WIDTH; x += 16) {
        for (int y = 0; y < OLED_HEIGHT; y += 2) {
            oled_pixel(x, y, true);
        }
    }
}

static void draw_waveform(float *vin_samples) {
    int trigger_idx = find_trigger(vin_samples, NUM_SAMPLES);
    if (trigger_idx < 0) trigger_idx = 0;

    float v_scale = v_scales[v_scale_idx];
    float v_max_display = v_scale * 4.0f;
    float v_center = (coupling_mode == COUPLING_AC) ? 0.0f : current_measurements.vdc;

    int prev_x = 0, prev_y = OLED_HEIGHT / 2;

    for (int i = 0; i < NUM_SAMPLES && i < OLED_WIDTH; i++) {
        int idx = (trigger_idx + i) % NUM_SAMPLES;
        float voltage = vin_samples[idx];
        float relative_voltage = voltage - v_center;

        int x = (i * OLED_WIDTH) / NUM_SAMPLES;
        int y = OLED_HEIGHT / 2 - (int)((relative_voltage / v_max_display) * (OLED_HEIGHT / 2));

        if (y < 0) y = 0;
        if (y >= OLED_HEIGHT) y = OLED_HEIGHT - 1;

        if (i == 0) {
            prev_x = x;
            prev_y = y;
        }

        oled_line(prev_x, prev_y, x, y, true);
        prev_x = x;
        prev_y = y;
    }

    int ref_y = OLED_HEIGHT / 2;
    for (int x = 0; x < OLED_WIDTH; x += 4) {
        oled_pixel(x, ref_y, true);
    }
}

static void draw_xy_mode(float *ch1, float *ch2) {
    float v_scale = v_scales[v_scale_idx];
    float v_max_display = v_scale * 4.0f;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        int x = OLED_WIDTH / 2 + (int)((ch1[i] / v_max_display) * (OLED_WIDTH / 2));
        int y = OLED_HEIGHT / 2 - (int)((ch2[i] / v_max_display) * (OLED_HEIGHT / 2));

        if (x >= 0 && x < OLED_WIDTH && y >= 0 && y < OLED_HEIGHT) {
            oled_pixel(x, y, true);
        }
    }
}

static void draw_fft(void) {
    int num_bars = OLED_WIDTH / 2;
    int bar_width = 2;

    for (int i = 0; i < num_bars && i < FFT_HALF_SIZE; i++) {
        int h = (int)(fft_result.mags[i] * (OLED_HEIGHT - 10));
        if (h < 1) h = 1;
        if (h > OLED_HEIGHT - 10) h = OLED_HEIGHT - 10;

        int x = i * bar_width;
        int y = OLED_HEIGHT - h;

        for (int bx = 0; bx < bar_width - 1; bx++) {
            oled_vline(x + bx, y, h, true);
        }
    }
}

static void draw_info(void) {
    char buf[32];

    oled_hline(0, 10, OLED_WIDTH, true);

    snprintf(buf, sizeof(buf), "%.0fV", v_scales[v_scale_idx]);
    oled_text(0, 0, buf, true);

    snprintf(buf, sizeof(buf), "%.1fms", t_scales[t_scale_idx]);
    oled_text(35, 0, buf, true);

    oled_text(70, 0, (coupling_mode == COUPLING_DC) ? "DC" : "AC", true);

    if (!running) {
        oled_text(100, 0, "||", true);
    }

    snprintf(buf, sizeof(buf), "F:%.0f", current_measurements.frequency);
    oled_text(0, 54, buf, true);

    snprintf(buf, sizeof(buf), "P:%.1f", current_measurements.vpp);
    oled_text(50, 54, buf, true);
}

static void draw_measurements_screen(void) {
    char buf[32];
    int y = 0;

    oled_text(0, y, "MEDICIONES", true); y += 10;
    oled_hline(0, y, OLED_WIDTH, true); y += 3;

    snprintf(buf, sizeof(buf), "Vpp:%.2fV", current_measurements.vpp);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "Vrms:%.2fV", current_measurements.vrms);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "VDC:%.2fV", current_measurements.vdc);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "VAC:%.2fV", current_measurements.vac);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "F:%.1fHz", current_measurements.frequency);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "T:%.4fs", current_measurements.period);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "D:%.1f%%", current_measurements.duty_cycle);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "R:%.6fs", current_measurements.rise_time);
    oled_text(0, y, buf, true); y += 9;

    snprintf(buf, sizeof(buf), "F:%.6fs", current_measurements.fall_time);
    oled_text(0, y, buf, true);
}

static void update_display(float *vin_samples, float *vin_ch2) {
    oled_clear();

    switch (current_mode) {
        case MODE_NORMAL:
            draw_grid();
            draw_waveform(vin_samples);
            draw_info();
            break;
        case MODE_XY:
            draw_xy_mode(vin_samples, vin_ch2);
            draw_info();
            break;
        case MODE_FFT:
            draw_fft();
            draw_info();
            break;
        case MODE_MEASUREMENTS:
            draw_measurements_screen();
            break;
    }

    oled_show();
}

// ============================================================================
// COMUNICACION USB - OPTIMIZADO PARA USB OTG
// ============================================================================

static void usb_send_json(const char *json) {
    printf("%s\n", json);
}

static void usb_send_waveform(float *samples, int count) {
    // Formato compacto para telefono - solo 64 muestras para rapidez
    strcpy(usb_tx_buffer, "{"type":"wf","s":[");
    int len = strlen(usb_tx_buffer);
    for (int i = 0; i < count && i < 64; i++) {
        len += snprintf(usb_tx_buffer + len, USB_BUFFER_SIZE - len, "%.2f", samples[i]);
        if (i < 63) {
            usb_tx_buffer[len++] = ',';
            usb_tx_buffer[len] = '\0';
        }
    }
    strcat(usb_tx_buffer, "]}");
    usb_send_json(usb_tx_buffer);
}

static void usb_send_measurements_compact(void) {
    // Formato ultra-compacto para telefono
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
        "{"t":"m","vpp":%.2f,"vrms":%.2f,"vdc":%.2f,"vac":%.2f,"f":%.1f,"p":%.4f,"d":%.1f,"r":%.6f,"fa":%.6f,"mx":%.2f,"mn":%.2f}",
        current_measurements.vpp, current_measurements.vrms,
        current_measurements.vdc, current_measurements.vac,
        current_measurements.frequency, current_measurements.period,
        current_measurements.duty_cycle,
        current_measurements.rise_time, current_measurements.fall_time,
        current_measurements.max_voltage, current_measurements.min_voltage);
    usb_send_json(usb_tx_buffer);
}

static void usb_send_fft_compact(void) {
    strcpy(usb_tx_buffer, "{"t":"ft","f":[");
    int len = strlen(usb_tx_buffer);
    for (int i = 0; i < FFT_HALF_SIZE && i < 32; i++) {
        len += snprintf(usb_tx_buffer + len, USB_BUFFER_SIZE - len, "%.1f", fft_result.freqs[i]);
        if (i < 31) {
            usb_tx_buffer[len++] = ',';
            usb_tx_buffer[len] = '\0';
        }
    }
    strcat(usb_tx_buffer, "],"m":[");
    len = strlen(usb_tx_buffer);
    for (int i = 0; i < FFT_HALF_SIZE && i < 32; i++) {
        len += snprintf(usb_tx_buffer + len, USB_BUFFER_SIZE - len, "%.3f", fft_result.mags[i]);
        if (i < 31) {
            usb_tx_buffer[len++] = ',';
            usb_tx_buffer[len] = '\0';
        }
    }
    strcat(usb_tx_buffer, "]}");
    usb_send_json(usb_tx_buffer);
}

static void usb_send_status(void) {
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
        "{"t":"st","mode":%d,"vscale":%.0f,"tscale":%.1f,"coupling":%d,"run":%d,"trig":%d}",
        current_mode, v_scales[v_scale_idx], t_scales[t_scale_idx],
        coupling_mode, running ? 1 : 0, trigger_edge);
    usb_send_json(usb_tx_buffer);
}

static void process_usb_command(const char *cmd) {
    if (strstr(cmd, "get_wf")) {
        float samples[NUM_SAMPLES];
        convert_buffer_to_voltage(adc_buffer, samples, NUM_SAMPLES);
        usb_send_waveform(samples, NUM_SAMPLES);
    } else if (strstr(cmd, "get_ft")) {
        usb_send_fft_compact();
    } else if (strstr(cmd, "get_m")) {
        usb_send_measurements_compact();
    } else if (strstr(cmd, "get_st")) {
        usb_send_status();
    } else if (strstr(cmd, "set_mode")) {
        int mode = atoi(strchr(cmd, ':') + 1);
        if (mode >= 0 && mode <= 3) current_mode = mode;
    } else if (strstr(cmd, "set_vscale")) {
        int idx = atoi(strchr(cmd, ':') + 1);
        if (idx >= 0 && idx < NUM_V_SCALES) v_scale_idx = idx;
    } else if (strstr(cmd, "set_tscale")) {
        int idx = atoi(strchr(cmd, ':') + 1);
        if (idx >= 0 && idx < NUM_T_SCALES) t_scale_idx = idx;
    } else if (strstr(cmd, "set_coupling")) {
        int c = atoi(strchr(cmd, ':') + 1);
        if (c == 0 || c == 1) coupling_mode = c;
    } else if (strstr(cmd, "set_run")) {
        int r = atoi(strchr(cmd, ':') + 1);
        running = (r == 1);
    } else if (strstr(cmd, "set_trig")) {
        int t = atoi(strchr(cmd, ':') + 1);
        if (t >= 0 && t <= 2) trigger_edge = t;
    } else if (strstr(cmd, "help")) {
        printf("\n=== COMANDOS USB OTG ===\n");
        printf("get_wf      - Obtener forma de onda\n");
        printf("get_ft      - Obtener FFT\n");
        printf("get_m       - Obtener mediciones\n");
        printf("get_st      - Obtener estado\n");
        printf("set_mode:N  - Modo 0=Normal 1=XY 2=FFT 3=Med\n");
        printf("set_vscale:N- Escala V 0-7\n");
        printf("set_tscale:N- Escala T 0-5\n");
        printf("set_coupling:N - 0=DC 1=AC\n");
        printf("set_run:N   - 1=Run 0=Pause\n");
        printf("set_trig:N  - 0=Rise 1=Fall 2=Both\n");
        printf("========================\n\n");
    }
}

static void check_usb_input(void) {
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT && c != EOF) {
        if (c == '\n' || c == '\r') {
            usb_rx_buffer[usb_rx_idx] = '\0';
            if (usb_rx_idx > 0) {
                process_usb_command(usb_rx_buffer);
            }
            usb_rx_idx = 0;
        } else if (usb_rx_idx < USB_BUFFER_SIZE - 1) {
            usb_rx_buffer[usb_rx_idx++] = (char)c;
        }
    }
}

// ============================================================================
// WIFI (MANTENIDO PERO OPCIONAL)
// ============================================================================

static void wifi_init(void) {
    uart_init(WIFI_UART, WIFI_BAUDRATE);
    gpio_set_function(WIFI_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(WIFI_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(WIFI_UART, 8, 1, UART_PARITY_NONE);
}

static void wifi_send_cmd(const char *cmd) {
    uart_puts(WIFI_UART, cmd);
    uart_puts(WIFI_UART, "\r\n");
}

static bool wifi_wait_for(const char *response, uint32_t timeout_ms) {
    char buffer[256];
    uint32_t start = time_us_32() / 1000;
    int idx = 0;
    while ((time_us_32() / 1000) - start < timeout_ms) {
        while (uart_is_readable(WIFI_UART) && idx < 255) {
            buffer[idx++] = uart_getc(WIFI_UART);
            buffer[idx] = '\0';
            if (strstr(buffer, response)) return true;
        }
        sleep_ms(10);
    }
    return false;
}

static bool wifi_connect(const char *ssid, const char *password) {
    wifi_send_cmd("AT+RST");
    sleep_ms(2000);
    wifi_send_cmd("AT+CWMODE=1");
    sleep_ms(100);
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    wifi_send_cmd(cmd);
    return wifi_wait_for("OK", 10000);
}

// ============================================================================
// FLASH STORAGE
// ============================================================================

static uint8_t flash_buffer[FLASH_PAGE_SIZE];

static void flash_save_waveform(float *samples, int count) {
    uint32_t timestamp = time_us_32() / 1000000;
    memset(flash_buffer, 0, sizeof(flash_buffer));
    memcpy(flash_buffer, &timestamp, 4);
    memcpy(flash_buffer + 4, &count, 2);
    memcpy(flash_buffer + 6, samples, count * sizeof(float));

    static int current_slot = 0;
    uint32_t offset = FLASH_TARGET_OFFSET + (current_slot * FLASH_PAGE_SIZE);
    current_slot = (current_slot + 1) % MAX_WAVEFORMS;

    if (current_slot == 0) {
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    }
    flash_range_program(offset, flash_buffer, FLASH_PAGE_SIZE);
}

// ============================================================================
// BOTONES - OPTIMIZADO (sin float en ISR)
// ============================================================================

static void button_callback(uint gpio, uint32_t events) {
    uint32_t now = time_us_32() / 1000;
    if (now - last_button_time < DEBOUNCE_MS) return;
    last_button_time = now;

    switch (gpio) {
        case BTN_VSCALE_PIN:
            v_scale_idx = (v_scale_idx + 1) % NUM_V_SCALES;
            break;
        case BTN_TSCALE_PIN:
            t_scale_idx = (t_scale_idx + 1) % NUM_T_SCALES;
            break;
        case BTN_PAUSE_PIN:
            running = !running;
            break;
        case BTN_MODE_PIN:
            current_mode = (current_mode + 1) % 4;
            break;
        case BTN_TRIGGER_PIN:
            trigger_edge = (trigger_edge + 1) % 3;
            break;
        case BTN_SAVE_PIN: {
            float samples[NUM_SAMPLES];
            convert_buffer_to_voltage(adc_buffer, samples, NUM_SAMPLES);
            flash_save_waveform(samples, NUM_SAMPLES);
            break;
        }
        case BTN_OFFSET_PIN:
            coupling_mode = (coupling_mode == COUPLING_DC) ? COUPLING_AC : COUPLING_DC;
            break;
    }
}

static void setup_buttons(void) {
    const uint pins[] = {BTN_VSCALE_PIN, BTN_TSCALE_PIN, BTN_PAUSE_PIN, 
                         BTN_MODE_PIN, BTN_TRIGGER_PIN, BTN_SAVE_PIN, BTN_OFFSET_PIN};
    for (int i = 0; i < 7; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    gpio_set_irq_enabled_with_callback(BTN_VSCALE_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    for (int i = 1; i < 7; i++) {
        gpio_set_irq_enabled(pins[i], GPIO_IRQ_EDGE_FALL, true);
    }
}

// ============================================================================
// CORE 1 - PROCESAMIENTO CON SPINLOCK SEGURO
// ============================================================================

static float voltage_buffer[NUM_SAMPLES];
static float voltage_buffer_ch2[NUM_SAMPLES];

void core1_entry(void) {
    while (true) {
        uint32_t irq_state = spin_lock_blocking(adc_spin_lock);
        bool done = adc_dma_done;
        spin_unlock(adc_spin_lock, irq_state);

        if (done) {
            convert_buffer_to_voltage(adc_buffer, voltage_buffer, NUM_SAMPLES);
            calculate_measurements(voltage_buffer, NUM_SAMPLES);
            if (current_mode == MODE_FFT) {
                compute_fft(voltage_buffer);
            }
            if (current_mode == MODE_XY) {
                adc_sample_ch2();
                convert_buffer_to_voltage(adc_buffer_ch2, voltage_buffer_ch2, NUM_SAMPLES);
            }

            irq_state = spin_lock_blocking(adc_spin_lock);
            adc_dma_done = false;
            spin_unlock(adc_spin_lock, irq_state);
        }
        sleep_us(100);
    }
}

// ============================================================================
// GENERADOR DE PRUEBA 1kHz (para GPIO 22)
// ============================================================================

#define TEST_SIGNAL_PIN     22
#define TEST_FREQ_HZ        1000

static void init_test_signal(void) {
    gpio_init(TEST_SIGNAL_PIN);
    gpio_set_dir(TEST_SIGNAL_PIN, GPIO_OUT);
    gpio_put(TEST_SIGNAL_PIN, 0);
}

static void update_test_signal(void) {
    static uint32_t last_toggle = 0;
    static bool state = false;

    uint32_t now = time_us_32();
    uint32_t half_period = 500000 / TEST_FREQ_HZ; // 500us para 1kHz

    if (now - last_toggle >= half_period) {
        last_toggle = now;
        state = !state;
        gpio_put(TEST_SIGNAL_PIN, state);
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    stdio_init_all();

    // Inicializar spinlock para sincronizacion segura
    adc_spin_lock_num = spin_lock_claim_unused(true);
    adc_spin_lock = spin_lock_init(adc_spin_lock_num);

    // Inicializar I2C para OLED
    i2c_init(OLED_I2C, OLED_I2C_FREQ);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SCL_PIN);
    gpio_pull_up(OLED_SDA_PIN);

    // Inicializar OLED
    oled_init();
    oled_clear();
    oled_text(10, 20, "OSCILOSCOPIO", true);
    oled_text(15, 32, "PICO v2.2", true);
    oled_text(5, 44, "USB OTG OK", true);
    oled_show();
    sleep_ms(2000);

    // Inicializar subsistemas
    fft_init_tables();
    adc_dma_init();
    wifi_init();
    setup_buttons();
    init_test_signal();

    // Pantalla de inicio
    oled_clear();
    oled_text(0, 0, "INICIANDO...", true);
    oled_show();
    sleep_ms(500);

    // Lanzar Core 1 para procesamiento
    multicore_launch_core1(core1_entry);

    // Variables de tiempo para USB
    uint32_t last_usb_update = 0;
    uint32_t last_status_update = 0;

    // Mensaje inicial por USB
    printf("\n");
    printf("========================================\n");
    printf("  OSCILOSCOPIO PICO v2.2 - USB OTG\n");
    printf("========================================\n");
    printf("Comandos disponibles:\n");
    printf("  get_wf     - Forma de onda\n");
    printf("  get_m      - Mediciones\n");
    printf("  get_ft     - FFT\n");
    printf("  get_st     - Estado\n");
    printf("  help       - Ayuda completa\n");
    printf("========================================\n\n");

    while (true) {
        // Actualizar señal de prueba 1kHz en GPIO 22
        update_test_signal();

        if (running) {
            adc_dma_start();
            adc_dma_wait();
            update_display(voltage_buffer, voltage_buffer_ch2);
        }

        // Procesar comandos USB entrantes
        check_usb_input();

        // Enviar datos automaticamente cada 100ms para telefono
        uint32_t now = time_us_32() / 1000;
        if (now - last_usb_update > USB_UPDATE_MS) {
            usb_send_measurements_compact();
            last_usb_update = now;
        }

        // Enviar estado cada 500ms
        if (now - last_status_update > 500) {
            usb_send_status();
            last_status_update = now;
        }

        sleep_ms(5);
    }

    return 0;
}
