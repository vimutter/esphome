#include "adc_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP8266
#ifdef USE_ADC_SENSOR_VCC
#include <Esp.h>
ADC_MODE(ADC_VCC)
#else
#include <Arduino.h>
#endif
#endif

namespace esphome {
namespace adc {

static const char *const TAG = "adc";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_CHAN0          ADC_CHANNEL_4
#define ADC1_CHAN1          ADC_CHANNEL_5
#else
#define ADC1_CHAN0          ADC_CHANNEL_0    // GPIO0
#define ADC1_CHAN1          ADC_CHANNEL_1    // GPIO1
#define ADC1_CHAN2          ADC_CHANNEL_2    // GPIO2
#define ADC1_CHAN3          ADC_CHANNEL_3    // GPIO3
#define ADC1_CHAN4          ADC_CHANNEL_4    // GPIO4
#define ADC1_CHAN5          ADC_CHANNEL_5    // GPIO5
#define ADC1_CHAN6          ADC_CHANNEL_6    // GPIO6
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define USE_ADC2            1
#endif

#if USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC2_CHAN0          ADC_CHANNEL_0
#else
#define ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if EXAMPLE_USE_ADC

#define ADC_ATTEN           ADC_ATTEN_DB_12   // The input voltage of ADC will be attenuated extending the range of measurement by about 12 dB.

static int adc_raw[2][10];
static int voltage[2][10];
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

void ADCSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADC '%s'...", this->get_name().c_str());
  ESP_LOGCONFIG(TAG, "ADC '%s' setup finished!", this->get_name().c_str());
}

void ADCSensor::dump_config() {
  LOG_SENSOR("", "ADC Sensor", this);
#if defined(USE_ESP8266) || defined(USE_LIBRETINY)
#ifdef USE_ADC_SENSOR_VCC
  ESP_LOGCONFIG(TAG, "  Pin: VCC");
#else
  LOG_PIN("  Pin: ", this->pin_);
#endif
#endif  // USE_ESP8266 || USE_LIBRETINY

#ifdef USE_ESP32
  LOG_PIN("  Pin: ", this->pin_);
  if (this->autorange_) {
    ESP_LOGCONFIG(TAG, "  Attenuation: auto");
  } /*else {
    switch (this->attenuation_) {
      case ADC_ATTEN_DB_0:
        ESP_LOGCONFIG(TAG, "  Attenuation: 0db");
        break;
      case ADC_ATTEN_DB_2_5:
        ESP_LOGCONFIG(TAG, "  Attenuation: 2.5db");
        break;
      case ADC_ATTEN_DB_6:
        ESP_LOGCONFIG(TAG, "  Attenuation: 6db");
        break;
      case ADC_ATTEN_DB_12_COMPAT:
        ESP_LOGCONFIG(TAG, "  Attenuation: 12db");
        break;
      default:  // This is to satisfy the unused ADC_ATTEN_MAX
        break;
    }
  }
  */
#endif  // USE_ESP32

#ifdef USE_RP2040
  if (this->is_temperature_) {
    ESP_LOGCONFIG(TAG, "  Pin: Temperature");
  } else {
#ifdef USE_ADC_SENSOR_VCC
    ESP_LOGCONFIG(TAG, "  Pin: VCC");
#else
    LOG_PIN("  Pin: ", this->pin_);
#endif  // USE_ADC_SENSOR_VCC
  }
#endif  // USE_RP2040
  ESP_LOGCONFIG(TAG, "  Samples: %i", this->sample_count_);
  LOG_UPDATE_INTERVAL(this);
}

float ADCSensor::get_setup_priority() const { return setup_priority::DATA; }
void ADCSensor::update() {
  float value_v = this->sample();
  ESP_LOGV(TAG, "'%s': Got voltage=%.4fV", this->get_name().c_str(), value_v);
  this->publish_state(value_v);
}

void ADCSensor::set_sample_count(uint8_t sample_count) {
  if (sample_count != 0) {
    this->sample_count_ = sample_count;
  }
}

#ifdef USE_ESP32
float ADCSensor::sample()
{
  
  // https://github.com/espressif/esp-idf/blob/release/v5.2/components/hal/include/hal/adc_types.h
  //-------------ADC1 Init---------------//
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,                    // Selects the ADC
      //.clk_src = 0,                           // Selects the source clock of the ADC. If set to 0, the driver will fall back to using a default clock source
      .ulp_mode = ADC_ULP_MODE_DISABLE,         // Sets if the ADC will be working under ULP mode.
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  
  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
  //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config));
  
  //-------------ADC1 Calibration Init---------------//
  adc_cali_handle_t adc1_cali_chan0_handle = NULL;
  adc_cali_handle_t adc1_cali_chan1_handle = NULL;
  bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);
  //bool do_calibration1_chan1 = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN1, ADC_ATTEN, &adc1_cali_chan1_handle);
  
  adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[0][0]);
  ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN0, adc_raw[0][0]);
  if (do_calibration1_chan0) {
      // Convert the ADC raw result into calibrated result
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
      ESP_LOGI(TAG, "ADC%d Channel[%d] Calibrated Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN0, voltage[0][0]);
  }
  
  /*
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][1]));
  ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1, adc_raw[0][1]);
  if (do_calibration1_chan1) {
      // Convert the ADC raw result into calibrated result
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
      ESP_LOGI(TAG, "ADC%d Channel[%d] Calibrated Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN1, voltage[0][1]);
  }
  */
  //Tear Down
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
  if (do_calibration1_chan0) {
      adc_calibration_deinit(adc1_cali_chan0_handle);
  }
  /*
  if (do_calibration1_chan1) {
      adc_calibration_deinit(adc1_cali_chan1_handle);
  }
  */
    uint32_t mv_scaled = voltage[0][0]; // ADC1_CHAN0
    //uint32_t mv_scaled = voltage[0][1]; // ADC1_CHAN1
    float result = (float)(mv_scaled) / 1000;
    return result;
  }

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
#endif  // USE_ESP32

}  // namespace adc
}  // namespace esphome
