#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "dsps_fft2r.h"    // ESP-DSP FFT库
#include "dsps_wind_hann.h" // 汉宁窗函数
#include "dsps_view.h"      // 用于数据可视化
#include <math.h>  // 添加这一行
// #include "freertos/task.h"
#define TAG "adc_continuous"
#define N_SAMPLES 1024      // FFT点数，必须是2的幂
#define SAMPLE_COUNT 1024   // 采样点数
#define LED_PIN 7
float *fft_input;    // 输入数据缓冲
float *fft_window;   // 窗函数
float *fft_output;   // FFT输出
__attribute__((aligned(16))) float fft_buffer[N_SAMPLES*2]; // FFT工作缓冲区
bool on_conv_done(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    QueueHandle_t queue = (QueueHandle_t)user_data;
    bool higher_priority_task_woken = false;
    // printf("conv done\n");
    xQueueSendFromISR(queue, edata, &higher_priority_task_woken);
    return higher_priority_task_woken;
}

bool on_pool_ovf(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    // printf("pool ovf\n");
    return false;
}
void on_conv_done_task(void *pvParameters)
{
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    adc_continuous_evt_data_t edata;
    
    // 初始化FFT缓冲区
    fft_input = heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_window = heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT);
    fft_output = heap_caps_malloc(N_SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT);
    
    // 初始化汉宁窗
    dsps_wind_hann_f32(fft_window, N_SAMPLES);
    
    // 初始化FFT
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));
    
    int sample_index = 0;
    
    while (true)
    {
        if(xQueueReceive(queue, &edata, portMAX_DELAY))
        {
            adc_digi_output_data_t *data = (adc_digi_output_data_t *)edata.conv_frame_buffer;
            
            for (int i = 0; i < edata.size/4; i++)
            {
                // 收集采样数据
                if (sample_index < N_SAMPLES) {
                    fft_input[sample_index] = (float)data[i].type2.data;
                    ESP_LOGI(TAG, "%.0f,", fft_input[sample_index]);
                    sample_index++;
                }
                
                // 当收集够足够的样本时，执行FFT
                if (sample_index >= N_SAMPLES) {
                    // 应用窗函数
                    for (int k = 0; k < N_SAMPLES; k++) {
                        fft_buffer[k*2] = fft_input[k] * fft_window[k];
                        fft_buffer[k*2+1] = 0; // 虚部设为0
                    }
                    
                    // 执行FFT
                    dsps_fft2r_fc32(fft_buffer, N_SAMPLES);
                    
                    // 计算幅值谱
                    for (int k = 0; k < N_SAMPLES/2; k++) {
                        float real = fft_buffer[k*2];
                        float imag = fft_buffer[k*2+1];
                        fft_output[k] = sqrtf(real*real + imag*imag);
                        
                        // 打印FFT结果
                        // printf("FFT bin %d: %f\n", k, fft_output[k]);
                    }
                    
                    // 清空缓冲区
                    sample_index = 0;
                }
            }
        }
        vTaskDelay(10);
    }
}

void app_main(void)
{
    gpio_config_t gpio_cfg;
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_cfg.pin_bit_mask = (1ULL << LED_PIN);
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_cfg);

    // gpio_config_t gpio_cfg2;
    // gpio_cfg2.mode = GPIO_MODE_INPUT;
    // gpio_cfg2.intr_type = GPIO_INTR_DISABLE;
    // gpio_cfg2.pin_bit_mask = (1ULL << 6);
    // // gpio_cfg2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    // // gpio_cfg2.pull_up_en = GPIO_PULLUP_ENABLE;
    // gpio_config(&gpio_cfg2);

    adc_continuous_handle_cfg_t adc_continuous_handle_cfg;
    adc_continuous_handle_cfg.conv_frame_size = 4096;
    adc_continuous_handle_cfg.flags.flush_pool = true;
    adc_continuous_handle_cfg.max_store_buf_size = 8192;
    adc_continuous_handle_t adc_continuous_handle;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_continuous_handle_cfg, &adc_continuous_handle));
    ESP_LOGI(TAG, "adc_continuous_handle: %p", adc_continuous_handle);

    adc_digi_pattern_config_t adc_digi_pattern_cfg[SOC_ADC_PATT_LEN_MAX]={0};
    adc_digi_pattern_cfg[0].atten = ADC_ATTEN_DB_0;
    adc_digi_pattern_cfg[0].bit_width = 12;
    // adc_digi_pattern_cfg[0].unit=ADC_UNIT_1;
    // adc_digi_pattern_cfg[0].channel=5;
    ESP_LOGI(TAG,"sizeof unit sizeof channel: %d, %d", sizeof(adc_digi_pattern_cfg[0].unit), sizeof(adc_digi_pattern_cfg[0].channel));
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(9, &(adc_digi_pattern_cfg[0].unit), &(adc_digi_pattern_cfg[0].channel)));
    ESP_LOGI(TAG, "adc_digi_pattern_cfg[0].unit: %d, adc_digi_pattern_cfg[0].channel: %d", adc_digi_pattern_cfg[0].unit, adc_digi_pattern_cfg[0].channel);

    adc_continuous_config_t adc_continuous_cfg;
    adc_continuous_cfg.pattern_num=1;
    adc_continuous_cfg.adc_pattern = adc_digi_pattern_cfg;
    adc_continuous_cfg.sample_freq_hz = 1000;
    adc_continuous_cfg.conv_mode=ADC_CONV_SINGLE_UNIT_1;
    adc_continuous_cfg.format=ADC_DIGI_OUTPUT_FORMAT_TYPE2; 
    if(adc_digi_pattern_cfg[0].bit_width < SOC_ADC_DIGI_MIN_BITWIDTH || adc_digi_pattern_cfg[0].bit_width > SOC_ADC_DIGI_MAX_BITWIDTH)
    {
        ESP_LOGE(TAG, "ADC bitwidth not supported %d", adc_digi_pattern_cfg[0].bit_width);
        // return;
    }
    // ESP_ERROR_CHECK((adc_digi_pattern_cfg[0].bit_width >= SOC_ADC_DIGI_MIN_BITWIDTH && adc_digi_pattern_cfg[0].bit_width <= SOC_ADC_DIGI_MAX_BITWIDTH), ESP_ERR_INVALID_ARG, TAG, "ADC bitwidth not supported");


    ESP_ERROR_CHECK(adc_continuous_config(adc_continuous_handle, &adc_continuous_cfg));
    ESP_LOGI(TAG, "adc_continuous_cfg.sample_freq_hz: %ld", adc_continuous_cfg.sample_freq_hz);
    adc_continuous_evt_cbs_t adc_continuous_evt_cbs;
    adc_continuous_evt_cbs.on_conv_done= on_conv_done;  
    adc_continuous_evt_cbs.on_pool_ovf=on_pool_ovf;
    QueueHandle_t queue = xQueueCreate(10, sizeof(adc_continuous_evt_data_t));

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_continuous_handle, &adc_continuous_evt_cbs, (void *)queue));
    // adc_continuous_start(adc_continuous_handle);
    ESP_ERROR_CHECK(adc_continuous_start(adc_continuous_handle));
    ESP_LOGI(TAG, "adc_continuous_start");
    xTaskCreate(on_conv_done_task, "on_conv_done_task", 4096, (void *)queue, 5, NULL);
    while (true)
    {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(100);
    }

}