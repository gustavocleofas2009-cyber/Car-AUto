#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"

// Cabeçalhos da biblioteca Edge Impulse
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"

static const char *TAG = "ESP32_CAM_OV3660";

// Mapeamento dos pinos da ESP32-CAM (Modelo AI-Thinker)
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_Y9      35
#define CAM_PIN_Y8      34
#define CAM_PIN_Y7      39
#define CAM_PIN_Y6      36
#define CAM_PIN_Y5      21
#define CAM_PIN_Y4      19
#define CAM_PIN_Y3      18
#define CAM_PIN_Y2       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// Buffer global para armazenar a imagem convertida para RGB888
static uint8_t *snapshot_buf = NULL;

// Inicialização e configuração da câmara e do sensor OV3660
static esp_err_t init_camera(void) {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_Y2;
    config.pin_d1 = CAM_PIN_Y3;
    config.pin_d2 = CAM_PIN_Y4;
    config.pin_d3 = CAM_PIN_Y5;
    config.pin_d4 = CAM_PIN_Y6;
    config.pin_d5 = CAM_PIN_Y7;
    config.pin_d6 = CAM_PIN_Y8;
    config.pin_d7 = CAM_PIN_Y9;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_siod = CAM_PIN_SIOD;
    config.pin_sioc = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG; // Captura em JPEG para economizar memória RAM

    config.frame_size = FRAMESIZE_QVGA;  // 320x240 pixels
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicializacao da camera: 0x%x", err);
        return err;
    }

    // Ajustes específicos do sensor OV3660 (orientação e cor)
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);      // Inverte imagem verticalmente se necessário
        s->set_hmirror(s, 1);    // Espelha horizontalmente se necessário
        s->set_saturation(s, 0); // Ajuste de saturação
        s->set_brightness(s, 0); // Ajuste de brilho
    }

    return ESP_OK;
}

// Callback do Edge Impulse para extrair e normalizar os píxeis
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    for (size_t i = 0; i < length; i++) {
        uint8_t r = snapshot_buf[pixel_ix];
        uint8_t g = snapshot_buf[pixel_ix + 1];
        uint8_t b = snapshot_buf[pixel_ix + 2];

        // Empacota os canais RGB num valor numérico para o classificador
        float pixel_f = (r << 16) | (g << 8) | b;
        out_ptr[i] = pixel_f;
        pixel_ix += 3;
    }
    return 0;
}

extern "C" void app_main(void) {
    // 1. Inicializa o sensor OV3660
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Erro fatal: Camera nao inicializada.");
        return;
    }
    ESP_LOGI(TAG, "Camera OV3660 inicializada com sucesso.");

    // 2. Aloca a memória para o buffer RGB888 do tamanho exato da entrada do modelo
    size_t resize_buf_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
    snapshot_buf = (uint8_t *)malloc(resize_buf_size);
    if (snapshot_buf == NULL) {
        ESP_LOGE(TAG, "Erro ao alocar memoria para snapshot_buf");
        return;
    }

    ESP_LOGI(TAG, "Iniciando loop de inferencia (%dx%d)...", 
             EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);

    while (1) {
        // 3. Captura o frame
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Falha ao capturar imagem da câmara");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 4. Decodifica e redimensiona a imagem capturada para a resolução do Edge Impulse
        bool conversion_success = ei::ei_camera_cut_and_get_data(
            fb->buf,
            fb->len,
            snapshot_buf,
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT,
            3
        );

        // Devolve o buffer da câmara para o driver
        esp_camera_fb_return(fb);

        if (!conversion_success) {
            ESP_LOGE(TAG, "Erro ao converter/cortar a imagem");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // 5. Configura a estrutura de sinal para a inferência
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &raw_feature_get_data;

        // 6. Executa a inferência
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "Erro na inferencia (%d)", res);
        } else {
            ESP_LOGI(TAG, "Inferência concluída | DSP: %d ms, Classificação: %d ms", 
                     result.timing.dsp, result.timing.classification);

            #if EI_CLASSIFIER_OBJECT_DETECTION == 1
            // Modelo de Deteção de Objetos (Bounding Boxes)
            bool detected = false;
            for (size_t i = 0; i < result.bounding_boxes_count; i++) {
                auto bb = result.bounding_boxes[i];
                if (bb.value >= 0.5f) { // Limiar de confiança em 50%
                    ESP_LOGI(TAG, "Detetado: %s (%.2f) em [ x:%d, y:%d, w:%d, h:%d ]",
                             bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                    detected = true;
                }
            }
            if (!detected) {
                ESP_LOGI(TAG, "Nenhum objeto detetado.");
            }
            #else
            // Modelo de Classificação de Imagem (Labels)
            for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                ESP_LOGI(TAG, "Classe %s: %.2f", 
                         result.classification[i].label, 
                         result.classification[i].value);
            }
            #endif
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Pequena pausa entre inferências
    }
}