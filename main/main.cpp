#ifdef __cplusplus
extern "C"
{
#endif

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "math.h"

#include "mfcc.h"
#include "mfcc_list.h"

#ifdef __cplusplus
}
#endif

#define TAG "I2S"

#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "HIDTypes.h"

#define I2S_WS GPIO_NUM_25
#define I2S_SD GPIO_NUM_34
#define I2S_SCK GPIO_NUM_33

#define SAMPLE_RATE 16000
#define BUFFER_SAMPLES 512

#define FFT_SIZE BUFFER_SAMPLES
#define NUM_MEL 20
#define NUM_MFCC 13

#define LOOP_LENGTH 17

#define S1_KEY 0x1E
#define S2_KEY 0x1F
#define S3_KEY 0x20
#define S4_KEY 0x21
#define S5_KEY 0x22
#define S6_KEY 0x23
#define S7_KEY 0x24
#define S8_KEY 0x25
#define S9_KEY 0x26

#define ATK_KEY 0x27

#define C1_KEY 0x3A
#define C2_KEY 0x3B
#define C3_KEY 0x3C
#define C4_KEY 0x3D
#define C5_KEY 0x3E

#define N1_KEY 0x44
#define N2_KEY 0x45
#define N3_KEY 0x46

#define NXT_KEY 0x70
#define CONT_DCD_KEY 0x71
#define BLAP_KEY 0x72

static const int mark_sound[LOOP_LENGTH] =
    {DECISION, DECISION, ATTACK, SCENE_CHANGE,
     ATTACK, SCENE_CHANGE,
     DECISION, DECISION, DECISION, DECISION, ATTACK, EXPERIENCE,
     TAP, TAP, DECISION, DECISION, TAP};

static int phase = 0;
static bool listening = false;
static bool detected = false;
static int silent = 0;
static int check_count = 0;

static NimBLEHIDDevice *hidd = nullptr;
static NimBLECharacteristic *inputReport = nullptr;

static bool ble_connected = false;

// I2S channel handler
static i2s_chan_handle_t rx_chan = NULL;

// Buffers
// 32-bit from INMP441
static int32_t i2s_raw_buffer[BUFFER_SAMPLES];

#define CIRCULAR_BUFFER_SIZE 4092
// Converted to 16-bit
static int16_t scaled_buffer[CIRCULAR_BUFFER_SIZE];

static int idx = 0;

static int16_t pcm[MFCC_FFT_SIZE];
static float mfcc[MFCC_NUM_COEFF];

void i2s_init(void)
{
    // Configure I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    // Create I2S channel with configuration and handle for rx without handle for tx
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // Configure I2S standard mode
    i2s_std_config_t std_cfg = {
        // Clock configuration
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        // Slot configuration with 32-bit data, monoral
        // What is slot?
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 32,
            .ws_pol = false,
            .bit_shift = true,

            .msb_right = false,
        },
        // Set GPIO pins
        // What is MCLK? What is it for?
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD,
            .invert_flags = {
                .mclk_inv = 0,
                .bclk_inv = 0,
                .ws_inv = 0,
            },
        },
    };

    // Initialize I2S channel with standard mode configuration
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    // Enable I2S channel
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "I2S initialized");
}

class CustomServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
        ble_connected = true;
        ESP_LOGI(TAG, "BLE connected");
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
    {
        ble_connected = false;
        ESP_LOGI(TAG, "BLE disconnected");
        NimBLEDevice::startAdvertising();
    }
};

void pressKey(uint8_t modifier, uint8_t keycode)
{
    if (inputReport == nullptr)
    {
        ESP_LOGE(TAG, "Input report not initialized");
        return;
    }

    if (ble_connected)
    {
        // キー押下
        uint8_t report[8] = {modifier, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00};
        inputReport->setValue(report, sizeof(report));
        inputReport->notify(); // 通知

        vTaskDelay(pdMS_TO_TICKS(500));

        // キー離上
        uint8_t release[8] = {};
        inputReport->setValue(release, sizeof(release));
        inputReport->notify();

        ESP_LOGI(TAG, "Sent key event: %d", keycode);
    }
    else
    {
        ESP_LOGW(TAG, "Cannot send key event, BLE not connected");
    }
}

// Standard keyboard HID report descriptor
static const uint8_t hidReportDescriptor[] = {
    USAGE_PAGE(1),
    0x01, // Generic Desktop
    USAGE(1),
    0x06, // Keyboard
    COLLECTION(1),
    0x01, // Application

    // --- Modifier keys (1 byte) ---
    REPORT_ID(1),
    0x01,
    USAGE_PAGE(1),
    0x07, // Keyboard/Keypad
    USAGE_MINIMUM(1),
    0xE0, // Left Ctrl
    USAGE_MAXIMUM(1),
    0xE7, // Right GUI
    LOGICAL_MINIMUM(1),
    0x00,
    LOGICAL_MAXIMUM(1),
    0x01,
    REPORT_SIZE(1),
    0x01,
    REPORT_COUNT(1),
    0x08,
    HIDINPUT(1),
    0x02, // Data, Var, Abs

    // --- Reserved (1 byte) ---
    REPORT_COUNT(1),
    0x01,
    REPORT_SIZE(1),
    0x08,
    HIDINPUT(1),
    0x01, // Const

    // --- Keycodes (6 bytes) ---
    REPORT_COUNT(1),
    0x06,
    REPORT_SIZE(1),
    0x08,
    LOGICAL_MINIMUM(1),
    0x00,
    LOGICAL_MAXIMUM(1),
    0x65,
    USAGE_MINIMUM(1),
    0x00,
    USAGE_MAXIMUM(1),
    0x65,
    HIDINPUT(1),
    0x00, // Data, Array, Abs

    // --- LED output (1 byte) ---
    REPORT_COUNT(1),
    0x05,
    REPORT_SIZE(1),
    0x01,
    USAGE_PAGE(1),
    0x08, // LEDs
    USAGE_MINIMUM(1),
    0x01,
    USAGE_MAXIMUM(1),
    0x05,
    HIDOUTPUT(1),
    0x02, // Data, Var, Abs
    REPORT_COUNT(1),
    0x01,
    REPORT_SIZE(1),
    0x03,
    HIDOUTPUT(1),
    0x01, // Const (padding)

    END_COLLECTION(0),
};

void ble_init(void)
{
    // NimBLE の初期化
    NimBLEDevice::init("FGA");
    // セキュリティ設定
    // Bonding ペアリング情報を保存するか MITM 中間者攻撃(第三者による傍受・改竄)を防ぐ Secure Connections より強力な暗号化アルゴリズム
    NimBLEDevice::setSecurityAuth(true, true, true);

    // サーバーの作成及びコールバック設定
    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new CustomServerCallbacks());

    // 名前設定
    NimBLEDevice::setDeviceName("FGA");

    // HID デバイスの作成
    hidd = new NimBLEHIDDevice(pServer);

    // Report Descriptor
    hidd->setReportMap((uint8_t *)hidReportDescriptor, sizeof(hidReportDescriptor));

    // Report ID 1 (キーボード) の入力レポート特徴？を取得
    inputReport = hidd->getInputReport(1);

    // デバイス情報のセットアップ
    hidd->setManufacturer("Espressif");
    // PnP ID の設定
    // Plug and Play ID
    // 製造元/製品モデル/バージョン情報を示し、ホストが適切なドライバを選択するのに役立つ
    hidd->setPnp(0x02, 0x05ac, 0x820a, 0x0210);
    // HID 情報の設定
    // 国コード 0x00 -> Not Localized, 動作フラグ -> 0x01 RemoteAwake
    hidd->setHidInfo(0x00, 0x01);
    // バッテリーレベルの初期値を 100% に設定
    hidd->setBatteryLevel(100);

    // 入出力能力の設定 -> 入力も出力もない
    // こちら側で承認するタイプのプロトコルを選ばせない
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    // サーバーの開始
    pServer->start();

    // アドバタイズ設定
    NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_KEYBOARD); // キーボードとして表示
    pAdvertising->setName("FGA");
    pAdvertising->addServiceUUID(hidd->getHidService()->getUUID()); // HID サービスの UUID をアドバタイズに追加
    pAdvertising->enableScanResponse(true);
    pAdvertising->start(); // アドバタイズ開始

    ESP_LOGI(TAG, "BLE advertising started");
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_23), // 設定するピンを選択
        .mode = GPIO_MODE_INPUT,               // 入力モードに設定
        .pull_up_en = GPIO_PULLUP_ENABLE,      // 内部プルアップを有効にする
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // プルダウンは無効
        .intr_type = GPIO_INTR_DISABLE         // 今回は割り込みなし（ポーリング）
    };
    gpio_config(&io_conf);

    mfcc_init();

    i2s_init();

    ble_init();

    while (1)
    {
        if (gpio_get_level(GPIO_NUM_23) == 0 && ble_connected)
        {
            if (!listening)
            {
                switch (phase)
                {
                case 0:
                    pressKey(0x00, S4_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S4_KEY);
                    break;
                case 1:
                    pressKey(0x00, S6_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S6_KEY);
                    break;
                case 2:
                    pressKey(0x00, ATK_KEY);
                    break;
                case 3:
                    pressKey(0x00, N3_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C1_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C2_KEY);
                    break;
                case 4:
                    pressKey(0x00, ATK_KEY);
                    break;
                case 5:
                    pressKey(0x00, N2_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C1_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C2_KEY);
                    break;
                case 6:
                    pressKey(0x00, S1_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S1_KEY);
                    break;
                case 7:
                    pressKey(0x00, S2_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S2_KEY);
                    break;
                case 8:
                    pressKey(0x00, S3_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S3_KEY);
                    break;
                case 9:
                    pressKey(0x00, S5_KEY);
                    // vTaskDelay(100);
                    // pressKey(0x00, S5_KEY);
                    break;
                case 10:
                    pressKey(0x00, ATK_KEY);
                    break;
                case 11:
                    pressKey(0x00, N1_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C1_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, C2_KEY);
                    break;
                case 12:
                    pressKey(0x00, NXT_KEY);
                    break;
                case 13:
                    pressKey(0x00, NXT_KEY);
                    break;
                case 14:
                    pressKey(0x00, NXT_KEY); // DECISION SOUND
                    break;
                case 15:
                    pressKey(0x00, CONT_DCD_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, BLAP_KEY);
                    vTaskDelay(100);
                    pressKey(0x00, CONT_DCD_KEY); // DECISION SOUND
                    break;
                case 16:
                    pressKey(0x00, C3_KEY); // TAP SOUND
                    break;
                }
                ESP_LOGI(TAG, "Key Pressed");
                listening = true;
            }
            else
            {
                // Read data
                size_t bytes_read = 0;
                ESP_ERROR_CHECK(
                    i2s_channel_read(
                        // Which channel
                        rx_chan,
                        // Where to store
                        i2s_raw_buffer,
                        // How many bytes
                        sizeof(i2s_raw_buffer),
                        // Actual amount
                        &bytes_read,
                        // Timeout
                        // Forever
                        portMAX_DELAY));

                int samples = bytes_read / sizeof(int32_t);

                int max = i2s_raw_buffer[0] >> 16;
                int min = i2s_raw_buffer[0] >> 16;

                // Write from idx
                for (int i = 0; i < samples; i++)
                {
                    // Convert 32-bit data to 16-bit by right shifting 14 bits, which is equivalent to dividing by 16384
                    // Scaling down + Removing Noise
                    scaled_buffer[idx] = i2s_raw_buffer[i] >> 16;

                    if (scaled_buffer[idx] < min)
                    {
                        min = scaled_buffer[idx];
                    }
                    if (scaled_buffer[idx] > max)
                    {
                        max = scaled_buffer[idx];
                    }

                    idx = (idx + 1) % CIRCULAR_BUFFER_SIZE;
                }

                int start = (idx - MFCC_FFT_SIZE + CIRCULAR_BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
                for (int i = 0; i < MFCC_FFT_SIZE; i++)
                {
                    pcm[i] = scaled_buffer[(start + i) % CIRCULAR_BUFFER_SIZE];
                }

                int threshold = 500;
                int wait_time = 100;
                if(mark_sound[phase] == EXPERIENCE){
                    threshold = 100;
                    wait_time = 200;
                }
                else if (mark_sound[phase] == TAP){
                    threshold = 200;
                    wait_time = 200;
                }
                if (max - min > threshold)
                {
                    if (((mark_sound[phase] != SCENE_CHANGE && mark_sound[phase] != EXPERIENCE) || 30 < silent) && check_count < 3)
                    {
                        vTaskDelay(1);
                        compute_mfcc(pcm, mfcc);

                        float fmfcc[NUM_MFCC];

                        int nn[3] = {-1, -1, -1};
                        float nn_dist[3] = {1e10, 1e10, 1e10};
                        float distance = 0;
                        for (int i = 0; i < LOOP_LENGTH; i++)
                        {
                            for (int j = 0; j < EPOCH_SIZE; j++)
                            {
                                distance = 0;
                                for (int k = 0; k < NUM_MFCC; k++)
                                {
                                    distance += (mfcc[k] - mfcc_list[i][j][k]) * (mfcc[k] - mfcc_list[i][j][k]); // * weight[k] / (mfcc_std[k] * mfcc_std[k]);
                                }
                                if (distance < nn_dist[0])
                                {
                                    nn_dist[2] = nn_dist[1];
                                    nn[2] = nn[1];
                                    nn_dist[1] = nn_dist[0];
                                    nn[1] = nn[0];
                                    nn_dist[0] = distance;
                                    nn[0] = i;
                                    for (int l = 0; l < NUM_MFCC; l++)
                                    {
                                        fmfcc[l] = mfcc_list[i][j][l];
                                    }
                                }
                                else if (distance < nn_dist[1])
                                {
                                    nn_dist[2] = nn_dist[1];
                                    nn[2] = nn[1];
                                    nn_dist[1] = distance;
                                    nn[1] = i;
                                }
                                else if (distance < nn_dist[2])
                                {
                                    nn_dist[2] = distance;
                                    nn[2] = i;
                                }
                            }
                        }
                        ESP_LOGI(TAG, "MFCC: %f %f %f %f %f %f %f %f %f %f %f %f %f", mfcc[0], mfcc[1], mfcc[2], mfcc[3], mfcc[4], mfcc[5], mfcc[6], mfcc[7], mfcc[8], mfcc[9], mfcc[10], mfcc[11], mfcc[12]);
                        ESP_LOGI(TAG, "Mark: %d, NN: %d %d %d, Dist: %f %f %f", mark_sound[phase], nn[0], nn[1], nn[2], nn_dist[0], nn_dist[1], nn_dist[2]);
                        if (nn_dist[0] < 50)
                        {
                            if (
                                (mark_sound[phase] == nn[0]) || (mark_sound[phase] == nn[1] && mark_sound[phase] == nn[2])
                                // (mark_sound[phase] == nn[0] && mark_sound[phase] == nn[1]) ||
                                // (mark_sound[phase] == nn[0] && mark_sound[phase] == nn[2]) ||
                                // (mark_sound[phase] == nn[1] && mark_sound[phase] == nn[2])
                            )
                            {
                                ESP_LOGI(TAG, "Sound Detected");
                                detected = true;
                            }
                        }
                        check_count++;
                    }
                    silent = 0;
                }
                else
                {
                    check_count = 0;
                    silent++;

                    if (silent > wait_time)
                    {
                        // silent = 0;
                        if (detected)
                        {
                            silent = 0;
                            listening = false;
                            detected = false;
                            phase = (phase + 1) % LOOP_LENGTH;
                        }
                        // else
                        // {
                        //     listening = false;
                        // }
                    }
                }
            }
        }

        vTaskDelay(1);
    }

    // // MFCC SAMPLING
    // int prev_range = 0;

    // while (1)
    // {
    //     // Read data
    //     size_t bytes_read = 0;
    //     ESP_ERROR_CHECK(
    //         i2s_channel_read(
    //             // Which channel
    //             rx_chan,
    //             // Where to store
    //             i2s_raw_buffer,
    //             // How many bytes
    //             sizeof(i2s_raw_buffer),
    //             // Actual amount
    //             &bytes_read,
    //             // Timeout
    //             // Forever
    //             portMAX_DELAY));

    //     int samples = bytes_read / sizeof(int32_t);

    //     int max = i2s_raw_buffer[0] >> 16;
    //     int min = i2s_raw_buffer[0] >> 16;

    //     // Write from idx
    //     for (int i = 0; i < samples; i++)
    //     {
    //         // Convert 32-bit data to 16-bit by right shifting 14 bits, which is equivalent to dividing by 16384
    //         // Scaling down + Removing Noise
    //         scaled_buffer[idx] = i2s_raw_buffer[i] >> 16;

    //         if (scaled_buffer[idx] < min)
    //         {
    //             min = scaled_buffer[idx];
    //         }
    //         if (scaled_buffer[idx] > max)
    //         {
    //             max = scaled_buffer[idx];
    //         }

    //         idx = (idx + 1) % CIRCULAR_BUFFER_SIZE;
    //     }

    //     int start = (idx - MFCC_FFT_SIZE + CIRCULAR_BUFFER_SIZE) % CIRCULAR_BUFFER_SIZE;
    //     for (int i = 0; i < MFCC_FFT_SIZE; i++)
    //     {
    //         pcm[i] = scaled_buffer[(start + i) % CIRCULAR_BUFFER_SIZE];
    //     }

    //     vTaskDelay(1);
    //     compute_mfcc(pcm, mfcc);

    //     if (max - min > 200)
    //     {
    //         if (check_count < 3)
    //         {
    //             for (int i = 0; i < MFCC_NUM_COEFF; i++)
    //             {
    //                 printf("%f ", mfcc[i]);
    //             }
    //             printf("\n");
    //         }
    //         silent = 0;
    //         check_count++;
    //     }
    //     else
    //     {
    //         silent++;
    //         check_count = 0;
    //         if (prev_range > 200)
    //         {
    //             printf(".\n");
    //         }
    //     }
    //     prev_range = max - min;

    //     vTaskDelay(1);
    // }
    // //
}
