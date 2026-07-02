#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <cmath>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_http_server.h>
#include <esp_netif.h>

#include "motor_controller.h"
#include "servo_controller.h"

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "SmartRobotBoard"

class SmartRobotBoard : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_;

    // 电机（用指针延迟构造，避免无效初始化）
    std::unique_ptr<MotorController> motor_a_;
    std::unique_ptr<MotorController> motor_b_;

    // 舵机（最多2路）
    ServoController servo_[2];
    int servo_count_;

    // 自动停止定时器
    esp_timer_handle_t auto_stop_timer_;
    static constexpr uint32_t AUTO_STOP_MS = 2500;

    // 串口控制
    TaskHandle_t serial_task_handle_;

    static void AutoStopCallback(void* arg) {
        auto* self = static_cast<SmartRobotBoard*>(arg);
        self->motor_a_->Stop();
        self->motor_b_->Stop();
        ESP_LOGI(TAG, "⏱ 自动停止");
    }

    void StartAutoStopTimer() {
        if (esp_timer_is_active(auto_stop_timer_)) {
            esp_timer_stop(auto_stop_timer_);
        }
        esp_timer_start_once(auto_stop_timer_, AUTO_STOP_MS * 1000);
    }

    // 串口命令处理任务
    static void SerialTask(void* arg) {
        auto* self = static_cast<SmartRobotBoard*>(arg);
        uint8_t buf[64];
        while (true) {
            int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf) - 1, pdMS_TO_TICKS(200));
            if (len > 0) {
                ESP_LOGI(TAG, "串口触发 (len=%d)", len);
                auto& app = Application::GetInstance();
                app.Schedule([&app]() {
                    auto state = app.GetDeviceState();
                    if (state == kDeviceStateIdle) {
                        // 唤醒 → 触发语音打招呼
                        app.WakeWordInvoke("你好小智");
                    } else if (state == kDeviceStateSpeaking ||
                               state == kDeviceStateListening) {
                        // 打断说话/听音
                        app.AbortSpeaking(kAbortReasonNone);
                    }
                });
            }
        }
    }

    void InitializeSerialControl() {
        // UART1: RX=IO39, TX=IO38（外部设备唤醒/打断）
        uart_config_t uart_config = {};
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_38, GPIO_NUM_39,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        xTaskCreate(SerialTask, "serial_ctrl", 3072, this, 5, &serial_task_handle_);
        ESP_LOGI(TAG, "串口控制已启动 (UART1 RX=IO39 TX=IO38)");
    }

    // ---- 电机测试网页 ----
    httpd_handle_t motor_http_server_;

    static esp_err_t HttpMotorPage(httpd_req_t* req) {
        const char* html = R"HTML(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>电机测试</title><style>
body{font-family:Arial;text-align:center;padding:20px;background:#1a1a2e;color:#eee}
h2{color:#e94560}.motor{margin:20px 0;padding:15px;background:#16213e;border-radius:10px}
.btn{display:inline-block;padding:15px 25px;margin:5px;border:none;border-radius:8px;font-size:16px;cursor:pointer}
.fwd{background:#0f3460;color:white}.rev{background:#533483;color:white}.stop{background:#e94560;color:white}
.both{background:#0f3460;color:white;padding:15px 30px;font-size:18px;margin:5px}
.speed-row{display:flex;align-items:center;justify-content:center;gap:10px;margin:10px 0}
.speed-row input{width:150px}.speed-row span{min-width:40px;font-size:18px}
</style></head><body>
<h2>电机测试控制台</h2>
<div class="motor"><h3>Motor A (IO1/IO2) - 右轮</h3>
<div class="speed-row"><input type="range" min="0" max="100" value="100" oninput="sa.innerText=this.value"><span id="sa">100</span>%</div>
<button class="btn fwd" onclick="send('/motor/a/forward',getSpeed('sa'))">正转</button>
<button class="btn rev" onclick="send('/motor/a/reverse',getSpeed('sa'))">反转</button>
<button class="btn stop" onclick="send('/motor/a/stop')">停止</button>
</div>
<div class="motor"><h3>Motor B (IO10/IO11) - 左轮</h3>
<div class="speed-row"><input type="range" min="0" max="100" value="100" oninput="sb.innerText=this.value"><span id="sb">100</span>%</div>
<button class="btn fwd" onclick="send('/motor/b/forward',getSpeed('sb'))">正转</button>
<button class="btn rev" onclick="send('/motor/b/reverse',getSpeed('sb'))">反转</button>
<button class="btn stop" onclick="send('/motor/b/stop')">停止</button>
</div>
<div class="motor"><h3>双电机同时</h3>
<button class="btn both" onclick="send('/motor/both/forward',getSpeed('sa'),getSpeed('sb'))">同向正转</button>
<button class="btn both" onclick="send('/motor/both/reverse',getSpeed('sa'),getSpeed('sb'))">同向反转</button>
<button class="btn both" onclick="send('/motor/both/opposite',getSpeed('sa'),getSpeed('sb'))">反向(A正B反)</button>
<button class="btn both" style="background:#e94560" onclick="send('/motor/both/stop')">全部停止</button>
</div>
<script>
function getSpeed(id){return document.getElementById(id).innerText}
function send(url,sa,sb){var u=url;if(sa)u+='?a='+sa;if(sb)u+='&b='+sb;fetch(u,{method:"POST"}).then(r=>r.text()).then(t=>console.log(t))}
</script></body></html>)HTML";
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    static int ParseSpeed(httpd_req_t* req, const char* key, int def) {
        char buf[16];
        if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
            char val[8];
            if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK) {
                int s = atoi(val);
                if (s >= 0 && s <= 100) return s;
            }
        }
        return def;
    }

    static esp_err_t HttpMotorAction(httpd_req_t* req) {
        auto* self = static_cast<SmartRobotBoard*>(req->user_ctx);
        std::string uri(req->uri);
        int a = ParseSpeed(req, "a", 100);
        int b = ParseSpeed(req, "b", 100);
        if (uri.find("/motor/a/forward") != std::string::npos)       { self->motor_a_->Run(a, 1); }
        else if (uri.find("/motor/a/reverse") != std::string::npos)  { self->motor_a_->Run(a, -1); }
        else if (uri.find("/motor/a/stop") != std::string::npos)     { self->motor_a_->Stop(); }
        else if (uri.find("/motor/b/forward") != std::string::npos)  { self->motor_b_->Run(b, 1); }
        else if (uri.find("/motor/b/reverse") != std::string::npos)  { self->motor_b_->Run(b, -1); }
        else if (uri.find("/motor/b/stop") != std::string::npos)     { self->motor_b_->Stop(); }
        else if (uri.find("/motor/both/forward") != std::string::npos) { self->motor_a_->Run(a, 1); self->motor_b_->Run(b, 1); }
        else if (uri.find("/motor/both/reverse") != std::string::npos){ self->motor_a_->Run(a, -1); self->motor_b_->Run(b, -1); }
        else if (uri.find("/motor/both/opposite") != std::string::npos){self->motor_a_->Run(a, 1); self->motor_b_->Run(b, -1); }
        else if (uri.find("/motor/both/stop") != std::string::npos)  { self->motor_a_->Stop(); self->motor_b_->Stop(); }
        httpd_resp_sendstr(req, "OK");
        return ESP_OK;
    }

    void InitializeMotorHttpServer() {
        httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
        cfg.server_port = 8080;
        cfg.uri_match_fn = httpd_uri_match_wildcard;
        if (httpd_start(&motor_http_server_, &cfg) != ESP_OK) {
            ESP_LOGW(TAG, "电机HTTP Server启动失败");
            return;
        }
        httpd_uri_t page = {.uri = "/", .method = HTTP_GET, .handler = HttpMotorPage, .user_ctx = this};
        httpd_register_uri_handler(motor_http_server_, &page);
        httpd_uri_t action = {.uri = "/motor/*", .method = HTTP_POST, .handler = HttpMotorAction, .user_ctx = this};
        httpd_register_uri_handler(motor_http_server_, &action);
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) == ESP_OK) {
            ESP_LOGI(TAG, "电机测试网页: http://" IPSTR ":8080", IP2STR(&ip.ip));
        } else {
            ESP_LOGI(TAG, "电机测试网页: http://<IP>:8080");
        }
    }

    // ---- 舵机初始化（独立 Timer2 + CH4~5） ----
    void InitializeServos() {
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution = LEDC_TIMER_14_BIT;  // ESP32-S3 最大 14-bit
        timer_cfg.timer_num = LEDC_TIMER_2;
        timer_cfg.freq_hz = 50;
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        gpio_num_t servo_pins[] = {SERVO1_PWM_PIN, SERVO2_PWM_PIN};
        ledc_channel_t ch_map[] = {LEDC_CHANNEL_4, LEDC_CHANNEL_5};  // 避免与电机 CH0~3 冲突
        servo_count_ = 0;
        for (int i = 0; i < 2; i++) {
            if (servo_pins[i] == GPIO_NUM_NC) continue;
            servo_[i].Init(servo_pins[i], ch_map[i], LEDC_TIMER_2);
            servo_count_++;
        }
        ESP_LOGI(TAG, "舵机初始化完成 (%d路 / 50Hz)", servo_count_);
    }

    // ---- 电机 LEDC PWM 初始化 ----
    void InitializeMotors() {
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution = LEDC_TIMER_10_BIT;
        timer_cfg.timer_num = LEDC_TIMER_3;
        timer_cfg.freq_hz = 5000;
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        motor_a_ = std::make_unique<MotorController>();
        motor_a_->Init(MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN,
                       LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_TIMER_3);

        motor_b_ = std::make_unique<MotorController>();
        motor_b_->Init(MOTOR_B_IN1_PIN, MOTOR_B_IN2_PIN,
                       LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_TIMER_3);

        ESP_LOGI(TAG, "电机 LEDC 初始化完成 (Timer3/5kHz)");
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

    // ---- MCP 远程控制工具 ----
    // 工具描述面向 AI 大模型，需用中文注明触发场景，
    // 让模型看到描述后能正确映射 "前进"→forward、"后退"→backward 等。
    void RegisterMcpTools() {
        auto& mcp = McpServer::GetInstance();

        // ========== 电机：移动 ==========
        // 两马达对装：直行=反向转动，转弯=同向转动
        mcp.AddTool(
            "self.motor.forward",
            "小车向前直线行驶。适用指令：前进、往前走、向前移动、直行、冲。"
            "speed: 速度百分比 0-100，默认 100",
            PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int speed = props["speed"].value<int>();
                motor_a_->Run(speed, 1);   // A 正转
                motor_b_->Run(speed, -1);  // B 反转
                StartAutoStopTimer();
                ESP_LOGI(TAG, "MCP forward speed=%d", speed);
                return true;
            });

        mcp.AddTool(
            "self.motor.backward",
            "小车向后直线行驶。适用指令：后退、往后退、倒车、向后移动。"
            "speed: 速度百分比 0-100，默认 100",
            PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int speed = props["speed"].value<int>();
                motor_a_->Run(speed, -1);  // A 反转
                motor_b_->Run(speed, 1);   // B 正转
                StartAutoStopTimer();
                ESP_LOGI(TAG, "MCP backward speed=%d", speed);
                return true;
            });

        mcp.AddTool(
            "self.motor.turn_left",
            "小车左转（右轮快/左轮慢，同向差速）。适用指令：左转、向左转、往左拐、转左边。"
            "speed: 速度百分比 0-100，默认 100",
            PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int speed = props["speed"].value<int>();
                motor_a_->Run(speed, 1);             // 右轮 100%
                motor_b_->Run(speed * 65 / 100, 1);  // 左轮 65%
                StartAutoStopTimer();
                ESP_LOGI(TAG, "MCP turn_left speed=%d", speed);
                return true;
            });

        mcp.AddTool(
            "self.motor.turn_right",
            "小车右转（右轮慢/左轮快，同向差速）。适用指令：右转、向右转、往右拐、转右边。"
            "speed: 速度百分比 0-100，默认 100",
            PropertyList({Property("speed", kPropertyTypeInteger, 100, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int speed = props["speed"].value<int>();
                motor_a_->Run(speed * 65 / 100, -1); // 右轮 65%
                motor_b_->Run(speed, -1);             // 左轮 100%
                StartAutoStopTimer();
                ESP_LOGI(TAG, "MCP turn_right speed=%d", speed);
                return true;
            });

        mcp.AddTool(
            "self.motor.stop",
            "立即停止所有电机运动。适用指令：停下、停止、刹车、别动了、站住。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                motor_a_->Stop();
                motor_b_->Stop();
                ESP_LOGI(TAG, "MCP stop");
                return true;
            });

        // ========== 舵机 ==========
        mcp.AddTool(
            "self.servo.set_angle",
            "控制机器人头部/手臂舵机转到指定角度。适用指令：抬头、低头、向左看、向右看、举手。"
            "servo_id: 1=舵机1(GPIO48), 2=舵机2(GPIO13)。angle: 0-180度",
            PropertyList({
                Property("servo_id", kPropertyTypeInteger, 1, 1, 2),
                Property("angle", kPropertyTypeInteger, 90, 0, 180)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int id = props["servo_id"].value<int>() - 1;
                int angle = props["angle"].value<int>();
                if (id < 0 || id >= servo_count_) {
                    char err[64];
                    snprintf(err, sizeof(err), "舵机ID无效，可用范围: 1~%d", servo_count_);
                    return std::string(err);
                }
                servo_[id].SetAngle(angle);
                return true;
            });

        mcp.AddTool(
            "self.servo.get_angle",
            "查询某个舵机当前角度。servo_id: 1=舵机1, 2=舵机2",
            PropertyList({Property("servo_id", kPropertyTypeInteger, 1, 1, 2)}),
            [this](const PropertyList& props) -> ReturnValue {
                int id = props["servo_id"].value<int>() - 1;
                if (id < 0 || id >= servo_count_) return std::string("舵机ID无效");
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"servo_id\":%d,\"angle\":%d}",
                         id + 1, servo_[id].GetAngle());
                return std::string(buf);
            });

        // ========== 状态 ==========
        mcp.AddTool(
            "self.status",
            "获取机器人完整运行状态：包括左右电机的转速/方向，以及两个舵机的当前角度。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"motor_a\":{\"speed\":%d,\"dir\":%d},"
                    "\"motor_b\":{\"speed\":%d,\"dir\":%d},"
                    "\"servo1\":%d,\"servo2\":%d}",
                    motor_a_->GetSpeed(), motor_a_->GetDir(),
                    motor_b_->GetSpeed(), motor_b_->GetDir(),
                    servo_[0].GetAngle(), servo_[1].GetAngle());
                return std::string(buf);
            });

        ESP_LOGI(TAG, "MCP 工具注册完成 (电机×5 + 舵机×2 + 状态×1)");
    }

public:
    SmartRobotBoard()
        : boot_button_(BOOT_BUTTON_GPIO),
          servo_count_(0)
    {
        // 创建自动停止定时器（2.5秒）
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = AutoStopCallback;
        timer_args.arg = this;
        timer_args.name = "auto_stop";
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &auto_stop_timer_));

        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeServos();
        InitializeMotors();
        InitializeTools();
        RegisterMcpTools();
        InitializeSerialControl();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        ESP_LOGI(TAG, "AI瓦力机器人初始化完成");
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        InitializeMotorHttpServer();
    }

    ~SmartRobotBoard() {
        if (auto_stop_timer_) {
            esp_timer_stop(auto_stop_timer_);
            esp_timer_delete(auto_stop_timer_);
        }
        if (motor_http_server_) httpd_stop(motor_http_server_);
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(SmartRobotBoard);
