// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "driver/mcpwm.h"
// #include "driver/gpio.h"
// #include "driver/pcnt.h"
// #include "driver/uart.h"
// #include "esp_err.h"
// #include "esp_timer.h"
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>
// #include <stdarg.h>

// // -----------------------------------------------------------------------------
// // Global UART Mutex for Protecting uart_printf
// // -----------------------------------------------------------------------------
// static SemaphoreHandle_t uartMutex = NULL;

// // -----------------------------------------------------------------------------
// // Helper: uart_printf (sends formatted messages via UART0)
// // -----------------------------------------------------------------------------
// static void uart_printf(const char *fmt, ...)
// {
//     xSemaphoreTake(uartMutex, portMAX_DELAY);
//     char buf[256];
//     va_list args;
//     va_start(args, fmt);
//     vsnprintf(buf, sizeof(buf), fmt, args);
//     va_end(args);
//     uart_write_bytes(UART_NUM_0, buf, strlen(buf));
//     xSemaphoreGive(uartMutex);
// }

// // -----------------------------------------------------------------------------
// // Robot and Encoder Configuration Constants
// // -----------------------------------------------------------------------------
// const uint32_t MOT_PWM_FREQ = 10000;
// const uint8_t MOTL_PWM_F = 2;
// const uint8_t MOTL_PWM_B = 15;
// const uint8_t MOTR_PWM_F = 17;
// const uint8_t MOTR_PWM_B = 5;
// const uint8_t MOTL_ENC_A = 16;
// const uint8_t MOTL_ENC_B = 4;
// const uint8_t MOTR_ENC_A = 18;
// const uint8_t MOTR_ENC_B = 19;
// const float ENCODER_CPR = 1393.1f;

// // -----------------------------------------------------------------------------
// // Drive and Robot Kinematics Parameters
// // -----------------------------------------------------------------------------
// const float WHEEL_BASE = 0.8f;            // meters
// const float WHEEL_RADIUS = 0.254f / 2.0f; // meters (~0.127 m)

// // -----------------------------------------------------------------------------
// // Control Loop and Command Defaults
// // -----------------------------------------------------------------------------
// const float CONTROL_LOOP_FREQUENCY = 100.0f;
// const float sampleTime = 1.0f / CONTROL_LOOP_FREQUENCY;
// const float DEFAULT_VELOCITY = 0.0f;            // m/s
// const float DEFAULT_ANGULAR = 0.0f;             // rad/s
// const float DEFAULT_TRANSLATIONAL_ACCEL = 1.0f; // m/s²
// const float DEFAULT_ANGULAR_ACCEL = 1.0f;       // rad/s²

// // -----------------------------------------------------------------------------
// // UART Configuration (Using UART0 - USB Serial)
// // -----------------------------------------------------------------------------
// const uart_port_t UART_PORT_NUM = UART_NUM_0;
// const int UART_BAUD_RATE = 115200;
// const int UART_TX_PIN = 1; // Default TX for UART0
// const int UART_RX_PIN = 3; // Default RX for UART0
// const int UART_BUF_SIZE = 1024;

// // -----------------------------------------------------------------------------
// // DriveCommand Class
// // -----------------------------------------------------------------------------
// class DriveCommand
// {
// public:
//     DriveCommand()
//         : desiredV(DEFAULT_VELOCITY),
//           desiredW(DEFAULT_ANGULAR),
//           desiredTransAccel(DEFAULT_TRANSLATIONAL_ACCEL),
//           desiredAngularAccel(DEFAULT_ANGULAR_ACCEL)
//     {
//         mutex = xSemaphoreCreateMutex();
//     }
//     ~DriveCommand()
//     {
//         if (mutex)
//         {
//             vSemaphoreDelete(mutex);
//         }
//     }
//     void update(float v, float w, float a, float b)
//     {
//         xSemaphoreTake(mutex, portMAX_DELAY);
//         desiredV = v;
//         desiredW = w;
//         desiredTransAccel = a;
//         desiredAngularAccel = b;
//         xSemaphoreGive(mutex);
//         uart_printf("DriveCommand updated (all): V=%.2f, W=%.2f, A=%.2f, B=%.2f\r\n", v, w, a, b);
//     }
//     void update(float v, float w)
//     {
//         xSemaphoreTake(mutex, portMAX_DELAY);
//         desiredV = v;
//         desiredW = w;
//         xSemaphoreGive(mutex);
//         uart_printf("DriveCommand updated (velocities only): V=%.2f, W=%.2f\r\n", v, w);
//     }
//     void get(float &v, float &w)
//     {
//         xSemaphoreTake(mutex, portMAX_DELAY);
//         v = desiredV;
//         w = desiredW;
//         xSemaphoreGive(mutex);
//     }
//     void getAcceleration(float &a, float &b)
//     {
//         xSemaphoreTake(mutex, portMAX_DELAY);
//         a = desiredTransAccel;
//         b = desiredAngularAccel;
//         xSemaphoreGive(mutex);
//     }

// private:
//     float desiredV;
//     float desiredW;
//     float desiredTransAccel;
//     float desiredAngularAccel;
//     SemaphoreHandle_t mutex;
// };

// // -----------------------------------------------------------------------------
// // Motor Class
// // -----------------------------------------------------------------------------
// class Motor
// {
// public:
//     Motor(mcpwm_timer_t timer, int pwm_pinA, int pwm_pinB)
//         : timer(timer), pwm_pinA(pwm_pinA), pwm_pinB(pwm_pinB)
//     {
//         mcpwm_io_signals_t signalA, signalB;
//         switch (timer)
//         {
//         case MCPWM_TIMER_0:
//             signalA = MCPWM0A;
//             signalB = MCPWM0B;
//             break;
//         case MCPWM_TIMER_1:
//             signalA = MCPWM1A;
//             signalB = MCPWM1B;
//             break;
//         case MCPWM_TIMER_2:
//             signalA = MCPWM2A;
//             signalB = MCPWM2B;
//             break;
//         default:
//             signalA = MCPWM0A;
//             signalB = MCPWM0B;
//             break;
//         }
//         mcpwm_gpio_init(MCPWM_UNIT_0, signalA, pwm_pinA);
//         mcpwm_gpio_init(MCPWM_UNIT_0, signalB, pwm_pinB);
//         mcpwm_config_t pwm_config;
//         pwm_config.frequency = MOT_PWM_FREQ;
//         pwm_config.cmpr_a = 0;
//         pwm_config.cmpr_b = 0;
//         pwm_config.counter_mode = MCPWM_UP_COUNTER;
//         pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//         mcpwm_init(MCPWM_UNIT_0, timer, &pwm_config);
//     }
//     void setSpeed(float speed)
//     {
//         if (speed >= 0)
//         {
//             mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, speed);
//             mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, 0);
//         }
//         else
//         {
//             mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, 0);
//             mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, -speed);
//         }
//     }

// private:
//     mcpwm_timer_t timer;
//     int pwm_pinA;
//     int pwm_pinB;
// };

// // -----------------------------------------------------------------------------
// // Encoder Class (x2 Decoding Using GPIO Interrupts)
// // -----------------------------------------------------------------------------
// class Encoder
// {
// public:
//     Encoder(int enc_a, int enc_b)
//         : pulse_gpio(enc_a), ctrl_gpio(enc_b), count(0)
//     {
//         gpio_config_t io_conf = {};
//         io_conf.intr_type = GPIO_INTR_ANYEDGE;
//         io_conf.mode = GPIO_MODE_INPUT;
//         io_conf.pin_bit_mask = (1ULL << pulse_gpio) | (1ULL << ctrl_gpio);
//         io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
//         io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
//         ESP_ERROR_CHECK(gpio_config(&io_conf));
//         last_count = 0;
//         last_time = esp_timer_get_time();
//         static bool isr_service_installed = false;
//         if (!isr_service_installed)
//         {
//             ESP_ERROR_CHECK(gpio_install_isr_service(0));
//             isr_service_installed = true;
//         }
//         ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)pulse_gpio, gpio_isr_handler, this));
//     }
//     int32_t getCount() { return count; }
//     float getAngularVelocity()
//     {
//         uint64_t now = esp_timer_get_time();
//         int32_t current_count = getCount();
//         uint64_t dt_us = now - last_time;
//         float dt = dt_us / 1e6f;
//         if (dt <= 0)
//             dt = 1e-3f;
//         int32_t delta = current_count - last_count;
//         last_count = current_count;
//         last_time = now;
//         float angular_velocity = (2.0f * 3.141592654f * delta / ENCODER_CPR) / dt;
//         return angular_velocity;
//     }

// private:
//     int pulse_gpio;
//     int ctrl_gpio;
//     volatile int32_t count;
//     int32_t last_count;
//     uint64_t last_time;
//     static void IRAM_ATTR gpio_isr_handler(void *arg)
//     {
//         Encoder *encoder = static_cast<Encoder *>(arg);
//         encoder->handleInterrupt();
//     }
//     void IRAM_ATTR handleInterrupt()
//     {
//         int a = gpio_get_level((gpio_num_t)pulse_gpio);
//         int b = gpio_get_level((gpio_num_t)ctrl_gpio);
//         if (a == b)
//             count++;
//         else
//             count--;
//     }
// };

// // -----------------------------------------------------------------------------
// // PID Class
// // -----------------------------------------------------------------------------
// class PID
// {
// public:
//     PID(float kp, float ki, float kd, float dt, float min_output, float max_output)
//         : kp(kp), ki(ki), kd(kd), dt(dt),
//           min_output(min_output), max_output(max_output),
//           integral(0), prev_error(0)
//     {
//     }
//     float update(float setpoint, float measurement)
//     {
//         float error = setpoint - measurement;
//         integral += error * dt;
//         float derivative = (error - prev_error) / dt;
//         float output = kp * error + ki * integral + kd * derivative;
//         prev_error = error;
//         if (output > max_output)
//             output = max_output;
//         if (output < min_output)
//             output = min_output;
//         return output;
//     }

// private:
//     float kp, ki, kd, dt;
//     float min_output, max_output;
//     float integral;
//     float prev_error;
// };

// // -----------------------------------------------------------------------------
// // UART Initialization and Task (Using UART0)
// // -----------------------------------------------------------------------------
// void init_uart()
// {
//     uart_config_t uart_config = {
//         .baud_rate = UART_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     };
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
//                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
// }

// // -----------------------------------------------------------------------------
// // UART Task: Accumulate characters until termination using 'X'
// // -----------------------------------------------------------------------------
// void uart_task(void *arg)
// {
//     DriveCommand *cmd = (DriveCommand *)arg;
//     uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
//     if (data == NULL)
//     {
//         // Handle allocation error (could log and then suspend or delete the task)
//         vTaskDelete(NULL);
//         return;
//     }
//     // Allocate the lineBuffer from the heap instead of the stack.
//     char *lineBuffer = (char *)malloc(UART_BUF_SIZE);
//     if (lineBuffer == NULL)
//     {
//         free(data);
//         vTaskDelete(NULL);
//         return;
//     }
//     size_t lineIndex = 0;

//     // Optional: delay a few seconds so that boot messages finish.
//     vTaskDelay(pdMS_TO_TICKS(5000));
//     uart_flush_input(UART_PORT_NUM);
//     while (1)
//     {
//         int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
//         if (len > 0)
//         {
//             uart_printf("Raw UART data: %.*s\r\n", len, data);
//             for (int i = 0; i < len; i++)
//             {
//                 char c = (char)data[i];
//                 if (c == 'X')
//                 { // termination marker
//                     // Ensure there's always space for the null terminator.
//                     if (lineIndex < (UART_BUF_SIZE - 1))
//                         lineBuffer[lineIndex] = '\0';
//                     else
//                         lineBuffer[UART_BUF_SIZE - 1] = '\0';

//                     // Convert all characters in lineBuffer to uppercase.
//                     for (size_t j = 0; j < lineIndex; j++)
//                     {
//                         lineBuffer[j] = toupper((unsigned char)lineBuffer[j]);
//                     }

//                     uart_printf("Received line: %s\r\n", lineBuffer);

//                     float v, w, a, b;
//                     int ret = sscanf(lineBuffer, "V:%f,W:%f,A:%f,B:%f", &v, &w, &a, &b);
//                     uart_printf("sscanf returned %d\r\n", ret);
//                     if (ret == 4)
//                     {
//                         cmd->update(v, w, a, b);
//                         uart_printf("Updated command: V=%.2f m/s, W=%.2f rad/s, A=%.2f m/s², B=%.2f rad/s²\r\n", v, w, a, b);
//                     }
//                     else if (ret == 2)
//                     {
//                         cmd->update(v, w);
//                         uart_printf("Updated command: V=%.2f m/s, W=%.2f rad/s\r\n", v, w);
//                     }
//                     else
//                     {
//                         uart_printf("Unrecognized command format: %s\r\n", lineBuffer);
//                     }
//                     memset(lineBuffer, 0, UART_BUF_SIZE);
//                     lineIndex = 0;
//                 }
//                 else
//                 {
//                     // Store the character if there is room.
//                     if (lineIndex < (UART_BUF_SIZE - 1))
//                         lineBuffer[lineIndex++] = c;
//                 }
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
//     free(data);
//     free(lineBuffer);
// }

// // -----------------------------------------------------------------------------
// // Application Entry Point
// // -----------------------------------------------------------------------------
// extern "C" void app_main(void)
// {
//     uartMutex = xSemaphoreCreateMutex();
//     init_uart();
//     DriveCommand driveCmd;
//     // Increase the stack size from 2048 to 4096 bytes for this task.
//     xTaskCreate(uart_task, "uart_task", 4096, &driveCmd, 10, NULL);

//     Motor motorL(MCPWM_TIMER_0, MOTL_PWM_F, MOTL_PWM_B);
//     Motor motorR(MCPWM_TIMER_1, MOTR_PWM_F, MOTR_PWM_B);

//     Encoder encoderL(MOTL_ENC_A, MOTL_ENC_B);
//     Encoder encoderR(MOTR_ENC_A, MOTR_ENC_B);

//     PID pidLeft(30.0f, 1.5f, 0.0f, sampleTime, -50.0f, 50.0f);
//     PID pidRight(30.0f, 1.5f, 0.0f, sampleTime, -50.0f, 50.0f);

//     float filteredV = DEFAULT_VELOCITY;
//     float filteredW = DEFAULT_ANGULAR;

//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS((uint32_t)(1000.0f / CONTROL_LOOP_FREQUENCY));

//     int count = 0;
//     while (1)
//     {
//         float measuredWheelLeft = encoderL.getAngularVelocity();
//         float measuredWheelRight = encoderR.getAngularVelocity();

//         float cmdV, cmdW;
//         driveCmd.get(cmdV, cmdW);
//         float transAccel, angAccel;
//         driveCmd.getAcceleration(transAccel, angAccel);

//         float maxDeltaV = transAccel * sampleTime;
//         float deltaV = cmdV - filteredV;
//         if (deltaV > maxDeltaV)
//             deltaV = maxDeltaV;
//         else if (deltaV < -maxDeltaV)
//             deltaV = -maxDeltaV;
//         filteredV += deltaV;

//         float maxDeltaW = angAccel * sampleTime;
//         float deltaW = cmdW - filteredW;
//         if (deltaW > maxDeltaW)
//             deltaW = maxDeltaW;
//         else if (deltaW < -maxDeltaW)
//             deltaW = -maxDeltaW;
//         filteredW += deltaW;

//         float leftWheelSetpoint = (filteredV - (filteredW * WHEEL_BASE / 2.0f)) / WHEEL_RADIUS;
//         float rightWheelSetpoint = (filteredV + (filteredW * WHEEL_BASE / 2.0f)) / WHEEL_RADIUS;

//         float outputLeft = pidLeft.update(leftWheelSetpoint, measuredWheelLeft);
//         float outputRight = pidRight.update(rightWheelSetpoint, measuredWheelRight);

//         motorL.setSpeed(outputLeft);
//         motorR.setSpeed(outputRight);

//         if (count > 100)
//         {
//             uart_printf("Cmd: V=%.2f,W=%.2f | Filt: V=%.2f,W=%.2f | L: set=%.2f, meas=%.2f, out=%.2f | R: set=%.2f, meas=%.2f, out=%.2f\r\n",
//                         cmdV, cmdW, filteredV, filteredW,
//                         leftWheelSetpoint, measuredWheelLeft, outputLeft,
//                         rightWheelSetpoint, measuredWheelRight, outputRight);
//             count = 0;
//         }
//         count++;
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <rom/ets_sys.h>

// Global UART Mutex for Protecting uart_printf
static SemaphoreHandle_t uartMutex = NULL;

// Helper: uart_printf (sends formatted messages via UART0)
static void uart_printf(const char *fmt, ...)
{
    xSemaphoreTake(uartMutex, portMAX_DELAY);
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write_bytes(UART_NUM_0, buf, strlen(buf));
    xSemaphoreGive(uartMutex);
}

// Robot and Encoder Configuration Constants
const uint32_t MOT_PWM_FREQ = 10000;
const uint32_t SER_PWM_FREQ = 50;
const uint8_t MOTL_PWM_F = 2;
const uint8_t MOTL_PWM_B = 15;
const uint8_t MOTR_PWM_F = 17;
const uint8_t MOTR_PWM_B = 5;
const uint8_t MOTD_PWM_F = 21;
const uint8_t MOTD_PWM_B = 22;
const uint8_t MOTL_ENC_A = 16;
const uint8_t MOTL_ENC_B = 4;
const uint8_t MOTR_ENC_A = 18;
const uint8_t MOTR_ENC_B = 19;
const uint8_t MOTD_ENC_A = 12;
const uint8_t MOTD_ENC_B = 13;
const uint8_t SERS_PWM = 23;
const float WHEEL_ENCODER_CPR = 1393.1f;
const float DISK_ENCODER_CPR = 864.0f;
const uint8_t US_TRIGGER_PIN = 32;
const uint8_t US_ECHO_FRONT_PINL = 36;
const uint8_t US_ECHO_FRONT_PINR = 33;
const uint8_t US_ECHO_RIGHT_PIN = 39;
const uint8_t US_ECHO_LEFT_PIN = 34;
const uint8_t US_ECHO_BACK_PIN = 35;
const uint8_t HEADLIGHT_PIN = 25;


// Drive and Robot Kinematics Parameters
const float WHEEL_BASE = 0.871f;            // meters
const float WHEEL_RADIUS = (0.254f / 2.0f)/1.2f; // meters (~0.127 m)

// Control Loop and Command Defaults
const float CONTROL_LOOP_FREQUENCY = 100.0f;
const float sampleTime = 1.0f / CONTROL_LOOP_FREQUENCY;
const float DEFAULT_VELOCITY = 0.0f; // m/s
const float DEFAULT_ANGULAR = 0.0f;  // rad/s
const float DEFAULT_DISK = 0;
const float DEFAULT_TRANSLATIONAL_ACCEL = 1.0f; // m/s²
const float DEFAULT_ANGULAR_ACCEL = 1.0f;       // rad/s²
const float DEFAULT_DISK_ACCEL = 0;

// UART Configuration (Using UART0 - USB Serial)
const uart_port_t UART_PORT_NUM = UART_NUM_0;
const int UART_BAUD_RATE = 115200;
const int UART_TX_PIN = 1;
const int UART_RX_PIN = 3;
const int UART_BUF_SIZE = 1024;

// DriveCommand Class
class DriveCommand
{
public:
    DriveCommand()
        : desiredV(DEFAULT_VELOCITY),
          desiredW(DEFAULT_ANGULAR),
          desiredD(DEFAULT_DISK),
          desiredTransAccel(DEFAULT_TRANSLATIONAL_ACCEL),
          desiredAngularAccel(DEFAULT_ANGULAR_ACCEL),
          desiredDiskAccel(DEFAULT_DISK_ACCEL)
    {
        mutex = xSemaphoreCreateMutex();
    }
    ~DriveCommand()
    {
        if (mutex)
        {
            vSemaphoreDelete(mutex);
        }
    }

    void update(float v, float w, float d, float s, float a, float b, float c)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        desiredV = v;
        desiredW = w;
        desiredD = d;
        desiredS = s;
        desiredTransAccel = a;
        desiredAngularAccel = b;
        desiredDiskAccel = c;
        xSemaphoreGive(mutex);
        uart_printf("DriveCommand updated (all): V=%.2f, W=%.2f, D=%.2f, S=%.2f, A=%.2f, B=%.2f, C=%.2f\r\n", v, w, d, s, a, b, c);
    }

    void update(float v, float w, float d, float s)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        desiredV = v;
        desiredW = w;
        desiredD = d;
        desiredS = s;
        xSemaphoreGive(mutex);
        uart_printf("DriveCommand updated (all): V=%.2f, W=%.2f, D=%.2f, S=%.2f\r\n", v, w, d, s);
    }
    void update(float v, float w, float d)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        desiredV = v;
        desiredW = w;
        desiredD = d;
        xSemaphoreGive(mutex);
        uart_printf("DriveCommand updated (velocities only): V=%.2f, W=%.2f, D=%.2f\r\n", v, w, d);
    }
    void getVel(float &v, float &w, float &d)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        v = desiredV;
        w = desiredW;
        d = desiredD;
        xSemaphoreGive(mutex);
    }
    void getSer(float &s)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        s = desiredS;
        xSemaphoreGive(mutex);
    }
    void getAccel(float &a, float &b, float &c)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        a = desiredTransAccel;
        b = desiredAngularAccel;
        c = desiredDiskAccel;
        xSemaphoreGive(mutex);
    }

private:
    float desiredV;
    float desiredW;
    float desiredD;
    float desiredS;
    float desiredTransAccel;
    float desiredAngularAccel;
    float desiredDiskAccel;
    SemaphoreHandle_t mutex;
};

// Motor Class
class Motor
{
public:
    Motor(mcpwm_timer_t timer, int pwm_pinA, int pwm_pinB)
        : timer(timer), pwm_pinA(pwm_pinA), pwm_pinB(pwm_pinB)
    {
        mcpwm_io_signals_t signalA, signalB;
        switch (timer)
        {
        case MCPWM_TIMER_0:
            signalA = MCPWM0A;
            signalB = MCPWM0B;
            break;
        case MCPWM_TIMER_1:
            signalA = MCPWM1A;
            signalB = MCPWM1B;
            break;
        case MCPWM_TIMER_2:
            signalA = MCPWM2A;
            signalB = MCPWM2B;
            break;
        default:
            signalA = MCPWM0A;
            signalB = MCPWM0B;
            break;
        }
        mcpwm_gpio_init(MCPWM_UNIT_0, signalA, pwm_pinA);
        mcpwm_gpio_init(MCPWM_UNIT_0, signalB, pwm_pinB);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = MOT_PWM_FREQ;
        pwm_config.cmpr_a = 0;
        pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_0, timer, &pwm_config);
    }
    void setSpeed(float speed)
    {
        if (speed >= 0)
        {
            mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, speed);
            mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, 0);
        }
        else
        {
            mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, -speed);
        }
    }

private:
    mcpwm_timer_t timer;
    int pwm_pinA;
    int pwm_pinB;
};

// Encoder Class (x2 Decoding Using GPIO Interrupts)
class Encoder
{
public:
    Encoder(int enc_a, int enc_b, float cpr)
        : pulse_gpio(enc_a), ctrl_gpio(enc_b), cpr(cpr), count(0)
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << pulse_gpio) | (1ULL << ctrl_gpio);
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        last_count = 0;
        last_time = esp_timer_get_time();
        static bool isr_service_installed = false;
        if (!isr_service_installed)
        {
            ESP_ERROR_CHECK(gpio_install_isr_service(0));
            isr_service_installed = true;
        }
        ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)pulse_gpio, gpio_isr_handler, this));
    }
    int32_t getCount() { return count; }
    float getAngularVelocity()
    {
        uint64_t now = esp_timer_get_time();
        int32_t current_count = getCount();
        uint64_t dt_us = now - last_time;
        float dt = dt_us / 1e6f;
        if (dt <= 0)
            dt = 1e-3f;
        int32_t delta = current_count - last_count;
        last_count = current_count;
        last_time = now;
        float angular_velocity = (2.0f * 3.141592654f * delta / cpr) / dt;
        return angular_velocity;
    }

private:
    int pulse_gpio;
    int ctrl_gpio;
    float cpr;
    volatile int32_t count;
    int32_t last_count;
    uint64_t last_time;
    static void IRAM_ATTR gpio_isr_handler(void *arg)
    {
        Encoder *encoder = static_cast<Encoder *>(arg);
        encoder->handleInterrupt();
    }
    void IRAM_ATTR handleInterrupt()
    {
        int a = gpio_get_level((gpio_num_t)pulse_gpio);
        int b = gpio_get_level((gpio_num_t)ctrl_gpio);
        if (a == b)
            count++;
        else
            count--;
    }
};

class Servo
{
public:
    Servo(mcpwm_timer_t timer, int pwm_pinA, int rom)
        : timer(timer), pwm_pinA(pwm_pinA), hrom(rom/2)
    {
        mcpwm_io_signals_t signalA;
        switch (timer)
        {
        case MCPWM_TIMER_0:
            signalA = MCPWM0A;
            break;
        case MCPWM_TIMER_1:
            signalA = MCPWM1A;
            break;
        case MCPWM_TIMER_2:
            signalA = MCPWM2A;
            break;
        default:
            signalA = MCPWM0A;
            break;
        }
        mcpwm_gpio_init(MCPWM_UNIT_1, signalA, pwm_pinA);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = SER_PWM_FREQ;
        pwm_config.cmpr_a = 7.5f;
        pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_1, timer, &pwm_config);
    }
    void set_servo_angle(float angle)
    {
        if (angle < -hrom)
        {
            angle = -hrom;
        }
        else if (angle > hrom)
        {
            angle = hrom;
        }

        float pulse_us = 1500.0f + (angle * (500.0f / hrom));

        float period_us = 1000000.0f / SER_PWM_FREQ;

        float duty_percent = (pulse_us / period_us) * 100.0f;

        mcpwm_set_duty(MCPWM_UNIT_1, timer, MCPWM_OPR_A, duty_percent);
    }

private:
    mcpwm_timer_t timer;
    int pwm_pinA;
    int hrom;
};

// PID Class
class PID
{
public:
    PID(float kp, float ki, float kd, float dt, float min_output, float max_output)
        : kp(kp), ki(ki), kd(kd), dt(dt),
          min_output(min_output), max_output(max_output),
          integral(0), prev_error(0)
    {
    }
    float update(float setpoint, float measurement)
    {
        float error = setpoint - measurement;
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        float output = kp * error + ki * integral + kd * derivative;
        prev_error = error;
        if (output > max_output)
            output = max_output;
        if (output < min_output)
            output = min_output;
        return output;
    }

private:
    float kp, ki, kd, dt;
    float min_output, max_output;
    float integral;
    float prev_error;
};

// Ultrasonic Sensor Section
struct UltrasonicMeasurements
{
    float frontL;
    float frontR;
    float right;
    float left;
    float back;
};

void init_ultrasonic()
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << US_TRIGGER_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << US_ECHO_FRONT_PINL) | (1ULL << US_ECHO_FRONT_PINR) | (1ULL << US_ECHO_RIGHT_PIN) |
                           (1ULL << US_ECHO_LEFT_PIN) | (1ULL << US_ECHO_BACK_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

UltrasonicMeasurements measure_ultrasonic_all()
{
    UltrasonicMeasurements meas = {-1, -1, -1, -1, -1};

    // Ensure trigger is low, then send a 10 µs HIGH pulse.
    gpio_set_level((gpio_num_t)US_TRIGGER_PIN, 0);
    ets_delay_us(2);
    gpio_set_level((gpio_num_t)US_TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level((gpio_num_t)US_TRIGGER_PIN, 0);

    uint32_t start_time_frontL = 0, start_time_frontR = 0, start_time_right = 0, start_time_left = 0, start_time_back = 0;
    uint32_t end_time_frontL = 0, end_time_frontR = 0, end_time_right = 0, end_time_left = 0, end_time_back = 0;
    bool frontL_done = false, frontR_done = false, right_done = false, left_done = false, back_done = false;

    // Set a timeout (30ms)
    uint32_t timeout = esp_timer_get_time() + 30000;

    while (!(frontL_done && frontR_done && right_done && left_done && back_done))
    {
        uint32_t now = esp_timer_get_time();
        if (now > timeout)
            break;

        // Front sensor
        if (!frontL_done)
        {
            int level = gpio_get_level((gpio_num_t)US_ECHO_FRONT_PINL);
            if (start_time_frontL == 0 && level == 1)
            {
                start_time_frontL = now;
            }
            else if (start_time_frontL != 0 && level == 0 && end_time_frontL == 0)
            {
                end_time_frontL = now;
                frontL_done = true;
            }
        }
        if (!frontR_done)
        {
            int level = gpio_get_level((gpio_num_t)US_ECHO_FRONT_PINR);
            if (start_time_frontR == 0 && level == 1)
            {
                start_time_frontR = now;
            }
            else if (start_time_frontR != 0 && level == 0 && end_time_frontR == 0)
            {
                end_time_frontR = now;
                frontL_done = true;
            }
        }
        // Right sensor
        if (!right_done)
        {
            int level = gpio_get_level((gpio_num_t)US_ECHO_RIGHT_PIN);
            if (start_time_right == 0 && level == 1)
            {
                start_time_right = now;
            }
            else if (start_time_right != 0 && level == 0 && end_time_right == 0)
            {
                end_time_right = now;
                right_done = true;
            }
        }
        // Left sensor
        if (!left_done)
        {
            int level = gpio_get_level((gpio_num_t)US_ECHO_LEFT_PIN);
            if (start_time_left == 0 && level == 1)
            {
                start_time_left = now;
            }
            else if (start_time_left != 0 && level == 0 && end_time_left == 0)
            {
                end_time_left = now;
                left_done = true;
            }
        }
        // Back sensor
        if (!back_done)
        {
            int level = gpio_get_level((gpio_num_t)US_ECHO_BACK_PIN);
            if (start_time_back == 0 && level == 1)
            {
                start_time_back = now;
            }
            else if (start_time_back != 0 && level == 0 && end_time_back == 0)
            {
                end_time_back = now;
                back_done = true;
            }
        }
        taskYIELD();
    }

    if (start_time_frontL && end_time_frontL)
        meas.frontL = (end_time_frontL - start_time_frontL) / 58.0f;
    if (start_time_frontR && end_time_frontR)
        meas.frontR = (end_time_frontR - start_time_frontR) / 58.0f;
    if (start_time_right && end_time_right)
        meas.right = (end_time_right - start_time_right) / 58.0f;
    if (start_time_left && end_time_left)
        meas.left = (end_time_left - start_time_left) / 58.0f;
    if (start_time_back && end_time_back)
        meas.back = (end_time_back - start_time_back) / 58.0f;

    return meas;
}

void ultrasonic_task(void *arg)
{
    DriveCommand *cmd = (DriveCommand *)arg;
    bool stop = false;
    int count = 0;
    while (1)
    {
        UltrasonicMeasurements meas = measure_ultrasonic_all();
        if ((meas.frontL < 0.5 || meas.frontR < 0.5 || meas.right < 0.2 || meas.left < 0.2 || meas.back < 0.2) && !stop)
        {
            cmd->update(0, 0, 0);
            stop = true;
            uart_printf("Obstacle detected - Stopping...\r\n");
        }
        if (!(meas.frontL < 0.5 || meas.frontR < 0.5 || meas.right < 0.2 || meas.left < 0.2 || meas.back < 0.2) && stop)
        {
            cmd->update(0, 0, 0);
            stop = false;
            uart_printf("Obstacle cleared - Resuming...\r\n");
        }
        if (count > 100)
        {
            uart_printf("Ultrasonic: Front: %.2f cm, Right: %.2f cm, Left: %.2f cm, Back: %.2f cm\r\n",
                        meas.frontL, meas.frontR, meas.right, meas.left, meas.back);
            count = 0;
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// UART Initialization and Task (Using UART0)
void init_uart()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
}

// UART Task: Accumulate characters until termination using 'X'
void uart_task(void *arg)
{
    DriveCommand *cmd = (DriveCommand *)arg;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL)
    {
        vTaskDelete(NULL);
        return;
    }
    char *lineBuffer = (char *)malloc(UART_BUF_SIZE);
    if (lineBuffer == NULL)
    {
        free(data);
        vTaskDelete(NULL);
        return;
    }
    size_t lineIndex = 0;
    vTaskDelay(pdMS_TO_TICKS(5000));
    uart_flush_input(UART_PORT_NUM);
    while (1)
    {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            uart_printf("Raw UART data: %.*s\r\n", len, data);
            for (int i = 0; i < len; i++)
            {
                char ch = (char)data[i];
                if (ch == 'X')
                {
                    if (lineIndex < (UART_BUF_SIZE - 1))
                        lineBuffer[lineIndex] = '\0';
                    else
                        lineBuffer[UART_BUF_SIZE - 1] = '\0';
                    for (size_t j = 0; j < lineIndex; j++)
                    {
                        lineBuffer[j] = toupper((unsigned char)lineBuffer[j]);
                    }
                    uart_printf("Received line: %s\r\n", lineBuffer);
                    float v, w, d, s, a, b, c;
                    int ret = sscanf(lineBuffer, "V:%f,W:%f,D:%f,S:%f,A:%f,B:%f,C:%f", &v, &w, &d, &s, &a, &b, &c);
                    uart_printf("sscanf returned %d\r\n", ret);
                    if (ret == 7)
                    {
                        cmd->update(v, w, d, s, a, b, c);
                        uart_printf("Updated command: V=%.2f m/s, W=%.2f rad/s, D=%.2f m/s², S=%.2f rad/s², A=%.2f m/s², B=%.2f rad/s², C=%.2f\r\n", v, w, d, s, a, b, c);
                    }
                    else if (ret == 4)
                    {
                        cmd->update(v, w, d, s);
                        uart_printf("Updated command: V=%.2f m/s, W=%.2f rad/s, D=%.2f m/s², S=%.2f\r\n", v, w, d, s);
                    }
                    else if (ret == 3)
                    {
                        cmd->update(v, w, d);
                        uart_printf("Updated command: V=%.2f m/s, W=%.2f rad/s\r\n", v, w, d);
                    }
                    else
                    {
                        uart_printf("Unrecognized command format: %s\r\n", lineBuffer);
                    }
                    memset(lineBuffer, 0, UART_BUF_SIZE);
                    lineIndex = 0;
                }
                else
                {
                    if (lineIndex < (UART_BUF_SIZE - 1))
                        lineBuffer[lineIndex++] = ch;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    free(data);
    free(lineBuffer);
}

// main
extern "C" void app_main(void)
{
    uartMutex = xSemaphoreCreateMutex();
    init_uart();
    init_ultrasonic();
    DriveCommand driveCmd;
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, &driveCmd, 5, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, &driveCmd, 10, NULL);

    Motor motorL(MCPWM_TIMER_0, MOTL_PWM_F, MOTL_PWM_B);
    Motor motorR(MCPWM_TIMER_1, MOTR_PWM_F, MOTR_PWM_B);
    Motor motorD(MCPWM_TIMER_2, MOTD_PWM_F, MOTD_PWM_B);

    Encoder encoderL(MOTL_ENC_A, MOTL_ENC_B, WHEEL_ENCODER_CPR);
    Encoder encoderR(MOTR_ENC_A, MOTR_ENC_B, WHEEL_ENCODER_CPR);
    Encoder encoderD(MOTD_ENC_A, MOTD_ENC_B, DISK_ENCODER_CPR);

    Servo servoS(MCPWM_TIMER_0, SERS_PWM, 270);

    PID pidLeft(10.0f, 0.0f, 0.0f, sampleTime, -50.0f, 50.0f);
    PID pidRight(10.0f, 0.0f, 0.0f, sampleTime, -50.0f, 50.0f);
    PID pidDisk(10.0f, 0.0f, 0.0f, sampleTime, -50.0f, 50.0f);

    float filteredV = DEFAULT_VELOCITY;
    float filteredW = DEFAULT_ANGULAR;
    float filteredD = DEFAULT_DISK;

    gpio_config_t headlight_config = {};
    headlight_config.intr_type = GPIO_INTR_DISABLE;
    headlight_config.mode = GPIO_MODE_OUTPUT;
    headlight_config.pin_bit_mask = (1ULL << HEADLIGHT_PIN);
    headlight_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    headlight_config.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&headlight_config));

    gpio_set_level((gpio_num_t)HEADLIGHT_PIN, 1);
    const TickType_t xFrequency = pdMS_TO_TICKS((uint32_t)(1000.0f / CONTROL_LOOP_FREQUENCY));
    TickType_t xLastWakeTime = xTaskGetTickCount();
    //
    //uart_printf("Starting control loop...\r\n");
    //
    int count = 0;
    while (1)
    {
        //uart_printf("Control loop iteration %d\r\n", count);
        float measuredWheelLeft = encoderL.getAngularVelocity();
        float measuredWheelRight = encoderR.getAngularVelocity();
        float measuredWheelDisk = encoderD.getAngularVelocity();

        float cmdV, cmdW, cmdD, cmdS;
        driveCmd.getVel(cmdV, cmdW, cmdD);
        float transAccel, angAccel, diskAccel;
        driveCmd.getSer(cmdS);
        driveCmd.getAccel(transAccel, angAccel, diskAccel);

        float maxDeltaV = transAccel * sampleTime;
        float deltaV = cmdV - filteredV;
        if (deltaV > maxDeltaV)
            deltaV = maxDeltaV;
        else if (deltaV < -maxDeltaV)
            deltaV = -maxDeltaV;
        filteredV += deltaV;

        float maxDeltaW = angAccel * sampleTime;
        float deltaW = cmdW - filteredW;
        if (deltaW > maxDeltaW)
            deltaW = maxDeltaW;
        else if (deltaW < -maxDeltaW)
            deltaW = -maxDeltaW;
        filteredW += deltaW;

        float maxDeltaD = diskAccel * sampleTime;
        float deltaD = cmdD - filteredD;
        if (deltaD > maxDeltaD)
            deltaD = maxDeltaD;
        else if (deltaD < -maxDeltaD)
            deltaD = -maxDeltaD;
        filteredD += deltaD;

        float leftWheelSetpoint = (filteredV - (filteredW * WHEEL_BASE / 2.0f)) / WHEEL_RADIUS;
        float rightWheelSetpoint = (filteredV + (filteredW * WHEEL_BASE / 2.0f)) / WHEEL_RADIUS;
        float diskWheelSetpoint = filteredD;

        float outputLeft = pidLeft.update(leftWheelSetpoint, measuredWheelLeft);
        float outputRight = pidRight.update(rightWheelSetpoint, measuredWheelRight);
        float outputDisk = pidDisk.update(diskWheelSetpoint, measuredWheelDisk);

        motorL.setSpeed(outputLeft);
        motorR.setSpeed(outputRight);
        motorR.setSpeed(outputDisk);

        servoS.set_servo_angle(cmdS);

        if (count > 100)
        {
            uart_printf("Cmd: V=%.2f,W=%.2f | Filt: V=%.2f,W=%.2f | L: set=%.2f, meas=%.2f, out=%.2f | R: set=%.2f, meas=%.2f, out=%.2f | D: set=%.2f, meas=%.2f, out=%.2f\r\n",
                        cmdV, cmdW, filteredV, filteredW,
                        leftWheelSetpoint, measuredWheelLeft, outputLeft,
                        rightWheelSetpoint, measuredWheelRight, outputRight,
                        diskWheelSetpoint, measuredWheelDisk, outputDisk);
            count = 0;
        }
        count++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
