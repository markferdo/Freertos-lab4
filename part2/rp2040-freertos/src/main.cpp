#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/gpio.h"
#include "PicoOsUart.h"
#include "ssd1306.h"
#include "timers.h"


#include "hardware/timer.h"
#include "event_groups.h"

#define BUTTON_PIN2 7 //sw_2
#define BUTTON_PIN1 8 //sw_1
#define BUTTON_PIN0 9 //sw_0
#define LED_PIN2 20 //D2
#define LED_PIN1 21 //D1
#define LED_PIN0 22 //D0
#define ROTARY_SW 12
#define ROTARY_A 10
#define ROTARY_B 11

extern "C" {
uint32_t read_runtime_ctr(void) {
    return timer_hw->timerawl;
}
}

#include "blinker.h"

SemaphoreHandle_t gpio_sem;
SemaphoreHandle_t charactor_sem; //lab_2

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // signal task that a button was pressed
    xSemaphoreGiveFromISR(gpio_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

struct led_params{
    uint pin;
    uint delay;
};

void modbus_task(void *param);
void display_task(void *param);
void i2c_task(void *param);
extern "C" {
    void tls_test(void);
}
void tls_task(void *param)
{
    tls_test();
    while(true) {
        vTaskDelay(100);
    }
}

void assign_pin(const uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}











//lab4 starts

#define DEBUG_QUEUE_LENGTH 16
#define DEBUG_BUFFER_SIZE 64
//#define BUTTON_BIT (1 << 0)
#define TASK1_BIT (1 << 0)
#define TASK2_BIT (1 << 1)
#define TASK3_BIT (1 << 2)

struct debugEvent {
    const char *format;
    uint32_t data[3];
    TickType_t timestamp;
};

struct SystemObjects {
    QueueHandle_t syslog_q;
    EventGroupHandle_t ev;
};

struct ButtonTaskParams{
    SystemObjects *sys;
    uint gpio;
    uint32_t bit;
    uint taskNum;
};

void debug(const char *format, uint32_t d1, uint32_t d2, uint32_t d3, SystemObjects *s) {
    debugEvent e;
    e.format = format;
    e.data[0] = d1;
    e.data[1] = d2;
    e.data[2] = d3;
    e.timestamp = xTaskGetTickCount();

    xQueueSend(s->syslog_q, &e, portMAX_DELAY);
}

void debugTask(void *pvParameters) {
    SystemObjects *s = (SystemObjects *)pvParameters;
    char buffer[64];
    debugEvent e;

    while (1) {
        if (xQueueReceive(s->syslog_q, &e, portMAX_DELAY) == pdTRUE) { //portMax_delay block forever until a message is available
            snprintf(buffer, sizeof(buffer), e.format, e.data[0], e.data[1], e.data[2]);
            printf("[%lu] %s", (unsigned long)e.timestamp, buffer);
        }
    }
}

void watchdogTask(void *pvParameters) {
    struct SystemObjects *s = (struct SystemObjects *)pvParameters;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            s->ev,
            TASK1_BIT | TASK2_BIT | TASK3_BIT,
            pdTRUE,   // clear on exit
            pdTRUE,   // wait for ALL
            pdMS_TO_TICKS(30000)
        );

        if ((bits & (TASK1_BIT | TASK2_BIT | TASK3_BIT)) ==
            (TASK1_BIT | TASK2_BIT | TASK3_BIT)) {
            TickType_t now = xTaskGetTickCount();
            debug("Watchdog: OK, elapsed %lu ticks\n", (uint32_t)(now - last), 0, 0, s);
            last = now;
            } else {
                debug("Watchdog: FAIL, missing tasks:\n", 0, 0, 0, s);
                if (!(bits & TASK1_BIT)) {
                    debug("  Task1\n", 0, 0, 0, s);
                }
                if (!(bits & TASK2_BIT)) {
                    debug("  Task2\n", 0, 0, 0, s);
                }
                if (!(bits & TASK3_BIT)) {
                    debug("  Task3\n", 0, 0, 0, s);
                }
                debug("Watchdog suspending itself after FAIL\n", 0, 0, 0,s);
                vTaskSuspend(NULL);
            }
    }
}


void buttonTask(void *pvParameters) {
    ButtonTaskParams *p = (ButtonTaskParams *)pvParameters;
    bool last_pressed = false;

    while (1) {
        bool pressed = (gpio_get(p->gpio) == 0);

        if (!pressed && last_pressed) {
            xEventGroupSetBits(p->sys->ev, p->bit);
            debug("Task %d button released -> set bit\n", p->taskNum, 0, 0, p->sys);
        }
        last_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(50)); // debounce
    }
}
//lab4 ends

int main()
{
    //lab_2 starts
    stdio_init_all();
    printf("\nBoot\n");
    gpio_init(LED_PIN1);
    gpio_set_dir(LED_PIN1, true);

    gpio_init(BUTTON_PIN0);
    gpio_set_dir(BUTTON_PIN0, false);
    gpio_pull_up(BUTTON_PIN0);

    gpio_init(BUTTON_PIN1);
    gpio_set_dir(BUTTON_PIN1, false);
    gpio_pull_up(BUTTON_PIN1);

    gpio_init(BUTTON_PIN2);
    gpio_set_dir(BUTTON_PIN2, false);
    gpio_pull_up(BUTTON_PIN2);

    SystemObjects sys;

    sys.syslog_q = xQueueCreate(16, sizeof(debugEvent));
    sys.ev = xEventGroupCreate();

    static ButtonTaskParams p1, p2, p3;
    p1 = (ButtonTaskParams){ &sys, BUTTON_PIN0, TASK1_BIT, 1 };
    p2 = (ButtonTaskParams){ &sys, BUTTON_PIN1, TASK2_BIT, 2 };
    p3 = (ButtonTaskParams){ &sys, BUTTON_PIN2, TASK3_BIT, 3 };

    xTaskCreate(buttonTask, "Task1", 512, &p1, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(buttonTask, "Task2", 512, &p2, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(buttonTask, "Task3", 512, &p3, tskIDLE_PRIORITY + 3, NULL);

    // Watchdog task
    xTaskCreate(watchdogTask, "Watchdog", 512, &sys, tskIDLE_PRIORITY + 2, NULL);

    // Debug task (lowest priority)
    xTaskCreate(debugTask, "DebugTask", 512, &sys, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while(true){};

    // https://www.freertos.org/Documentation/02-Kernel/04-API-references/11-Software-timers/01-xTimerCreate
}















#include <cstdio>
#include "ModbusClient.h"
#include "ModbusRegister.h"

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#if 0
#define UART_NR 0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#else
#define UART_NR 1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#endif

#define BAUD_RATE 9600
#define STOP_BITS 2 // for real system (pico simualtor also requires 2 stop bits)

#define USE_MODBUS

void modbus_task(void *param) {

    const uint led_pin = 22;
    const uint button = 9;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);

    // Initialize chosen serial port
    //stdio_init_all();

    //printf("\nBoot\n");

#ifdef USE_MODBUS
    auto uart{std::make_shared<PicoOsUart>(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE, STOP_BITS)};
    auto rtu_client{std::make_shared<ModbusClient>(uart)};
    ModbusRegister rh(rtu_client, 241, 256);
    ModbusRegister t(rtu_client, 241, 257);
    ModbusRegister produal(rtu_client, 1, 0);
    produal.write(100);
    vTaskDelay((100));
    produal.write(100);
#endif

    while (true) {
#ifdef USE_MODBUS
        gpio_put(led_pin, !gpio_get(led_pin)); // toggle  led
        printf("RH=%5.1f%%\n", rh.read() / 10.0);
        vTaskDelay(5);
        printf("T =%5.1f%%\n", t.read() / 10.0);
        vTaskDelay(3000);
#endif
    }
}

#include "ssd1306os.h"
void display_task(void *param)
{
    auto i2cbus{std::make_shared<PicoI2C>(1, 400000)};
    ssd1306os display(i2cbus);
    display.fill(0);
    display.text("Boot", 0, 0);
    display.show();
    while(true) {
        vTaskDelay(100);
    }
}

void i2c_task(void *param) {
    auto i2cbus{std::make_shared<PicoI2C>(0, 100000)};

    const uint led_pin = 21;
    const uint delay = pdMS_TO_TICKS(250);
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    uint8_t buffer[64] = {0};
    i2cbus->write(0x50, buffer, 2);

    auto rv = i2cbus->read(0x50, buffer, 64);
    printf("rv=%u\n", rv);
    for(int i = 0; i < 64; ++i) {
        printf("%c", isprint(buffer[i]) ? buffer[i] : '_');
    }
    printf("\n");

    buffer[0]=0;
    buffer[1]=64;
    rv = i2cbus->transaction(0x50, buffer, 2, buffer, 64);
    printf("rv=%u\n", rv);
    for(int i = 0; i < 64; ++i) {
        printf("%c", isprint(buffer[i]) ? buffer[i] : '_');
    }
    printf("\n");

    while(true) {

        gpio_put(led_pin, 1);
        vTaskDelay(delay);
        gpio_put(led_pin, 0);
        vTaskDelay(delay);

    }


}