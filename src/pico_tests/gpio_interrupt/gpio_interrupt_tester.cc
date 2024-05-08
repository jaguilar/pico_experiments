
#include <cstdio>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "pico/time.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"

#ifndef EMITTER_PIN
#define EMITTER_PIN 6
#endif

#ifndef RECEIVER_PIN
#define RECEIVER_PIN 7
#endif

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                              char *pcTaskName) {
  panic("stack overflow in task %s\n", pcTaskName);
}

namespace jagspico {

// Holds certain events for synchronization between tasks.
EventGroupHandle_t Events() {
  static EventGroupHandle_t events = xEventGroupCreate();
  return events;
}

constexpr TickType_t kEventEmitterTaskReady = 0b1;
constexpr TickType_t kEventShutdown = 0b10;
constexpr TickType_t kEventIrqReceived = 0b100;

bool WaitShutdown(BaseType_t ticks) {
  return xEventGroupWaitBits(Events(), kEventShutdown, 0, pdFALSE, ticks) &
         kEventShutdown;
}

void EmitterTask(void *) {
  gpio_init(EMITTER_PIN);
  gpio_set_dir(EMITTER_PIN, true);
  gpio_put(EMITTER_PIN, 1);
  printf("Emitter task ready (on core %d)\n", get_core_num());
  xEventGroupSetBits(Events(), kEventEmitterTaskReady);
  int count = 0;
  while (!WaitShutdown(pdMS_TO_TICKS(100))) {
    // Make sure the RECEIVER_PIN is what we say it should be.
    assert(gpio_get(RECEIVER_PIN) == 1);
    if ((++count % 16) == 1) {
      printf("Put emitter pin low\n");
    }
    gpio_put(EMITTER_PIN, 0);
    sleep_ms(100);
    // Make sure the RECEIVER_PIN is what we say it should be.
    assert(gpio_get(RECEIVER_PIN) == 0);
    gpio_put(EMITTER_PIN, 1);
  }
  vTaskDelete(nullptr);
  printf("Emitter shutdown\n");
}

void ReceiverTask(void *) {
  gpio_init(RECEIVER_PIN);
  gpio_set_dir(RECEIVER_PIN, false);
  gpio_pull_up(RECEIVER_PIN);

  printf("Waiting for emitter to start (on core %d)\n", get_core_num());
  xEventGroupWaitBits(Events(), kEventEmitterTaskReady, 0, true, portMAX_DELAY);

  // Set up the gpio interrupt.
  vTaskEnterCritical();
  gpio_set_irq_enabled(RECEIVER_PIN, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_callback(+[](uint gpio, uint32_t event_mask) {
    (void)gpio;
    (void)event_mask;
    BaseType_t higher_priorty_task_awoken = false;
    xEventGroupSetBitsFromISR(Events(), kEventIrqReceived,
                              &higher_priorty_task_awoken);
    portYIELD_FROM_ISR(higher_priorty_task_awoken);
  });
  irq_set_enabled(IO_IRQ_BANK0, true);
  vTaskExitCritical();

  printf("IRQ enabled, waiting for message\n");
  TickType_t eventbits = xEventGroupWaitBits(
      Events(), kEventIrqReceived | kEventShutdown, 0, false, portMAX_DELAY);
  if (eventbits & kEventShutdown) {
    printf("FAIL -- no IRQ received\n");
  } else if (eventbits & kEventIrqReceived) {
    printf("PASS\n");
  } else {
    panic("Unexpected event bits: %0x\n", eventbits);
  }
  vTaskDelete(nullptr);
}

#ifndef RECEIVER_TASK_AFFINITY
#define RECEIVER_TASK_AFFINITY 0b01
#endif

void MainTask(void *) {
  // Initialize the events here. It seems like the CXX static initializer guard
  // does not work correctly.
  Events();

  TaskHandle_t emitter_task_handle;
  xTaskCreate(EmitterTask, "EmitterTask", 1024, nullptr, 1,
              &emitter_task_handle);

  TaskHandle_t receiver_task_handle;
  xTaskCreateAffinitySet(ReceiverTask, "ReceiverTask", 1024, nullptr, 1,
                         RECEIVER_TASK_AFFINITY, &receiver_task_handle);

  // Wait up to five seconds for the needful to happen.
  xEventGroupWaitBits(Events(), kEventIrqReceived, 0, true,
                      pdMS_TO_TICKS(5000));
  // Shut it down. This will print an error message if the success condition
  // hasn't been met.
  xEventGroupSetBits(Events(), kEventShutdown);
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

}  // namespace jagspico

int main() {
  stdio_uart_init();
  xTaskCreate(jagspico::MainTask, "MainTask", 1024, nullptr, 1, nullptr);
  vTaskStartScheduler();
}