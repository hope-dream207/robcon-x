#include "control_Cylinder.h"
#include "stm32f4xx_hal_gpio.h"

extern int16_t g_sbus_channels[18];

typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  const char* name;
} Cylinder_Pin_t;

static const Cylinder_Pin_t cylinder_pins[] = {
  {GPIOB, GPIO_PIN_5, "LIFT"},      // 升气缸
  {GPIOB, GPIO_PIN_4, "CLAW1"},     // 爪子1
  {GPIOB, GPIO_PIN_3, "CLAW2"},     // 爪子2
  {GPIOB, GPIO_PIN_1, "EXTEND"},    // 伸出气缸1，2
  {GPIOE, GPIO_PIN_5, "SUCKER"}     // 吸盘
};

// static const Cylinder_Pin_t cylinder_pins[] = {
//   {GPIOB, GPIO_PIN_10, "SUCKER"},    // 吸盘
//   {GPIOA, GPIO_PIN_2,  "EXTEND"},    // 伸出气缸1，2
//   {GPIOA, GPIO_PIN_3,  "CLAW1"},     // 爪子1
//   {GPIOA, GPIO_PIN_9, "LIFT"},      // 爪子上升
//   {GPIOA, GPIO_PIN_10,  "CLAW2"}      // 爪子2
// };

#define CYLINDER_PIN_COUNT (sizeof(cylinder_pins) / sizeof(Cylinder_Pin_t))
#define CYLINDER_ON_THRESHOLD  300

static uint8_t cylinder_state[5] = {0};

void Cylinder_Init(void) {
  for (uint8_t i = 0; i < CYLINDER_PIN_COUNT; i++) {
    cylinder_state[i] = 0;
    HAL_GPIO_WritePin(cylinder_pins[i].port, cylinder_pins[i].pin, GPIO_PIN_RESET);
  }
}

static void set_cylinder_by_channel(uint8_t channel, uint8_t pin_index) {
  if (pin_index >= CYLINDER_PIN_COUNT) return;

  const int16_t ch = g_sbus_channels[channel];
  if (ch >= CYLINDER_ON_THRESHOLD) {
    cylinder_state[pin_index] = 1;
  } else {
    cylinder_state[pin_index] = 0;
  }

  GPIO_PinState state = cylinder_state[pin_index] ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(cylinder_pins[pin_index].port, cylinder_pins[pin_index].pin, state);
}

static void force_cylinder_off(uint8_t pin_index) {
  if (pin_index >= CYLINDER_PIN_COUNT) return;
  cylinder_state[pin_index] = 0;
  HAL_GPIO_WritePin(cylinder_pins[pin_index].port, cylinder_pins[pin_index].pin, GPIO_PIN_RESET);
}

static uint8_t lift_saved = 0;

void Cylinder_Control(void) {
  set_cylinder_by_channel(8, 1);  // 爪子1
  set_cylinder_by_channel(9, 2);  // 爪子2,2
  set_cylinder_by_channel(7, 0);  // 升气缸
  force_cylinder_off(4);          // 吸盘
  lift_saved = cylinder_state[0]; // 保存升气缸状态，供GET模式使用
}

void Cylinder_ForceOff(uint8_t pin_index) {
  force_cylinder_off(pin_index);
}

void Cylinder_Controlx(void) {
  set_cylinder_by_channel(7, 4);  // 吸盘
  set_cylinder_by_channel(6, 3);  // 伸出气缸1，2

  cylinder_state[0] = lift_saved;
  GPIO_PinState state = lift_saved ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(cylinder_pins[0].port, cylinder_pins[0].pin, state);

  force_cylinder_off(1);          // 爪子1
  force_cylinder_off(2);          // 爪子2
}