#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Pin definitions
#define BUTTON_PIN 2           // Input pin for the button
#define RELAY_PIN 3            // Output pin for the relay
#define CURRENT_SENSOR_PIN A2  // Input pin for the current sensor (0.2 - 2.8V)
#define RX_PIN 8               // RX pin for RS485 comm
#define MAX485_RE 9            // Output pin to enable RS485 communication
#define MAX485_DE 10           // Output pin to enable RS485 communication
#define TX_PIN 11              // TX pin for RS485 comm
#define SLAVE_ID 1             // Modbus slave ID

SoftwareSerial modbus_serial(RX_PIN, TX_PIN);
ModbusMaster node;
bool is_button_pressed = false;
unsigned long button_press_duration = 0;

enum class LiftState { UP, DOWN, INTER, UNKNOWN };

enum class MoveDirection { UP, DOWN };

enum ProjectorState { ON, OFF, UNKNOWN };

enum class State {
  INIT,
  PREP,
  IS_UP,
  IS_UP_AND_ON,
  IS_DOWN,
  IS_INTERMEDIATE,
  PRE_DOWN,
  MOVE_DOWN,
  MOVE_UP,
  STOP,
  IDLE,
  RESUME
};

void init_button();
void init_relay();
float read_current_sensor_value();
bool is_projector_on();
void init_modbus();
void modbus_pre_tx();
void modbus_post_tx();
void lower_column();
void raise_column();
void stop_column();
void disable_psu();
void enable_psu();
LiftState get_lift_state();
State run_state_machine(State state);

const unsigned long BUTTON_DEBOUNCE_DELAY = 20u;      // ms
const unsigned long BUTTON_MAX_PRESS_DURATION = 200;  // ms
const unsigned long BUTTON_REPEAT_DELAY = 1000u;      // ms
const unsigned long PROJECTOR_DEBOUNCE_DELAY = 500u;  // ms
const unsigned long LIFT_LOWER_DELAY = 15000u;        // ms
const unsigned long IDLE_TIMEOUT = 60000u;            // ms
const unsigned long RESUME_TIMEOUT = 100u;            // ms
const unsigned long MAX_MOVE_TIME = 20000u;           // ms
const auto CURRENT_SENSOR_AMP_PER_VOLT = 20.0f;       // A/V
const auto VREF = 3.3f;                               // V
const auto CURRENT_PROJECTOR_ON = 0.06f;              // A
const auto CURRENT_HYSTERESIS = 0.03f;                // A

void setup() {
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  // configure LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

  init_button();
  init_relay();
  init_modbus();
}

void loop() {
  static State current_state = State::INIT;
  current_state = run_state_machine(current_state);
  delay(1);
}

State run_state_machine(State current_state) {
  static unsigned long last_state_change_time = 0;
  static MoveDirection last_move_direction = MoveDirection::DOWN;
  State next_state = current_state;
  bool renter_state = false;

  const auto now = millis();
  const auto lift_state = get_lift_state();
  const auto projector_on = is_projector_on();
  static bool saved_button_pressed = false;
  const auto button_pressed = is_button_pressed || saved_button_pressed;
  is_button_pressed = false;

  switch (current_state) {
    case State::INIT:
      next_state = State::PREP;
      break;

    case State::PREP:
      switch (lift_state) {
        case LiftState::UP:
          next_state = State::IS_UP;
          break;
        case LiftState::DOWN:
          next_state = State::IS_DOWN;
          break;
        case LiftState::INTER:
          next_state = State::IS_INTERMEDIATE;
          break;
        case LiftState::UNKNOWN:
          next_state = State::PREP;
          break;
      }
      break;

    case State::IS_UP:
      saved_button_pressed = false;
      if (projector_on) {
        next_state = State::IS_UP_AND_ON;
      } else if (button_pressed) {
        next_state = State::MOVE_DOWN;
      } else if (lift_state == LiftState::DOWN ||
                 lift_state == LiftState::INTER) {
        next_state = State::PREP;
      } else if (now - last_state_change_time >= IDLE_TIMEOUT) {
        next_state = State::IDLE;
      }
      break;

    case State::IS_UP_AND_ON:
      saved_button_pressed = false;
      if (!projector_on) {
        next_state = State::PRE_DOWN;
      } else if (lift_state == LiftState::DOWN ||
                 lift_state == LiftState::INTER) {
        next_state = State::PREP;
      }
      break;

    case State::PRE_DOWN:
      saved_button_pressed = false;
      if (projector_on) {
        next_state = State::IS_UP_AND_ON;
      } else if (button_pressed) {
        next_state = State::MOVE_DOWN;
      } else if (lift_state == LiftState::DOWN ||
                 lift_state == LiftState::INTER) {
        next_state = State::PREP;
      }
      break;

    case State::MOVE_DOWN:
      saved_button_pressed = false;
      if (projector_on) {
        next_state = State::MOVE_UP;
      } else if (button_pressed) {
        next_state = State::STOP;
      } else if (lift_state == LiftState::DOWN) {
        next_state = State::IS_DOWN;
      } else if (now - last_state_change_time >= MAX_MOVE_TIME) {
        renter_state = true;
      }
      break;

    case State::IS_DOWN:
      saved_button_pressed = false;
      if (projector_on) {
        next_state = State::MOVE_UP;
      } else if (button_pressed) {
        next_state = State::MOVE_UP;
      } else if (lift_state == LiftState::UP ||
                 lift_state == LiftState::INTER) {
        next_state = State::PREP;
      } else if (now - last_state_change_time >= IDLE_TIMEOUT) {
        next_state = State::IDLE;
      }
      break;

    case State::IDLE:
      saved_button_pressed = button_pressed;
      if (button_pressed || projector_on) {
        next_state = State::RESUME;
      }
      break;

    case State::RESUME:
      if (now - last_state_change_time >= RESUME_TIMEOUT) {
        next_state = State::PREP;
      }
      break;

    case State::MOVE_UP:
      saved_button_pressed = false;
      if (lift_state == LiftState::UP) {
        next_state = State::IS_UP;
      } else if (button_pressed) {
        next_state = State::STOP;
      } else if (now - last_state_change_time >= MAX_MOVE_TIME) {
        renter_state = true;
      }
      break;

    case State::IS_INTERMEDIATE:
      saved_button_pressed = false;
      if (projector_on) {
        next_state = State::MOVE_UP;
      } else if (button_pressed) {
        if (last_move_direction == MoveDirection::UP) {
          next_state = State::MOVE_DOWN;
        } else {
          next_state = State::MOVE_UP;
        }
      } else if (lift_state == LiftState::UP || lift_state == LiftState::DOWN) {
        next_state = State::PREP;
      } else if (now - last_state_change_time >= IDLE_TIMEOUT) {
        next_state = State::IDLE;
      }
      break;

    case State::STOP:
      next_state = State::PREP;
      break;
  }

  if (next_state != current_state || renter_state) {
    last_state_change_time = now;

    switch (next_state) {
      case State::PREP:
        Serial.println("PREP");
        enable_psu();
        break;

      case State::IS_UP:
        Serial.println("IS_UP");
        break;

      case State::IS_UP_AND_ON:
        Serial.println("IS_UP_AND_ON");
        break;

      case State::IS_DOWN:
        Serial.println("IS_DOWN");
        break;

      case State::IS_INTERMEDIATE:
        Serial.println("IS_INTERMEDIATE");
        break;

      case State::PRE_DOWN:
        Serial.println("PRE_DOWN");
        break;

      case State::MOVE_DOWN:
        Serial.println("MOVE_DOWN");
        lower_column();
        last_move_direction = MoveDirection::DOWN;
        break;

      case State::MOVE_UP:
        Serial.println("MOVE_UP");
        raise_column();
        last_move_direction = MoveDirection::UP;
        break;

      case State::STOP:
        Serial.println("STOP");
        stop_column();
        break;

      case State::IDLE:
        Serial.println("IDLE");
        disable_psu();
        break;

      case State::RESUME:
        Serial.println("RESUME");
        enable_psu();
        break;

      default:
        Serial.println("UNKNOWN");
        break;
    }
  }

  return next_state;
}

void init_button() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // configure ISR on both, falling and rising edges
  attachInterrupt(
      digitalPinToInterrupt(BUTTON_PIN),
      []() {
        static unsigned long last_button_press_time = 0;
        static unsigned long dead_time_start = 0;
        const auto now = millis();

        bool dead_time_elapsed = dead_time_start == 0 ||
                                 now - dead_time_start >= BUTTON_REPEAT_DELAY;

        if (!is_button_pressed && dead_time_elapsed) {
          const auto button_pressed = digitalRead(BUTTON_PIN) == LOW;

          if (button_pressed) {
            last_button_press_time = now;
          } else {
            const auto press_duration = now - last_button_press_time;

            Serial.print("Button pressed for ");
            Serial.print(press_duration);
            Serial.println("ms");

            if (press_duration >= BUTTON_DEBOUNCE_DELAY &&
                press_duration <= BUTTON_MAX_PRESS_DURATION) {
              button_press_duration = press_duration;
              is_button_pressed = true;
              last_button_press_time = 0;
              dead_time_start = now;
            }
          }
        }
      },
      CHANGE);
}

void init_relay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

float read_current_sensor_value() {
  const auto SAMPLE_COUNT = 20u;
  auto peak_voltage = 0.0f;
  for (auto i = 0u; i < SAMPLE_COUNT; ++i) {
    peak_voltage += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  peak_voltage = peak_voltage / SAMPLE_COUNT;

  // change the peak voltage to the Virtual Value of voltage
  auto voltage_virt_value = peak_voltage * 0.707f;

  // The circuit is amplified by 2 times, so it is divided by 2
  voltage_virt_value = (voltage_virt_value / 1024.0f * VREF) / 2.0f;

  auto result = voltage_virt_value * CURRENT_SENSOR_AMP_PER_VOLT;

  return result;
}

bool is_projector_on() {
  // only report projector on if it has been on for at least 1 second
  static ProjectorState last_projector_state = ProjectorState::UNKNOWN;
  static unsigned long last_projector_debounce_time = 0;

  const auto now = millis();
  const auto current = read_current_sensor_value();
  const auto threshold =
      CURRENT_PROJECTOR_ON - last_projector_state ? CURRENT_HYSTERESIS : 0.0f;

  auto projector_currently_on =
      current > threshold ? ProjectorState::ON : ProjectorState::OFF;

  if (projector_currently_on != last_projector_state) {
    if (last_projector_debounce_time == 0) {
      last_projector_debounce_time = now;
    }
  } else {
    last_projector_debounce_time = 0;
  }

  if (last_projector_debounce_time != 0 &&
      now - last_projector_debounce_time >= PROJECTOR_DEBOUNCE_DELAY) {
    last_projector_state = projector_currently_on;
  }

  return last_projector_state == ProjectorState::ON;
}

void init_modbus() {
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  modbus_serial.begin(9600);

  node.begin(SLAVE_ID, modbus_serial);
  node.preTransmission(modbus_pre_tx);
  node.postTransmission(modbus_post_tx);
}

void modbus_pre_tx() {
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void modbus_post_tx() {
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

void modbus_send_command(uint16_t address, uint16_t value) {
  uint8_t result = node.writeSingleRegister(address, value);

  if (result != node.ku8MBSuccess) {
    Serial.println("Failed to send command");
  }
}

void lower_column() { modbus_send_command(0x0003, 0x0001); }
void raise_column() { modbus_send_command(0x0001, 0x0001); }
void stop_column() { modbus_send_command(0x0002, 0x0001); }

// Get the lift state
LiftState get_lift_state() {
  if (digitalRead(RELAY_PIN) == LOW) {
    return LiftState::UNKNOWN;
  }

  uint8_t result = node.readHoldingRegisters(0x0001, 2);
  if (result == node.ku8MBSuccess) {
    auto lower_limit = node.getResponseBuffer(0) & 0xff;
    auto upper_limit = node.getResponseBuffer(1) & 0xff;

    switch (lower_limit << 1 | upper_limit) {
      case 0b10:
        return LiftState::DOWN;
      case 0b11:
        return LiftState::INTER;
      case 0b01:
        return LiftState::UP;
      default:
        return LiftState::UNKNOWN;
    }
  } else {
    return LiftState::UNKNOWN;
  }
}

void disable_psu() { digitalWrite(RELAY_PIN, LOW); }
void enable_psu() { digitalWrite(RELAY_PIN, HIGH); }

ISR(WDT_vect) {
  // Watchdog Timer interrupt service routine, can be empty.
}
