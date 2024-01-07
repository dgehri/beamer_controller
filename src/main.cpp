#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// Pin definitions
#define BUTTON_PIN 2          // Input pin for the button
#define RELAY_PIN 3           // Output pin for the relay
#define CURRENT_SENSOR_PIN A2 // Input pin for the current sensor (0.2 - 2.8V)
#define MAX485_DE_RE 4        // Output pin to enable RS485 communication
#define RX_PIN 5
#define TX_PIN 6
#define SLAVE_ID 1 // Modbus slave ID

SoftwareSerial modbus_serial(RX_PIN, TX_PIN);
ModbusMaster node;
bool is_button_pressed = false;

enum class LiftState { RAISED, LOWERED, INTERMEDIATE, UNKNOWN };
enum class MoveDirection { UP, DOWN };
enum class State {
  INIT,
  IS_UP,
  IS_DOWN,
  IS_INTERMEDIATE,
  PRE_DOWN,
  MOVE_DOWN,
  MOVE_UP,
  STOP
};

void init_button();
void init_relay();
float read_current_sensor_value();
bool is_projector_on();
void init_modbus();
void modbus_pre_tx();
void modbus_post_tx();
void lift_lower();
void lift_raise();
void lift_stop();
LiftState get_lift_state();
State run_state_machine(State state);

const unsigned long BUTTON_DEBOUNCE_DELAY = 4u;  // ms
const unsigned long BUTTON_REPEAT_DELAY = 1000u; // ms

const auto CURRENT_SENSOR_AMP_PER_VOLT = 20.0f;      // A/V
const auto VREF = 3.3f;                              // V
const auto CURRENT_PROJECTOR_ON = 0.09f;             // A
const auto CURRENT_HYSTERESIS = 0.10f;               // A
const unsigned long PROJECTOR_DEBOUNCE_DELAY = 500u; // ms
volatile unsigned long last_projector_debounce_time = 0u;

const unsigned long LIFT_LOWER_DELAY = 15000u; // ms
volatile unsigned long lift_lower_start_time = 0u;

void setup() {
  Serial.begin(9600);

  // configure LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

  init_button();
  init_relay();
  init_modbus();
}

void loop() {
  // Check the flag set by the ISR
  if (is_button_pressed) {
    is_button_pressed = false;
    // Toggle the LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    if (digitalRead(LED_BUILTIN) == HIGH) {
      Serial.println("Button pressed");
      // lift_lower();
    } else {
      Serial.println("Button released");
      // lift_raise();
    }
  }

  if (is_projector_on()) {
    lift_lower_start_time = 0;

    if (digitalRead(RELAY_PIN) == LOW) {
      Serial.println("Raising lift");
      digitalWrite(RELAY_PIN, HIGH);
    }
  } else {
    if (lift_lower_start_time == 0) {
      Serial.println("Starting delay to lower lift");
      lift_lower_start_time = millis();
    } else if (millis() - lift_lower_start_time >= LIFT_LOWER_DELAY) {
      if (digitalRead(RELAY_PIN) == HIGH) {
        Serial.println("Lowering lift");
        digitalWrite(RELAY_PIN, LOW);
      }
    }
  }

  static LiftState last_lift_state = LiftState::UNKNOWN;
  auto lift_state = get_lift_state();

  if (lift_state != last_lift_state) {
    switch (lift_state) {
    case LiftState::RAISED:
      Serial.println("Lift is raised");
      break;
    case LiftState::LOWERED:
      Serial.println("Lift is lowered");
      break;
    case LiftState::INTERMEDIATE:
      Serial.println("Lift is in intermediate position");
      break;
    case LiftState::UNKNOWN:
      Serial.println("Lift state is unknown");
      break;
    }
    last_lift_state = lift_state;
  }
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

            if (press_duration >= BUTTON_DEBOUNCE_DELAY) {
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

  if (digitalRead(RELAY_PIN) == HIGH) {
    result -= CURRENT_HYSTERESIS;
  }

  return result;
}

bool is_projector_on() {
  // only report projector on if it has been on for at least 1 second
  static bool last_projector_state = false;
  static unsigned long last_projector_debounce_time = 0;

  const auto now = millis();
  auto current = read_current_sensor_value();

  bool projector_currently_on = current > CURRENT_PROJECTOR_ON;
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

  return last_projector_state;
}

void init_modbus() {
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, 0);

  modbus_serial.begin(9600);

  node.begin(SLAVE_ID, modbus_serial);
  node.preTransmission(modbus_pre_tx);
  node.postTransmission(modbus_post_tx);
}

void modbus_pre_tx() { digitalWrite(MAX485_DE_RE, 1); }

void modbus_post_tx() { digitalWrite(MAX485_DE_RE, 0); }

void sendModbusCommand(uint16_t address, uint16_t value) {
  uint8_t result = node.writeSingleRegister(address, value);

  if (result == node.ku8MBSuccess) {
    Serial.println("Command sent successfully");
  } else {
    Serial.println("Failed to send command");
  }
}

void lift_lower() { sendModbusCommand(0x0003, 0x0001); }
void lift_raise() { sendModbusCommand(0x0001, 0x0001); }
void lift_stop() { sendModbusCommand(0x0002, 0x0001); }

// Get the lift state
LiftState get_lift_state() {
  uint8_t result = node.readHoldingRegisters(0x0001, 2);
  if (result == node.ku8MBSuccess) {
    auto lower_limit = node.getResponseBuffer(0) & 0xff;
    auto upper_limit = node.getResponseBuffer(1) & 0xff;

    switch (lower_limit << 1 | upper_limit) {
    case 0b10:
      return LiftState::LOWERED;
    case 0b11:
      return LiftState::INTERMEDIATE;
    case 0b01:
      return LiftState::RAISED;
    default:
      return LiftState::UNKNOWN;
    }
  } else {
    return LiftState::UNKNOWN;
  }
}
