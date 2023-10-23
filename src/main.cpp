#include <Arduino.h>
#include <ModbusRTU.h>

/* Global constants */

#define TANK_LEVEL_MIN_THRESHOLD 100
#define TANK_LEVEL_MAX_THRESHOLD 3800

/* Modbus pre-processing settings */

#define SLAVE_ID 1
//Coil, Discrete Input, Input Register, Holding Register
#define COIL_BASE_ADDRESS       1
#define BUILTIN_LED_COIL        (1 - COIL_BASE_ADDRESS)         //Digital output - built-in LED

#define I_STATUS_BASE_ADDRESS   10001
#define LEVEL_SWITCH_I_COIL     (10001 - I_STATUS_BASE_ADDRESS) //Digital input

#define I_REG_BASE_ADDRESS      30001
#define TANK_LEVEL_I_REG        (30001 - I_REG_BASE_ADDRESS)    //Analog input (read only)
#define DIAGNOSTICS_I_REG       (30002 - I_REG_BASE_ADDRESS)    //Analog input (read only)

#define HREG_BASE_ADDRESS       40001
#define VALVE_SIGNAL_HREG       (40001 - HREG_BASE_ADDRESS)     //Analog output

/* Pinout */
#define LEVEL_SWITCH_PIN  15
#define TANK_LEVEL_PIN 4
#define LED_BUILTIN 2

/* Global variables*/
  
ModbusRTU mb;
bool led_status;   /* false for off and true for on */
bool level_switch; /* false for closed and true for open */
uint16_t tank_level;
uint16_t led_brightness; /* 0 to 255 */
uint16_t diagnostics;   /* 1 if potentiometer is disconnected */
size_t timestamp;

/* Callback functions */

static uint16_t cb_led_builtin_coil(TRegister* reg, uint16_t val)
{
  led_status = (val != 0);
  digitalWrite(LED_BUILTIN, led_status);
  return val;
}

static uint16_t cb_valve_signal(TRegister* reg, uint16_t val)
{
  //scale val from 0-65535 to 0-255
  led_brightness = map(val, 0, 65535, 0, 255);
  analogWrite(LED_BUILTIN, led_brightness);
  return val;
}

void setup() {
   /* Set initial values */

  tank_level = 0;
  led_brightness = 0;
  led_status = false;
  level_switch = false;
  diagnostics = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEVEL_SWITCH_PIN, INPUT);
  pinMode(TANK_LEVEL_PIN, INPUT);

  Serial.begin(9600, SERIAL_8N1);
  mb.begin(&Serial);
  //mb.begin(&Serial, RXTX_PIN);  //or use RX/TX direction control pin (if required)
  mb.slave(SLAVE_ID);

  /* Add coil status */
  mb.addCoil(BUILTIN_LED_COIL);
  mb.Coil(BUILTIN_LED_COIL, led_status);
  mb.onSetCoil(BUILTIN_LED_COIL, cb_led_builtin_coil, 1);

  /* Add input status */
  mb.addIsts(LEVEL_SWITCH_I_COIL);
  mb.Ists(LEVEL_SWITCH_I_COIL, level_switch);

  /* Add input registers */
  mb.addIreg(TANK_LEVEL_I_REG);
  mb.Ireg(TANK_LEVEL_I_REG, tank_level);

  mb.addIreg(DIAGNOSTICS_I_REG);
  mb.Ireg(DIAGNOSTICS_I_REG, diagnostics);

  /* Add holding register */

  mb.addHreg(VALVE_SIGNAL_HREG);
  mb.Hreg(VALVE_SIGNAL_HREG  , led_brightness);
  mb.onSetHreg(VALVE_SIGNAL_HREG  , cb_valve_signal, 1);

}


void loop() {

  if (millis() - timestamp > 200)
    {
      timestamp = millis();

      tank_level = analogRead(TANK_LEVEL_PIN);
      mb.Ireg(TANK_LEVEL_I_REG, tank_level);

      level_switch = digitalRead(LEVEL_SWITCH_PIN);
      mb.Ists(LEVEL_SWITCH_I_COIL, level_switch);

      /* Diagnostics value update */

      if (tank_level < TANK_LEVEL_MIN_THRESHOLD)
        {
          diagnostics = 1;
        }
      
      else if (tank_level > TANK_LEVEL_MAX_THRESHOLD)
        {
          diagnostics = 2;
        }
      else
        {
          diagnostics = 0;
        }

      mb.Ireg(DIAGNOSTICS_I_REG, diagnostics);
    }

  mb.task();
  yield();
}