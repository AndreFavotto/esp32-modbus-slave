#include <Arduino.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

#define TANK_LEVEL_MIN_THRESHOLD 100
#define TANK_LEVEL_MAX_THRESHOLD 3800

/* Modbus pre-processing settings */

#define SLAVE_ID 1

//Coil, Discrete Input, Input Register, Holding Register
// #define COIL_BASE_ADDRESS       1
// #define ALARM_COIL        (1 - COIL_BASE_ADDRESS)         //Digital output - built-in LED

// #define I_STATUS_BASE_ADDRESS   10001
// #define LEVEL_SWITCH_I_COIL     (10001 - I_STATUS_BASE_ADDRESS) //Digital input

// #define I_REG_BASE_ADDRESS      30001
// #define TANK_LEVEL_I_REG        (30001 - I_REG_BASE_ADDRESS)    //Analog input (read only)
// #define DIAGNOSTICS_I_REG       (30002 - I_REG_BASE_ADDRESS)    //Analog input (read only)

#define HREG_BASE_ADDRESS       40001
#define ALARM_COIL              (40001 - HREG_BASE_ADDRESS)      //Digital output
#define LEVEL_SWITCH_I_COIL     (40002 - HREG_BASE_ADDRESS)      //Digital input
#define TANK_LEVEL_I_REG        (40003 - HREG_BASE_ADDRESS)      //Analog input (read only)
#define VALVE_SIGNAL_HREG       (40004 - HREG_BASE_ADDRESS)      //Analog output
#define DIAGNOSTICS_I_REG       (40005 - HREG_BASE_ADDRESS)      //Analog input (read only)

/* Pinout */
#define LEVEL_SWITCH_PIN  34
#define TANK_LEVEL_PIN 35
#define ALARM_BUZZER 32
#define VALVE_SIGNAL_LEDBUILTIN 2

/* Global variables*/
  
ModbusIP mb;
bool alarm_status;      /* false for off and true for on */
bool level_switch;      /* false for closed and true for open */
uint16_t tank_level;
uint16_t valve_output;  /* 0 to 255 */
uint16_t diagnostics;   /* 1 if potentiometer is disconnected */
size_t timestamp;

/* Callback functions */

static uint16_t cb_alarm_buzzer_coil(TRegister* reg, uint16_t val)
{
  alarm_status = (val != 0);
  digitalWrite(ALARM_BUZZER, alarm_status);
  Serial.println("Alarm status changed");
  return val;
}

static uint16_t cb_valve_signal(TRegister* reg, uint16_t val)
{
  //scale val from 0-65535 to 0-255
  valve_output = map(val, 0, 65535, 0, 255);
  analogWrite(VALVE_SIGNAL_LEDBUILTIN, valve_output);
  return val;
}

void setup() {
  /* Connect to network*/
  Serial.begin(9600);
  WiFi.begin("LinkBox", "industria50");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
   /* Set initial values */

  tank_level = 0;
  valve_output = 0;
  alarm_status = false;
  level_switch = false;
  diagnostics = 0;

  pinMode(ALARM_BUZZER, OUTPUT);
  pinMode(VALVE_SIGNAL_LEDBUILTIN, OUTPUT);
  pinMode(LEVEL_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(TANK_LEVEL_PIN, INPUT);

  // /* Add coil status */
  // mb.addCoil(ALARM_COIL);
  // mb.Coil(ALARM_COIL, alarm_status);
  // mb.onSetCoil(ALARM_COIL, cb_alarm_buzzer_coil, 1);

  // /* Add input status */
  // mb.addIsts(LEVEL_SWITCH_I_COIL);
  // mb.Ists(LEVEL_SWITCH_I_COIL, level_switch);

  // /* Add input registers */
  // mb.addIreg(TANK_LEVEL_I_REG);
  // mb.Ireg(TANK_LEVEL_I_REG, tank_level);

  // mb.addIreg(DIAGNOSTICS_I_REG);
  // mb.Ireg(DIAGNOSTICS_I_REG, diagnostics);

  /* Add holding register */

  mb.addHreg(ALARM_COIL);
  mb.Hreg(ALARM_COIL, alarm_status);
  mb.onSetHreg(ALARM_COIL  , cb_alarm_buzzer_coil, 1);

  mb.addHreg(LEVEL_SWITCH_I_COIL);
  mb.Hreg(LEVEL_SWITCH_I_COIL  , level_switch);

  mb.addHreg(TANK_LEVEL_I_REG);
  mb.Hreg(TANK_LEVEL_I_REG  , tank_level);

  mb.addHreg(VALVE_SIGNAL_HREG);
  mb.Hreg(VALVE_SIGNAL_HREG  , valve_output);
  mb.onSetHreg(VALVE_SIGNAL_HREG  , cb_valve_signal, 1);

  mb.addHreg(DIAGNOSTICS_I_REG);
  mb.Hreg(DIAGNOSTICS_I_REG  , diagnostics);
  
  //initialize modbus connection
  mb.server();
}


void loop() {

  if (millis() - timestamp > 200)
    {
      timestamp = millis();

      tank_level = analogRead(TANK_LEVEL_PIN);
      mb.Hreg(TANK_LEVEL_I_REG, tank_level);

      level_switch = digitalRead(LEVEL_SWITCH_PIN);
      mb.Hreg(LEVEL_SWITCH_I_COIL, level_switch);

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

      mb.Hreg(DIAGNOSTICS_I_REG, diagnostics);
    }

  mb.task();
  yield();
}