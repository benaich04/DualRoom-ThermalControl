#include <Arduino.h>
#include <math.h> 

#define MAX_FAULT_CYCLES 5
#define MAX_JUMP_COUNT 3
#define STUCK_THRESHOLD 0.1
#define MAX_VOLT 2.56
#define MIN_VOLT 0.0


// ====== Global Flags ======
uint8_t SAFE_MODE = 0;
uint8_t ALARM_OVER60 = 0;
uint8_t CURRENT_STAGE = 0;
unsigned long lastStageChangeMs = 0;
const unsigned long STAGE_DWELL_MS = 300000UL; // 5 minutes



void setup() {
  Serial.begin(115200);
  SetupPWM();
  SetupADC_GPIO();
  lastStageChangeMs = millis();

}

void loop() {
  updateSystemState();
  delay(1000); // 1 Hz control loop
}







//SetupPWM function: timer0 - 490Hz freq - Fast PWM - Channel A and B independent
void SetupPWM() {
  //prescaler : 64 --> 490Hz frequency 
  TCCR0B |= (1<<CS00);
  TCCR0B |= (1<<CS01);
  TCCR0B &= ~(1<<CS02);

  //Fast PWM
  TCCR0A &= ~(1<<COM0A0);
  TCCR0A |= (1<<COM0A1);
  TCCR0A &= ~(1<<COM0B0);
  TCCR0A |= (1<<COM0B1);

  //wave generation : Fast PWM & Top 0xFF
  TCCR0A |= (1<<WGM00);
  TCCR0A |= (1<<WGM01);
  TCCR0B &= ~(1<<WGM02);

  //set Duty Cycles to 0 in SetUp
  OCR0A = 0;
  OCR0B = 0;

  //set up the PWM pins as Outputs PB7 and PD0
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
}

//SetUp ADC and GPIO 
void SetupADC_GPIO() {
  //=========  ADC configuration  =============//
  //Vtemp Max = 2.12V  --> use internal 2.56V as reference for high resolution
  ADMUX |= (1<<REFS0);
  ADMUX |= (1<<REFS1);
  // !! Input channel and Gain selection will be set in a separate function
  //ADC Prescaler : 8
  ADCSRA |= (1<<ADPS0);
  ADCSRA |= (1<<ADPS1);
  ADCSRA &= ~(1<<ADPS2);
  //Adjust right : ADLAR = 0
  ADMUX &= ~(1<<ADLAR);
  //ADC Auto trigger souce : Free running
  ADCSRB &= ~(1<<ADTS0);
  ADCSRB &= ~(1<<ADTS1);
  ADCSRB &= ~(1<<ADTS2);
  ADCSRB &= ~(1<<ADTS3);
  // enable ADC + start conversion
  ADCSRA |= (1<<ADEN) | (1<<ADSC);  
  // no auto-trigger
  ADCSRA &= ~(1<<ADATE);             


  //=========  GPIO configuration  =============//
  //set up the GPIO pins as Outputs PC6 & PC7
  DDRC |= (1<<PC6) | (1<<PC7);
  //initialze the Heating System to stage 0 :  1 1      Active Low
  PORTC |= (1<<PC6) | (1<<PC7);
}
 
//setMux function to set MUX to which room to read ADC from
void setMux (uint8_t rmNum) {
 // Select channel: room1 -> ADC0, room2 -> ADC1
  if (rmNum == 1)
    ADMUX = (ADMUX & 0xF0) | 0;   // ADC0
  else
    ADMUX = (ADMUX & 0xF0) | 1;   // ADC1

}

float getTemp (uint8_t rmNum) {
  setMux(rmNum);

  // Dummy conversion
  ADCSRA |= (1 << ADSC);
  while (!(ADCSRA & (1 << ADIF)));
  (void)ADCL; (void)ADCH;
  ADCSRA |= (1 << ADIF);

  // Real conversion
  ADCSRA |= (1 << ADSC);
  while (!(ADCSRA & (1 << ADIF)));
  uint16_t adc_val = ADCW;
  ADCSRA |= (1 << ADIF);

  float Vtemp = (adc_val / 1023.0f) * 2.56f;
  float Temp  = 199.6f * Vtemp - 303.2f;
  return Temp;
}

void controlDamper(uint8_t rmNum, uint8_t val) {
  // Clamp val to 0–100 range
  if (val > 100) val = 100;

  // Scale 0–100 → 0–255
  uint8_t duty = (uint8_t)(((uint16_t)val * 255) / 100);


  // Apply to correct PWM channel
  if (rmNum == 1) 
    OCR0A = duty;
  else if (rmNum == 2) 
    OCR0B = duty;
}

void controlHeat (uint8_t stage) {

  switch (stage)
  {
  case 0:         // stage O : Off --> 11
    PORTC |= (1<<PC6); 
    PORTC |= (1<<PC7);
  break;

  case 1:         // stage 1 : Low Heat --> 01
    PORTC &= ~(1<<PC6);
    PORTC |= (1<<PC7);
  break;

  case 2:         // stage 2 : High Heat --> x0
    PORTC &= ~(1<<PC7);
  break;

  default:        //default is Off
    PORTC |= (1<<PC6); 
    PORTC |= (1<<PC7);
    break;
  }

}








void setNormalMode(void) {
  SAFE_MODE = 0;
  ALARM_OVER60 = 0;

  // Heating will be handled separately by logic (Stage 0/1/2)
  // Dampers controlled dynamically in loop()
  controlHeat(0);          // Start from Stage 0 (Idle)
  controlDamper(1, 0);     // Damper 1 closed
  controlDamper(2, 0);     // Damper 2 closed
}

// Safe mode: stop heating and open dampers halfway for ventilation
void setSafeMode(void) {
  SAFE_MODE = 1; ALARM_OVER60 = 0;
  controlHeat(0);
  controlDamper(1, 100);   // 100% (not 255)
  controlDamper(2, 100);
}

// Over-temp mode: shut off heating and fully open dampers for maximum cooling
void setOverTempMode(void) {
  SAFE_MODE = 0; ALARM_OVER60 = 1;
  controlHeat(0);
  controlDamper(1, 100);   // 100% (not 255)
  controlDamper(2, 100);
}

// Normal operation: set the heating system to the desired stage (0, 1, or 2)
void setStageMode(uint8_t stage) {

  SAFE_MODE = 0;
  ALARM_OVER60 = 0;

  // stage = 0, 1, or 2
  controlHeat(stage);
}

// Classify temperature into 3 states
uint8_t classifyTemp(float T) {
  if (T < 24.5) return 0;       // COLD
  else if (T <= 25.5) return 1; // IDEAL
  else return 2;                // HOT
}

// Map temperature to damper opening as a percentage (0–100)
uint8_t computeDamper(float T) {
  if (T <= 25.0f) return 0;          // no cooling at/below setpoint
  if (T >= 60.0f) return 100;        // full open at/above 60°C

  // Linear map: 25°C -> 0%, 60°C -> 100%
  float pct = ((T - 25.0f) / 35.0f) * 100.0f;

  // Round and clamp
  int p = (int)(pct + 0.5f);
  if (p < 0)   p = 0;
  if (p > 100) p = 100;
  return (uint8_t)p;
}

// Decide heating stage based on room states
uint8_t decideStage(uint8_t s1, uint8_t s2) {
  if (s1 == 0 && s2 == 0) return 2; // both cold -> stage 2
  else if ((s1 == 0 && s2 == 1) || (s1 == 1 && s2 == 0) || 
           (s1 == 0 && s2 == 2) || (s1 == 2 && s2 == 0))
    return 1;                       // one cold -> stage 1
  else
    return 0;                       // none cold -> stage 0
}

float getVoltage(uint8_t rmNum){
  setMux(rmNum);

  // Dummy conversion (discard result) to let MUX settle
  ADCSRA |= (1 << ADSC);                    // start conversion
  while (!(ADCSRA & (1 << ADIF)));          // wait done
  (void)ADCL; (void)ADCH;                   // read & discard
  ADCSRA |= (1 << ADIF);  

  // Real conversion
  ADCSRA |= (1 << ADSC);                    // start conversion
  while (!(ADCSRA & (1 << ADIF)));          // wait done
  uint16_t adc_val = ADCW;                  // 10-bit result
  ADCSRA |= (1 << ADIF); 

  float Vtemp = (adc_val / 1023.0f) * 2.56f;

  return Vtemp;
}

// Check for sensor faults (simplified placeholder)
bool sensorFaultDetected(float T1, float T2) {
  // ---- 1) Voltage sanity check ----
  // Read raw voltages for each sensor and verify within expected ADC range
  float V1 = getVoltage(1);
  float V2 = getVoltage(2);
  if (V1 < MIN_VOLT || V1 > MAX_VOLT || V2 < MIN_VOLT || V2 > MAX_VOLT) {
    Serial.println("FAULT: ADC voltage out of range");
    return true;
  }

  // ---- 2) Temperature plausibility gate ----
  // Reject any readings outside a wide physical range
  if (T1 < -20.0f || T1 > 90.0f || T2 < -20.0f || T2 > 90.0f) {
    Serial.println("FAULT: Temperature out of plausible range");
    return true;
  }

  // ---- 3) Stateful trackers for stuck / jump detection ----
  static float   prevT1 = NAN, prevT2 = NAN;
  static uint8_t stuckCount1 = 0, stuckCount2 = 0;  // repeated tiny change
  static uint8_t jumpCount1  = 0,  jumpCount2  = 0; // repeated big jump

  // Compute per-room delta since last reading (skip first sample)
  float dT1 = (!isnan(prevT1)) ? fabsf(T1 - prevT1) : 0.0f;
  float dT2 = (!isnan(prevT2)) ? fabsf(T2 - prevT2) : 0.0f;

  // ---- 4) STUCK detection ----
  // If change ≤ STUCK_THRESHOLD for many cycles, mark sensor as stuck
  if (!isnan(prevT1) && dT1 <= STUCK_THRESHOLD) stuckCount1++; else stuckCount1 = 0;
  if (!isnan(prevT2) && dT2 <= STUCK_THRESHOLD) stuckCount2++; else stuckCount2 = 0;

  // ---- 5) JUMP detection ----
  // If change > 5 °C repeatedly, suspect noise or wiring fault
  if (!isnan(prevT1) && dT1 > 5.0f) jumpCount1++; else jumpCount1 = 0;
  if (!isnan(prevT2) && dT2 > 5.0f) jumpCount2++; else jumpCount2 = 0;

  // ---- 6) Save current temps for next iteration ----
  prevT1 = T1;
  prevT2 = T2;

  // ---- 7) Decide fault based on consecutive counts ----
  if (stuckCount1 >= MAX_FAULT_CYCLES || stuckCount2 >= MAX_FAULT_CYCLES) {
    Serial.println("FAULT: Sensor stuck");
    return true;
  }
  if (jumpCount1 >= MAX_JUMP_COUNT || jumpCount2 >= MAX_JUMP_COUNT) {
    Serial.println("FAULT: Sudden temperature jump");
    return true;
  }

  // ---- 8) If none triggered, readings look valid ----
  return false;
}

// Apply system outputs to hardware
void applySystemState(uint8_t stage, uint8_t D1, uint8_t D2) {
  controlHeat(stage);
  controlDamper(1, D1);
  controlDamper(2, D2);
}

//Debug print
void logStatus(float T1, float T2, uint8_t stage) {
  Serial.print("T1="); Serial.print(T1);
  Serial.print("  T2="); Serial.print(T2);
  Serial.print("  Stage="); Serial.println(stage);
}

void applyStageWithDwell(uint8_t desired) {
  unsigned long now = millis();
  if (desired != CURRENT_STAGE) {
    if (now - lastStageChangeMs >= STAGE_DWELL_MS) {
      CURRENT_STAGE = desired;
      lastStageChangeMs = now;
      controlHeat(CURRENT_STAGE);
    } else {
      // keep current stage until dwell expires
      controlHeat(CURRENT_STAGE);
    }
  } else {
    controlHeat(CURRENT_STAGE);
  }
}

void updateSystemState(void) {
  float T1 = getTemp(1);
  float T2 = getTemp(2);

  // 1) Fault first
  if (sensorFaultDetected(T1, T2)) {
    setSafeMode();
    logStatus(T1, T2, CURRENT_STAGE);
    return;
  }

  // 2) Over-temp
  if (T1 >= 60.0 || T2 >= 60.0) {
    setOverTempMode();
    logStatus(T1, T2, CURRENT_STAGE);
    return;
  }

  // 3) Classify
  uint8_t s1 = classifyTemp(T1);
  uint8_t s2 = classifyTemp(T2);

  // 4) Decide desired stage
  uint8_t desiredStage = decideStage(s1, s2);

  // 5) Compute dampers (cooling per room)
  uint8_t D1 = computeDamper(T1);
  uint8_t D2 = computeDamper(T2);

  // 6) Apply with dwell for stage; dampers immediately
  applyStageWithDwell(desiredStage);
  controlDamper(1, D1);
  controlDamper(2, D2);

  // 7) Log
  logStatus(T1, T2, CURRENT_STAGE);
}
