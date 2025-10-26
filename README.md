The **Dual-Room Thermal Control System** is an embedded hardware project designed to autonomously regulate and balance temperature across two independent zones using feedback control. It leverages a combination of analog sensing, PWM-driven actuation, and logical decision-making to dynamically adjust airflow and heating intensity based on real-time environmental conditions. This project demonstrates the integration of hardware interfacing, control systems, and firmware design typical of modern HVAC and automation systems. It was implemented as part of **ECE4144 – Advanced Embedded Systems (Fall 2025)** using the **Adafruit Feather 32U4** (ATmega32U4 microcontroller).

---

### System Overview

The system consists of two separate thermal zones (Room 1 and Room 2), each equipped with a **temperature sensor (thermistor)** and a **servo-driven airflow damper**. The airflow dampers open or close to regulate air exchange between rooms and a central heating source. A heating element is controlled via a multi-stage system that adjusts power levels based on temperature deviation.

The control goal is to maintain both rooms within a comfortable range (e.g., **23°C ≤ T ≤ 27°C**) while minimizing temperature differences between them. The system continuously samples temperatures, computes errors, and adjusts actuators in a **closed-loop feedback architecture**.

---

### Hardware Components

1. **Microcontroller: Adafruit Feather 32U4 (ATmega32U4)**

   * 8-bit AVR microcontroller running at 16 MHz.
   * 10-bit ADC channels used for reading thermistor voltage.
   * PWM channels used for servo control.
   * UART serial communication for debugging and telemetry.

2. **Temperature Sensors: NTC Thermistors (10kΩ @ 25°C)**

   * Connected in a voltage divider configuration.

   * Provide analog voltage corresponding to temperature.

   * Converted to temperature using the **Steinhart–Hart equation**:

     [
     \frac{1}{T} = A + B \ln(R) + C (\ln(R))^3
     ]

     where:

     * ( T ) = temperature in Kelvin
     * ( R ) = thermistor resistance
     * ( A, B, C ) = Steinhart–Hart coefficients (specific to thermistor type)

   * Alternatively, for small ranges:
     [
     T(°C) ≈ \frac{1}{A + B \ln(R)} - 273.15
     ]

3. **Servo Motors: SG90 Micro Servos**

   * Controlled via PWM from the ATmega32U4.
   * Used to modulate damper position between 0° (closed) and 90° (fully open).
   * PWM duty cycle ( D ) proportional to desired angle:
     [
     D = \frac{\text{angle}}{180} \times (max_{PWM} - min_{PWM}) + min_{PWM}
     ]

4. **Heating Element**

   * Driven by a transistor or MOSFET stage.
   * Controlled digitally in **staged heating**:

     * Stage 0: OFF
     * Stage 1: Low power (PWM ≈ 40%)
     * Stage 2: Medium power (PWM ≈ 70%)
     * Stage 3: High power (PWM ≈ 100%)
   * Controlled based on both absolute temperature and rate of change (ΔT/Δt).

5. **Power Supply**

   * 5V regulated rail for servos and logic.
   * Thermistor dividers biased with 3.3V analog reference for ADC stability.

---

### Control Logic

At the core of the system is a **multi-branch feedback algorithm** implemented in firmware. The logic runs in discrete time steps (1 Hz loop rate) and evaluates all sensor inputs before commanding actuators.

1. **Temperature Reading and Conversion**

   * ADC samples each thermistor channel.
   * Resistance is computed from the divider equation:
     [
     R_{NTC} = R_{ref} \left(\frac{V_{ADC}}{V_{ref} - V_{ADC}}\right)
     ]
   * Converted to °C using calibration constants.

2. **Decision Flow (Simplified)**

   * If both temperatures are within [23°C, 27°C]: maintain current state.
   * If one room exceeds 27°C:

     * Open the corresponding servo damper linearly with temperature (e.g., 27°C → 10%, 60°C → 100%).
     * If above 60°C: fully open and trigger **alarm** flag.
   * If one room is colder (<23°C):

     * Increase heating stage if both rooms are below threshold.
     * If only one room is cold, bias airflow toward it by closing the opposite damper.
   * Safety condition: if sensor reading is stuck, out of range, or heating is on for >5 minutes without change → trigger **safe mode**.

3. **PWM Mapping Example**

   * For servo:
     [
     PWM_{duty} = 2.5% + \left(\frac{D}{180°}\right) \times 10%
     ]
   * For heating:
     [
     PWM_{heat} = f(T_{target} - T_{avg})
     ]

4. **Timers and Safety**

   * 5-minute stage dwell enforced using `millis()`.
   * Fault detection includes stuck sensors, voltage out-of-bounds, or unresponsive servos.
   * SAFE_MODE disables heating and fully opens airflow until manual reset.

---

### Firmware Architecture

* **Timer0 (Fast PWM Mode)**: Drives servo motors at ~490 Hz.
* **ADC Subsystem**: Sequentially samples temperature sensors.
* **State Machine**: Encapsulates operational stages — `IDLE`, `HEAT_LOW`, `HEAT_HIGH`, `SAFE_MODE`.
* **Interrupt Service Routine (ISR)**: Used for timing precision in ADC sampling and servo refresh.
* **UART Logging**: Reports current temperature, servo angle, and stage to Serial Monitor (115200 baud).

The firmware is written in **Arduino C/C++**, with functions for each subsystem (`SetupPWM()`, `SetupADC_GPIO()`, `updateSystemState()`, etc.) and modular logic for readability and scalability.

---

### Example of Control Loop Logic (Pseudo-Code)

```c
void loop() {
  readTemperatures();
  updateServoPositions();
  regulateHeating();
  safetyCheck();
  delay(1000); // 1 Hz loop
}

void regulateHeating() {
  if (bothHot()) setStage(0);
  else if (bothCold()) setStage(3);
  else if (oneHotOneCold()) balanceAirflow();
}
```

---

### Analytical Model

The system can be approximated using first-order thermal dynamics:

[
C \frac{dT}{dt} = \frac{1}{R_{th}} (T_{env} - T)
]

where:

* ( C ): thermal capacitance of the room (J/°C)
* ( R_{th} ): thermal resistance between room and environment (°C/W)
* ( T_{env} ): ambient temperature
* ( T ): current room temperature

The controller effectively adjusts ( R_{th} ) (via airflow) and energy input (via PWM heating) to minimize error:

[
e(t) = T_{set} - T(t)
]

A proportional response was implemented:
[
u(t) = K_p e(t)
]
where ( K_p ) determines how aggressively the system reacts to deviations.

---

### Observations and Results

* **Response Time:** System stabilizes both rooms within 2–3 minutes under moderate temperature imbalance.
* **Accuracy:** ±0.5°C average deviation from target.
* **Energy Efficiency:** Staged heating significantly reduces power waste.
* **Fault Handling:** SAFE_MODE effectively prevents runaway heating under sensor failure.

---

### Limitations and Future Work

* Servo position jitter observed due to PWM timing overlap with ADC reads.
* Sensor calibration drift over long sessions — can be corrected with averaging or lookup tables.
* Future improvement: implement PID control for smoother transitions.
* Optional expansion: integrate wireless telemetry (ESP32 or LoRa) for remote temperature monitoring.

---


