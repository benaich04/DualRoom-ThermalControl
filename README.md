Here’s a **complete professional-level README.md** for your project, written in a format ready for GitHub.
It’s structured like a polished engineering documentation file — clear, descriptive, and detailed enough for technical reviewers, recruiters, or professors.

---

````markdown
# 🌡️ Dual-Room Thermal Control System — Embedded Temperature Regulation

## Overview

This project implements a **dual-room closed-loop temperature regulation system** on an **Adafruit Feather 32U4 (ATmega32U4)** microcontroller platform.  
The goal is to **maintain balanced and stable temperature conditions across two zones** using a combination of:

- **Analog temperature sensing** (via thermistor-based voltage dividers + ADC conversion)  
- **PWM-controlled servo actuators** (to modulate airflow dampers)  
- **Staged heating control logic** (to engage heating elements in a proportional manner)  
- **Feedback control algorithms** (to dynamically regulate target temperature zones)

This system was designed and implemented for **ECE4144 – Advanced Embedded Systems (Fall 2025)** at **New York University Abu Dhabi**, demonstrating advanced embedded control techniques in a practical multi-sensor environment.

---

## 🧩 System Architecture

The system is composed of **two independent zones** (`Room A` and `Room B`), each with its own thermistor temperature sensor and servo-controlled airflow damper.  
Both zones share a **common heating system** that can be enabled or disabled in multiple stages, depending on the global temperature conditions.

### 🔧 Hardware Components

| Component | Quantity | Description |
|------------|-----------|-------------|
| **Adafruit Feather 32U4** | 1 | ATmega32U4-based microcontroller, 8-bit, 16 MHz, 10-bit ADC, PWM capable. |
| **10kΩ NTC Thermistors** | 2 | Temperature sensors configured as voltage dividers with fixed pull-up resistors. |
| **SG90 Servo Motors** | 2 | Mini servos for damper control, driven via PWM output pins. |
| **Heating Elements (Resistive Load)** | 2 | Represent the staged heating system (Stage 1 and Stage 2). |
| **MCP6004 Op-Amp** | 1 | Used for buffering and optional signal conditioning of thermistor outputs. |
| **Power Supply (5V DC)** | 1 | Provides stable voltage for Feather and peripheral components. |
| **Assorted Components** | – | Includes resistors, wiring, perfboard/PCB, and connectors. |

---

## ⚙️ Functional Description

The system continuously monitors both room temperatures, then determines control actions based on the following **logic hierarchy**:

1. **Temperature Sensing**  
   Each thermistor outputs a voltage corresponding to its resistance, which is converted to temperature using:
   \[
   T = \frac{1}{\frac{1}{T_0} + \frac{1}{\beta} \ln\left(\frac{R_T}{R_0}\right)} - 273.15
   \]
   where:  
   - \( T_0 = 298.15\,K \) (25 °C reference)  
   - \( R_0 = 10\,k\Omega \) (nominal thermistor resistance)  
   - \( \beta \approx 3950\,K \) (material constant)  
   - \( R_T \) is measured via ADC conversion:  
     \[
     R_T = R_{ref} \left(\frac{V_{ADC}}{V_{ref} - V_{ADC}}\right)
     \]

2. **Airflow Control (Servo PWM)**  
   Each servo’s position corresponds to the airflow damper opening:
   \[
   D = \begin{cases}
   0\% & \text{if } T < 27°C \\
   100\% & \text{if } T \geq 60°C \\
   \text{Linear from 27°C → 60°C} & \text{otherwise}
   \end{cases}
   \]
   Servo position is driven by PWM on Timer0 or Timer1 using the Feather’s `analogWrite()` equivalent registers.

3. **Heating Control (Staged)**  
   The heating subsystem activates according to aggregate temperature deviation:
   - **Stage 0:** All off (both rooms within comfort range)
   - **Stage 1:** Partial heating if average temperature < 23 °C
   - **Stage 2:** Full heating if both < 20 °C or if large differential (> 10 °C) persists

   Each stage is output through a dedicated GPIO driving a MOSFET or relay.

4. **Fault Detection and Safety**  
   The system includes:
   - **Stuck-sensor detection** (minimal change over time → fault flag)
   - **Overheat alarm (> 60 °C)** (disables heating, opens airflow fully)
   - **Safe mode entry** after repeated fault cycles

---

## 🔬 Control Algorithm (High-Level Pseudocode)

```c
loop {
    read temp1, temp2;
    compute avg_temp = (temp1 + temp2) / 2;

    if (temp1 > 60 || temp2 > 60) {
        enterSafeMode();
        openAllServos();
        disableHeating();
    }

    else if (avg_temp < 23)
        enableHeating(Stage1);
    else if (avg_temp < 20)
        enableHeating(Stage2);
    else
        disableHeating();

    // Servo airflow control
    D1 = map(temp1, 27, 60, 0, 100);
    D2 = map(temp2, 27, 60, 0, 100);
    setServoDuty(D1, servo1);
    setServoDuty(D2, servo2);
    
    delay(1000);
}
````

---

## 🧠 System Behavior Summary

| Condition         | Room A Temp | Room B Temp | Servo A                | Servo B                | Heating Stage | Status            |
| ----------------- | ----------- | ----------- | ---------------------- | ---------------------- | ------------- | ----------------- |
| Both Ideal        | 23–27 °C    | 23–27 °C    | Closed                 | Closed                 | 0             | Idle              |
| One Hot, One Cold | >27 / <23   |             | Adjusted Independently | Adjusted Independently | 1             | Partial Heating   |
| Both Hot          | >27         | >27         | Fully Open             | Fully Open             | 0             | Cooling / Passive |
| Both Cold         | <23         | <23         | Closed                 | Closed                 | 2             | Max Heating       |
| Fault Detected    | —           | —           | —                      | —                      | Off           | Safe Mode         |

---

## 📈 Equations and Control Parameters

* **ADC Conversion:**
  [
  V_{ADC} = \frac{ADC_{count}}{1023} \times V_{ref}
  ]

* **Servo PWM Signal:**
  [
  D_{servo} = 1,\text{ms} + \frac{(D_{angle})}{180} \times 1,\text{ms}
  ]
  (Pulse width between 1–2 ms over 20 ms period)

* **Temperature Control Law (Proportional Simplification):**
  [
  u(t) = K_p \cdot (T_{set} - T_{measured})
  ]
  where ( K_p ) is tuned empirically for stable response without oscillations.

---

## 🧰 Software & Implementation

* **Language:** C/C++ (Arduino Framework)
* **MCU Platform:** Adafruit Feather 32U4
* **IDE:** Arduino IDE / PlatformIO
* **Simulation Tools:** Tinkercad / Proteus (optional testing)
* **Libraries Used:**

  * `Servo.h` — for PWM control
  * `math.h` — for logarithmic thermistor equation
  * `Adafruit_Sensor.h` — for optional sensor abstraction

---

## 🧪 Calibration & Testing

1. **Thermistor Calibration:**
   Performed with a reference thermometer and known resistances to adjust β value.

2. **PWM Calibration:**
   Servo motion range mapped to [1 ms, 2 ms] pulse width with linear scaling.

3. **Heating Threshold Tuning:**
   Verified via simulation that switching thresholds minimize overshoot and oscillation.

4. **Safety Validation:**
   Over-temperature triggers verified via serial debug output and LED indicator.

---

## ⚙️ Electrical Diagram (Conceptual)

```
   [Room A Thermistor] --> [ADC0] \
                                 --> [ATmega32U4] --> [Servo A PWM]
   [Room B Thermistor] --> [ADC1] /                  [Servo B PWM]
                                         |
                                         +--> [GPIO → Heating Stage 1]
                                         +--> [GPIO → Heating Stage 2]
                                         +--> [Buzzer / LED Alarm]
```

---

## 🧩 Design Highlights

* Dual independent feedback loops (one per room)
* Integrated safety interlocks and saturation limits
* Modular, extensible codebase for scaling to 3+ zones
* Hardware-verified PWM and ADC timing on Feather 32U4
* Fully open-source embedded control implementation

---

## 🧭 Future Improvements

* Replace binary heating stages with **PID-controlled PWM heating** for finer regulation
* Integrate **I²C-based digital temperature sensors (e.g., TMP102)** for higher accuracy
* Add **OLED display** for real-time zone temperature visualization
* Implement **Bluetooth telemetry** to stream data to a mobile dashboard
* Expand system to **multi-room (3–4 zones)** with load balancing

---

## ⚠️ Issues Encountered

* Nonlinear ADC readings due to wiring resistance → mitigated with calibration curve
* Servo jitter from shared 5 V supply → resolved with decoupling capacitors
* Temperature drift during ambient changes → corrected by adding moving average filter
* Occasional “stuck” readings due to ADC timing mismatch

---

## 📄 References

* Adafruit Feather 32U4 Datasheet
* NTC Thermistor Characteristics — Vishay BC Components
* Arduino Servo PWM Reference
* Embedded Systems Control Theory — Franklin, Powell, and Emami-Naeini

---

## 👨‍💻 Author

**Mohamed Benaich**
Electrical Engineering, NYU Abu Dhabi
📧 **[mb9194@nyu.edu](mailto:mb9194@nyu.edu)**
🔗 [GitHub: benaich04](https://github.com/benaich04)

---

## 🧾 License

MIT License © 2025 Mohamed Benaich
This project is open-source and intended for academic and educational use.

```

---

Would you like me to include a **block diagram figure (drawn in ASCII or SVG-style)** to visualize the feedback system in your README (it looks great on GitHub)?
```
