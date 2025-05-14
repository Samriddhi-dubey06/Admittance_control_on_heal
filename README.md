# Admittance Control on 6-DOF Heal Robot

This repository contains the implementation of **Admittance Control** on a 6-DOF Heal robot. A **biaxial FUTEK force-torque (FT) sensor** is mounted on the robot's end-effector to measure forces in the X and Y directions. These measurements are used to control the robot's end-effector motion using admittance control strategies.

## 🛠️ Hardware Setup

- **Robot**: Custom-built 6-DOF Heal robot
- **FT Sensor**: Biaxial FUTEK sensor (X and Y axes)
- **Mounting**: FT sensor is mounted on the end-effector

---

## 📂 Repository Structure

### 🔁 Admittance Control Scripts

- **`Admittance in x-y.py`**  
  Applies full **2D admittance control** using the FT sensor's X and Y force readings. The end-effector responds dynamically in both directions based on the admittance control model.

- **`Admittance in y.py`**  
  Applies **1D admittance control** only in the **Y-direction**. The X-direction remains passive.

---

### 🔄 Force-to-Velocity Mapping Scripts

- **`Cart F to V.py`**  
  Maps the force detected by the FT sensor in **both X and Y axes** to corresponding Cartesian velocities. No control is applied—just direct mapping.

- **`cart F to V in x.py`**  
  Maps the force detected by the FT sensor only in the **X-direction** to velocity in the X-direction.

---

### 📊 FT Sensor Data Scripts

- **`x ft sensor.py`**  
  Reads real-time force values from the **X and Y FT sensors** placed along the X-direction. Helpful for sensor calibration and debugging.

---

## 🚀 How to Use

1. Ensure the FT sensor is calibrated and mounted on the Heal robot's end-effector.
2. Run any of the admittance control scripts (`Admittance in x-y.py` or `Admittance in y.py`) depending on your control objective.
3. For testing raw force-to-velocity mapping, use the `Cart F to V.py` or `cart F to V in x.py` scripts.
4. Use `x ft sensor.py` to observe live force data.

---

## 📌 Notes

- The admittance control parameters (mass, damping, etc.) can be tuned in the script files.
- Make sure communication between the FT sensor and the control PC is established before running the scripts.

---

## 📷 Demo (Optional)

You can include videos or images here if you'd like to show the robot in action.

---

## 📬 Contact

For any questions or contributions, feel free to open an issue or contact me.

Hari Bol 🙏
