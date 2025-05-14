![FT sesor mounted on heal](https://github.com/user-attachments/assets/693c04f0-8d24-4558-ac6e-fdd1a22ed443)# 🤖 Admittance Control on 6-DOF Heal Robot

This repository contains the implementation of **Admittance Control** on a custom 6-DOF Heal robot. A **biaxial FUTEK force-torque (FT) sensor** is mounted on the robot's end-effector to measure forces in the X and Y directions. These readings are used to command motion through an admittance control strategy in Cartesian space.

---

## 📌 System Dynamics

The fundamental dynamic equation used for implementing admittance control is:

$$
M \Delta\ddot{x} + K_d \Delta\dot{x} + h_e = h
$$

Where:
- \( M \) = Mass Matrix  
- \( K_d \) = Damping Matrix  
- \( h_e \) = Wrench (force/torque) at the end-effector  
- \( h \) = Desired/commanded wrench  

---

### 👉 Quasi-Static Approximation

Under quasi-static conditions, we neglect inertial effects:

$$
K_d \Delta\dot{x} + h_e = h  
\Rightarrow K_d \Delta\dot{x} + (h_e - h) = 0
$$

Multiplying both sides by \( K_d^{-1} \):

$$
\Delta\dot{x} + K_d^{-1}(h_e - h) = 0
$$

Rewriting the control law for the reference velocity:

$$
\dot{x_r} = \dot{x_d} + K_d^{-1}(h_e - h)
$$

And using the Jacobian \( J \) such that \( \dot{x} = J\dot{q} \), we map Cartesian velocity to joint velocity:

$$
\dot{q_r} = \dot{q_d} + J^{-1}[K_d^{-1}(h_e - h)]
$$

---

## 🛠️ Hardware Setup

- **Robot**: Custom 6-DOF Heal robot
- **Sensor**: Biaxial **FUTEK FT sensor** (for X and Y axes)
- **Mounting**: Sensor mounted on the end-effector

---

## 📂 Repository Structure

### 🔁 Admittance Control

- **`Admittance in x-y.py`**  
  Full **2D admittance control** using both X and Y forces.

- **`Admittance in y.py`**  
  **1D admittance control** in **Y-axis** only.

### 🔄 Force-to-Velocity Mapping

- **`Cart F to V.py`**  
  Maps X and Y forces to Cartesian velocities (no control loop).

- **`cart F to V in x.py`**  
  Maps only X force to X-direction velocity.

### 📊 FT Sensor Data

- **`x ft sensor.py`**  
  Reads and prints real-time force values in both X and Y axes.

---

## 📷 Demonstration

### 🎥 Video

https://user-images.githubusercontent.com/your-username/your-demo-video.mp4  
*(Replace this link with your uploaded video link)*

### 🖼️ Image

![Heal Robot with FT Sensor](https://user-images.githubusercontent.com/your-username/your-image.jpg)  
*(Replace this with the link to your uploaded image)*

---

## 🚀 How to Use

1. Mount the FT sensor on the robot's end-effector.
2. Calibrate and ensure the sensor is communicating correctly.
3. Run any of the control scripts depending on your desired behavior:
   - Use `Admittance in x-y.py` for full 2D admittance control.
   - Use `Cart F to V.py` or `x ft sensor.py` for testing and debugging force mappings.
4. Tune damping values inside the scripts for smooth control.

---

## 📬 Contact

For any questions or contributions, feel free to open an issue or reach out.

Hari Bol 🙏
