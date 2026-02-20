# Installation & Testing Guide

### 📥 1. Download Firmware
* **[Latest Stable Release](releases/latest)**: Recommended for most users.
* **[Unstable/Nightly Builds](actions/workflows/arduino-cli.yml)**: The very latest version from the code (Select a run -> scroll to Artifacts).

### 🛠 2. How to Flash
1. **Connect** your board via USB.
2. **Enter Bootloader Mode**:
   - **XIAO SAMD21**: Short the `RST` pins twice quickly.
   - **RP2040 / RP2350**: Hold the `BOOTSEL` button while plugging in the USB cable.
3. A USB drive will appear (e.g., `RPI-RP2`).
4. **Drag and drop** the `.uf2` file onto that drive.

### 🔍 3. Recommended Test Procedure (Tera Term)
> [!IMPORTANT]
> **Check the Serial Output first!**
> Before connecting external hardware, verify the system state using a Serial Monitor. 

**Using Tera Term:**
1. Flash the firmware and let the device reboot.
2. Open **Tera Term**.
3. Go to `File` -> `New Connection` -> `Serial`.
4. Select the port for your device (it often says "USB Serial Port").
5. Go to `Setup` -> `Serial Port` and set the speed to **115200**.
6. Observe the logs. If you see initialization errors, do not connect your hardware yet!

---

### ⚖️ Disclaimer / Haftungsausschluss

**DE:** Die Nutzung dieser Firmware erfolgt **auf eigene Gefahr**. Der Autor haftet nicht für jegliche Schäden an Hardware, Software oder anderen Geräten, die durch die Verwendung dieser Dateien entstehen könnten.

**EN:** Use this firmware **at your own risk**. The author is not liable for any damage to hardware, software, or other equipment resulting from the use of these files.