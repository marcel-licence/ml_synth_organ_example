# Installation & Testing Guide

### 📥 1. Download Firmware
Download the `.uf2` file corresponding to your hardware from the **[Latest Release](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME/releases/latest)**.

### 🛠 2. How to Flash
1. **Connect** your board to your computer via USB.
2. **Enter Bootloader Mode**:
   - **XIAO SAMD21**: Quickly short the `RST` pins twice with tweezers or a jumper.
   - **RP2040 / RP2350**: Hold the `BOOTSEL` button while plugging in the USB cable.
3. A new USB drive (e.g., `RPI-RP2` or `Arduino`) will appear.
4. **Drag and drop** the `.uf2` file onto that drive. The board will reboot automatically.

### 🔍 3. Recommended Test Procedure
> [!IMPORTANT]
> **Serial Monitor First!**
> Before connecting the device to any external synthesizers or circuits, we strongly recommend checking the internal state:
> 1. Flash the firmware as described above.
> 2. Open a Serial Monitor (Arduino IDE, PuTTY, or Web Serial) at **115200 baud**.
> 3. Verify the initialization logs and connection info. This ensures the firmware is running correctly before hardware is at risk.

---

### ⚖️ Disclaimer / Haftungsausschluss

**DE:** Die Nutzung dieser Firmware erfolgt **auf eigene Gefahr**. Der Autor haftet nicht für jegliche Schäden an Hardware, Software oder anderen Geräten, die durch die Verwendung dieser Dateien entstehen könnten.

**EN:** Use this firmware **at your own risk**. The author is not liable for any damage to hardware, software, or other equipment resulting from the use of these files.