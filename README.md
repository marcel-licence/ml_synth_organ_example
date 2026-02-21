<h1 align="center">ml_synth_organ_example</h1>
<h3 align="center">MIDI Organ using the ML_SynthTools library (little example arduino project)</h3>  
<p align="center"> 
  <img src="img/splash.jpg" alt="project picture" width="480px" height="270px"><br>
  <a href="https://youtu.be/9AHQ4mQrjE8">link to the video</a>
</p>

This project is an example supporting different platforms:
- ESP32
- ESP8266
- Seeedstudio XIAO (samd21 - cortex-m0plus)
- Teensy 4.1 (imxrt1062)
- Daisy Seed (cortex-m7)
- Raspberry Pi Pico (rp2040)

The required library can be found here: https://github.com/marcel-licence/ML_SynthTools

All platforms are tested. Actually the sound quality might be a bit limited.
The organ supports full polyphony (you can play all 64 notes of the upper manual at the same time).
You can modify the sound using 9 drawbars.
Percussion is supported (2nd, 3rd).
A simple rotary implementation creates a simple (noisy) leslie like sound.

Demo using ESP32, ESP8266: https://youtu.be/c7TL8jcrnzs
Demo using the XIAO: https://youtu.be/2wT8nByoUNw
Demo using the Teensy4.1: https://youtu.be/H-NDCQnSDV0

More information will be available in future

<h3>Compileable build configurations</h3>

Below you will find a list of some possible build configurations.
They should all work without any modification of the code.

<hr>
<b>Core:</b> ESP32 Arduino (2.0.4)<br />
<b>Version:</b> 2.0.4<br />
<b>Board:</b> WEMOS D1 MINI ESP32<br />
<br />
<b>Flash Frequency:</b> 80MHz<br />
<b>Partition Scheme:</b> Default<br />
<b>CPU Frequency:</b> 240MHz (WiFi/BT)<br />
<b>Upload Speed:</b> 921600<br />
<b>Core Debug Level:</b> None<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>WiFi</td>
        <td>2.0.0</td>
        <td></td>
        <td>https://github.com/espressif/arduino-esp32.git</td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> ESP32 Arduino<br />
<b>Version:</b> 1.0.6<br />
<b>Board:</b> ESP32 Dev Module<br />
<br />
<b>PSRAM:</b> Disabled<br />
<b>Partition Scheme:</b> No OTA (2MB APP/2MB SPIFFS)<br />
<b>CPU Frequency:</b> 240MHz (WiFi/BT)<br />
<b>Flash Mode:</b> QIO<br />
<b>Flash Frequency:</b> 80MHz<br />
<b>Flash Size:</b> 4MB (32Mb)<br />
<b>Upload Speed:</b> 921600<br />
<b>Core Debug Level:</b> None<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>WiFi</td>
        <td>1.0</td>
        <td></td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> ESP32 Arduino (2.0.5)<br />
<b>Version:</b> 2.0.5<br />
<b>Board:</b> WEMOS D1 MINI ESP32<br />
<br />
<b>Flash Frequency:</b> 80MHz<br />
<b>Partition Scheme:</b> Default<br />
<b>CPU Frequency:</b> 240MHz (WiFi/BT)<br />
<b>Upload Speed:</b> 921600<br />
<b>Core Debug Level:</b> None<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>WiFi</td>
        <td>2.0.0</td>
        <td></td>
        <td>https://github.com/espressif/arduino-esp32.git</td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> ESP32 Arduino (2.0.0)<br />
<b>Version:</b> 2.0.0<br />
<b>Board:</b> ESP32 Dev Module<br />
<br />
<b>PSRAM:</b> Disabled<br />
<b>Partition Scheme:</b> Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)<br />
<b>CPU Frequency:</b> 240MHz (WiFi/BT)<br />
<b>Flash Mode:</b> QIO<br />
<b>Flash Frequency:</b> 80MHz<br />
<b>Flash Size:</b> 4MB (32Mb)<br />
<b>Upload Speed:</b> 921600<br />
<b>Arduino Runs On:</b> Core 1<br />
<b>Events Run On:</b> Core 1<br />
<b>Core Debug Level:</b> None<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>WiFi</td>
        <td>2.0.0</td>
        <td></td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> Raspberry Pi RP2040 Boards(1.13.1)<br />
<b>Version:</b> 1.13.1<br />
<b>Board:</b> Generic RP2040<br />
<br />
<b>Flash Size:</b> 2MB (no FS)<br />
<b>CPU Speed:</b> 125 MHz<br />
<b>Optimize:</b> Small (-Os) (standard)<br />
<b>RTTI:</b> Disabled<br />
<b>Debug Port:</b> Serial<br />
<b>Debug Level:</b> None<br />
<b>USB Stack:</b> Pico SDK<br />
<b>Boot Stage 2:</b> W25Q080 QSPI /4<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> Raspberry Pi RP2040 Boards(1.13.1)<br />
<b>Version:</b> 1.13.1<br />
<b>Board:</b> Raspberry Pi Pico<br />
<br />
<b>Flash Size:</b> 2MB (no FS)<br />
<b>CPU Speed:</b> 125 MHz<br />
<b>Debug Port:</b> Disabled<br />
<b>Debug Level:</b> None<br />
<b>USB Stack:</b> Pico SDK<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> Teensyduino<br />
<b>Version:</b> 1.8.5<br />
<b>Board:</b> Teensy 4.1<br />
<br />
<b>USB Type:</b> Serial + MIDI + Audio<br />
<b>CPU Speed:</b> 600 MHz<br />
<b>Optimize:</b> Faster<br />
<b>Keyboard Layout:</b> US English<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>Audio</td>
        <td>1.3</td>
        <td>http://www.pjrc.com/teensy/td_libs_Audio.html</td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>SPI</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/SPI</td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>SD</td>
        <td>2.0.0</td>
        <td>https://github.com/PaulStoffregen/SD</td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>SerialFlash</td>
        <td>0.5</td>
        <td>https://github.com/PaulStoffregen/SerialFlash</td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>Wire</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/Wire</td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> STM32GENERIC for STM32 boards<br />
<b>Version:</b> 1.0.0<br />
<b>Board:</b> BLACK F407VG/VE/ZE/ZG boards<br />
<br />
<b>USB:</b> Serial [Virtual COM port, PA11/PA12 pins]<br />
<b>Serial communication:</b> SerialUSB<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
    <tr>
        <td>DMA</td>
        <td>1.0</td>
        <td></td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> STM32GENERIC for STM32 boards<br />
<b>Version:</b> 1.0.0<br />
<b>Board:</b> Discovery F407VG<br />
<br />
<b>USB:</b> Serial [Virtual COM port]<br />
<b>Serial communication:</b> SerialUSB<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>Wire</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/Wire</td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
    <tr>
        <td>DMA</td>
        <td>1.0</td>
        <td></td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> STM32 boards groups (Board to be selected from Tools submenu 'Board part number')<br />
<b>Version:</b> 2.1.0<br />
<b>Board:</b> Generic STM32H7 Series<br />
<br />
<b>Board part number:</b> Daisy Seed<br />
<b>Upload method:</b> STM32CubeProgrammer (DFU)<br />
<b>U(S)ART support:</b> Enabled (generic 'Serial')<br />
<b>USB support (if available):</b> CDC (generic 'Serial' supersede U(S)ART)<br />
<b>USB speed (if available):</b> Low/Full Speed<br />
<b>Optimize:</b> Smallest (-Os default)<br />
<b>Debug symbols:</b> None<br />
<b>C Runtime Library:</b> Newlib Nano (default)<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>DaisyDuino</td>
        <td>1.4.0</td>
        <td>https://github.com/electro-smith/DaisyDuino</td>
        <td>https://github.com/marcel-licence/DaisyDuino.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>Wire</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/Wire</td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>Source Wrapper</td>
        <td>1.0.1</td>
        <td></td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> ESP8266 Boards (3.0.2)<br />
<b>Version:</b> 3.0.2<br />
<b>Board:</b> LOLIN(WEMOS) D1 R2 & mini<br />
<br />
<b>CPU Frequency:</b> 160 MHz<br />
<b>VTables:</b> Flash<br />
<b>C++ Exceptions:</b> Disabled (new aborts on oom)<br />
<b>Stack Protection:</b> Disabled<br />
<b>SSL Support:</b> All SSL ciphers (most compatible)<br />
<b>MMU:</b> 32KB cache + 32KB IRAM (balanced)<br />
<b>Non-32-Bit Access:</b> Use pgm_read macros for IRAM/PROGMEM<br />
<b>Flash Size:</b> 4MB (FS:2MB OTA:~1019KB)<br />
<b>lwIP Variant:</b> v2 Lower Memory<br />
<b>Debug port:</b> Disabled<br />
<b>Debug Level:</b> None<br />
<b>Erase Flash:</b> Only Sketch<br />
<b>Upload Speed:</b> 921600<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>ESP8266WiFi</td>
        <td>1.0</td>
        <td></td>
        <td></td>
        <td>True</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td></td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> Seeed SAMD (32-bits ARM Cortex-M0+ and Cortex-M4) Boards<br />
<b>Version:</b> 1.8.2<br />
<b>Board:</b> Seeeduino XIAO<br />
<br />
<b>USB Stack:</b> Arduino<br />
<b>Debug:</b> Off<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
</table><hr>
<b>Core:</b> STM32GENERIC for STM32 boards<br />
<b>Version:</b> 1.0.0<br />
<b>Board:</b> BLUE F103VE/ZE/ZG boards<br />
<br />
<b>USB:</b> Serial [Virtual COM port, PA11/PA12 pins]<br />
<b>Serial communication:</b> SerialUSB<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
    <tr>
        <td>DMA</td>
        <td>1.0</td>
        <td></td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
</table><hr>
<b>Core:</b> STM32GENERIC for STM32 boards<br />
<b>Version:</b> 1.0.0<br />
<b>Board:</b> BluePill F103CB<br />
<br />
<b>USB:</b> Serial [Virtual COM port, PA11/PA12 pins]<br />
<b>Serial communication:</b> SerialUSB<br />
<br />
<b>Used libraries:</b><br />
<table>
    <tr>
        <td>Name</td>
        <td>Version</td>
        <td>Url</td>
        <td>Git</td>
        <td>Core library</td>
    </tr>
    <tr>
        <td>ML SynthTools</td>
        <td>1.0.1</td>
        <td>https://github.com/marcel-licence/ML_SynthTools</td>
        <td>https://github.com/marcel-licence/ML_SynthTools.git</td>
        <td>False</td>
    </tr>
    <tr>
        <td>I2S</td>
        <td>1.0</td>
        <td>http://www.arduino.cc/en/Reference/I2S</td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
    <tr>
        <td>DMA</td>
        <td>1.0</td>
        <td></td>
        <td>https://github.com/marcel-licence/STM32GENERIC.git</td>
        <td>True</td>
    </tr>
</table>
