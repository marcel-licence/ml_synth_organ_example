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
