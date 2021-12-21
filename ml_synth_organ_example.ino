/*
 * Copyright (c) 2021 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file ml_synth_organ_example.ino
 * @author Marcel Licence
 * @date 26.11.2021
 *
 * @brief   This is the main project file to test the ML_SynthLibrary (organ module)
 *          It should be compatible with ESP32, ESP8266, Seedstudio XIAO, PRJC Teensy 4.1, Electrosmith Daisy Seed, Raspberry Pi Pico, STM32F4
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include "config.h"


#include <Arduino.h>

/*
 * Library can be found on https://github.com/marcel-licence/ML_SynthTools
 */
#ifdef USE_ML_SYNTH_PRO
#include <ml_organ_pro.h>
#else
#include <ml_organ.h>
#endif

#ifdef ARDUINO_GENERIC_F407VGTX
#include <Wire.h> /* todo remove, just for scanning */
#endif

void blink(uint8_t cnt)
{
    delay(500);
    for (int i = 0; i < cnt; i++)
    {

        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
}

void blink_slow(uint8_t cnt)
{
    delay(500);
    for (int i = 0; i < cnt; i++)
    {

        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void setup()
{
    /*
     * this code runs once
     */

    pinMode(LED_PIN, OUTPUT);
    blink(1);

#ifdef ARDUINO_DAISY_SEED
    DaisySeed_Setup();
#endif

    delay(500);

#ifdef SWAP_SERIAL
    /* only one hw serial use this for ESP */
    Serial.begin(115200);
#else
    Serial.begin(115200);
#endif
    delay(500);

    Serial.println();


    Serial.printf("Loading data\n");

    Serial.printf("Firmware started successfully\n");

#ifdef USE_ML_SYNTH_PRO
    OrganPro_Setup(&Serial, SAMPLE_RATE);
#else
    Organ_Setup(&Serial, SAMPLE_RATE);
#endif


#ifdef ESP8266
    setup_Serial2();
#endif

    Serial.printf("Initialize Audio Interface\n");
    Audio_Setup();

#ifdef TEENSYDUINO
    Teensy_Setup();
#else

#ifdef ARDUINO_SEEED_XIAO_M0
    pinMode(7, INPUT_PULLUP);
    Midi_Setup();
    pinMode(LED_BUILTIN, OUTPUT);
#else
    pinMode(LED_PIN, OUTPUT);
#ifndef ESP8266 /* otherwise it will break audio output */
    setup_Serial2();
#endif
#endif

#endif








#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif

#if 1 /* set this to one to test the audio output with a noteOn event on startup */
#ifdef USE_ML_SYNTH_PRO
    OrganPro_NoteOn(0, 60, 127);
    OrganPro_SetLeslCtrl(17);
#else
    Organ_NoteOn(0, 60, 127);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
    Organ_SetLeslCtrl(127);
#endif
#endif

#if (defined MIDI_VIA_USB_ENABLED)
#ifdef ESP32
    Core0TaskInit();
#else
#error only supported by ESP32 platform
#endif
#endif
}

#ifdef ESP32
/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

void Core0TaskSetup()
{
    /*
     * init your stuff for core0 here
     */
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
}

void Core0TaskLoop()
{
    /*
     * put your loop stuff for core0 here
     */
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Loop();
#endif
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}
#endif



void loop_1Hz()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));   // turn the LED on (HIGH is the voltage level)
#ifdef CYCLE_MODULE_ENABLED
    CyclePrint();
#endif
}

void loop()
{
    static int loop_cnt_1hz = 0; /*!< counter to allow 1Hz loop cycle */

#ifdef SAMPLE_BUFFER_SIZE
    loop_cnt_1hz += SAMPLE_BUFFER_SIZE;
#else
    loop_cnt_1hz += 1; /* in case only one sample will be processed per loop cycle */
#endif
    if (loop_cnt_1hz >= SAMPLE_RATE)
    {
        loop_cnt_1hz -= SAMPLE_RATE;
        loop_1Hz();
    }

    /*
     * MIDI processing
     */
    Midi_Process();
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_ProcessSync();
#endif

    /*
     * And finally the audio stuff
     */
#ifdef USE_ML_SYNTH_PRO
#if (defined ESP8266) || (defined ARDUINO_SEEED_XIAO_M0)|| (defined ARDUINO_RASPBERRY_PI_PICO)
#error Configuration is not supported
#else
    float mono[SAMPLE_BUFFER_SIZE], left[SAMPLE_BUFFER_SIZE], right[SAMPLE_BUFFER_SIZE];
    OrganPro_Process_fl(mono, SAMPLE_BUFFER_SIZE);
    Rotary_Process(left, right, mono, SAMPLE_BUFFER_SIZE);
    Audio_Output(left, right);
#endif
#else
    int32_t mono[SAMPLE_BUFFER_SIZE];
    Organ_Process_Buf(mono, SAMPLE_BUFFER_SIZE);
    Audio_OutputMono(mono);
#endif
}

/*
 * MIDI via USB Host Module
 */
#ifdef MIDI_VIA_USB_ENABLED
void App_UsbMidiShortMsgReceived(uint8_t *msg)
{
    Midi_SendShortMessage(msg);
    Midi_HandleShortMsg(msg, 8);
}
#endif

/*
 * MIDI callbacks
 */
inline void Organ_PercSetMidi(uint8_t setting, uint8_t value)
{
    if (value > 0)
    {
#ifdef USE_ML_SYNTH_PRO
        OrganPro_PercussionSet(setting);
#else
        Organ_PercussionSet(setting);
#endif
    }
}

inline void Organ_SetDrawbarInv(uint8_t id, uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetDrawbar(id, value);
#else
    Organ_SetDrawbar(id, value);
#endif
}

inline void Organ_SetCtrl(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
#endif
}

inline void Organ_SetLeslieSpeedNorm(uint8_t unused __attribute__((unused)), uint8_t speed)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(speed);
#else
    Organ_SetLeslCtrl(speed);
#endif
}

inline void Organ_ModulationWheel(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef USE_ML_SYNTH_PRO
    OrganPro_SetLeslCtrl(value);
#else
    Organ_SetLeslCtrl(value);
#endif
}

#ifdef ARDUINO_GENERIC_F407VGTX
void  ScanI2C(void)
{
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();//I2C_SDA, I2C_SCL);

    byte r_error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        r_error = Wire.endTransmission();

        if (r_error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (r_error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
#endif

