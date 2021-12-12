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
 * @file esp32_esp8266_organ.ino
 * @author Marcel Licence
 * @date 26.11.2021
 *
 * @brief   This is the main project file to test the ML_SynthLibrary (organ module)
 *          It should be compatible with ESP32 and ESP8266
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#include "config.h"


/*
 * Library can be found on https://github.com/marcel-licence/ML_SynthTools
 */
#ifdef USE_ML_SYNTH_PRO
#include <ml_organ_pro.h>
#else
#include <ml_organ.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif

#ifdef ESP32
#include <WiFi.h>
#endif

#ifdef TEENSYDUINO
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#endif

#ifdef ARDUINO_DAISY_SEED
#include "DaisyDuino.h"
#endif

void setup()
{
    // put your setup code here, to run once:
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
    OrganPro_Setup();
#else
    Organ_Setup();
#endif

#if (defined ESP8266) || (defined ESP32)
    WiFi.mode(WIFI_OFF);
#endif

#ifdef TEENSYDUINO
    Teensy_Setup();
#else
    pinMode(LED_PIN, OUTPUT);
    setup_Serial2();
#endif

#ifdef ARDUINO_DAISY_SEED
    DaisySeed_Setup();
#endif

#if 0 //ndef ESP8266
    btStop();
    esp_wifi_deinit();
#endif



#if (defined ESP8266) || (defined ESP32)
#ifdef I2S_NODAC
    I2S_init();
#else
    setup_i2s();
#endif

    pinMode(2, INPUT); //restore GPIOs taken by i2s
    pinMode(15, INPUT);
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
    Organ_SetLeslCtrl(17);
#endif
#endif

#ifdef MIDI_VIA_USB_ENABLED
    Core0TaskInit();
#endif
}


#ifdef TEENSYDUINO

const int ledPin = LED_PIN; /* pin configured in config.h */

#if 1
AudioPlayQueue           queue1;
AudioPlayQueue           queue2;
AudioOutputI2S           i2s1;
AudioConnection          patchCord1(queue1, 0, i2s1, 0); /* left channel */
AudioConnection          patchCord2(queue2, 0, i2s1, 1); /* right channel */
#else

// GUItool: begin automatically generated code
AudioInputUSB            usb1;           //xy=134,545
AudioPlayQueue           queue1;         //xy=258,380
AudioPlayQueue           queue2;         //xy=261,423
AudioRecordQueue         queue6;         //xy=265,557
AudioRecordQueue         queue5;         //xy=274,516
AudioPlayQueue           queue4;         //xy=352,622
AudioPlayQueue           queue3;         //xy=380,530
AudioOutputI2S           i2s1;           //xy=470,393
AudioOutputUSB           usb2;           //xy=494,544
AudioConnection          patchCord1(usb1, 0, queue5, 0);
AudioConnection          patchCord2(usb1, 1, queue6, 0);
AudioConnection          patchCord3(queue1, 0, i2s1, 0);
AudioConnection          patchCord4(queue2, 0, i2s1, 1);
AudioConnection          patchCord5(queue3, 0, usb2, 0);
AudioConnection          patchCord6(queue4, 0, usb2, 1);
// GUItool: end automatically generated code
#endif


static int16_t   sampleBuffer[AUDIO_BLOCK_SAMPLES];
static int16_t   sampleBuffer2[AUDIO_BLOCK_SAMPLES];
static int16_t   *queueTransmitBuffer;
static int16_t   *queueTransmitBuffer2;

void Teensy_Setup()
{
    AudioMemory(4);
    pinMode(ledPin, OUTPUT);
    Midi_Setup();
}

#endif /* TEENSYDUINO */

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
    static int midi_cnt = 0; /*!< used to reduce MIDI processing time */
    static int led_cnt = 0; /*!< used for delay of the blinking LED */



    /*
     * generates a signal of 44100/256 -> ~172 Hz. If lower than we have buffer underruns -> audio drop outs
     */
    if (led_cnt >= 44100)
    {
        led_cnt = 0;
        loop_1Hz();
    }
#ifdef TEENSYDUINO
    led_cnt += AUDIO_BLOCK_SAMPLES;
#else
    led_cnt++;
#endif

#ifdef ESP8266
    // I2S_Wait();
    if (I2S_isNotFull())
    {
        int16_t sig = Organ_Process();
        sig *= 4;
        // Serial.printf("%d\n", sig);
        writeDAC(0x8000 + sig);
    }
#endif /* ESP8266 */

#ifdef ESP32
#ifdef USE_ML_SYNTH_PRO
    led_cnt--;
    led_cnt += SAMPLE_BUFFER_SIZE;
    float mono[SAMPLE_BUFFER_SIZE], left[SAMPLE_BUFFER_SIZE], right[SAMPLE_BUFFER_SIZE];
    OrganPro_Process_fl(mono, SAMPLE_BUFFER_SIZE);
    Rotary_Process(left, right, mono, SAMPLE_BUFFER_SIZE);
    i2s_write_stereo_samples_buff(left, right, SAMPLE_BUFFER_SIZE);
#else
    int16_t sig = Organ_Process();
    //static int16_t sig = 0;
    //sig += 1024;

    i2s_write_stereo_samples_i16(&sig, &sig);
#endif
#endif /* ESP32 */

#ifdef TEENSYDUINO
    {
#ifdef CYCLE_MODULE_ENABLED
        calcCycleCountPre();
#endif
        queueTransmitBuffer = queue1.getBuffer(); /* blocking? */
        queueTransmitBuffer2 = queue2.getBuffer();
#ifdef CYCLE_MODULE_ENABLED
        calcCycleCount();
#endif
        if (queueTransmitBuffer)
        {
            {
#ifdef USE_ML_SYNTH_PRO
                float mono[AUDIO_BLOCK_SAMPLES], left[AUDIO_BLOCK_SAMPLES], right[AUDIO_BLOCK_SAMPLES];
                OrganPro_Process_fl(mono, AUDIO_BLOCK_SAMPLES);
                Rotary_Process(left, right, mono, AUDIO_BLOCK_SAMPLES);
                for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
                {
                    sampleBuffer[i] = (int16_t)(left[i] * INT16_MAX);
                    sampleBuffer2[i] = (int16_t)(right[i] * INT16_MAX);
                }

                memcpy(queueTransmitBuffer, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
                memcpy(queueTransmitBuffer2, sampleBuffer2, AUDIO_BLOCK_SAMPLES * 2);
#else
                int32_t u32buf[AUDIO_BLOCK_SAMPLES];

                Organ_Process_Buf(u32buf, AUDIO_BLOCK_SAMPLES);

                for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
                {
                    sampleBuffer[i] = (int16_t)((u32buf[i]));
                }

                memcpy(queueTransmitBuffer, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
                memcpy(queueTransmitBuffer2, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
#endif

            }

            queue1.playBuffer();
            queue2.playBuffer();
        }
    }
#endif /* TEENSYDUINO */

#ifdef SAMPLE_BUFFER_SIZE
    midi_cnt += SAMPLE_BUFFER_SIZE;
#else
    midi_cnt++;
#endif
    if (midi_cnt > 64)
    {
#ifdef ESP8266
        Midi_CheckSerial2();
#else
        Midi_Process();
#endif
#ifdef MIDI_VIA_USB_ENABLED
        UsbMidi_ProcessSync();
#endif
        midi_cnt = 0;
    }
}

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

