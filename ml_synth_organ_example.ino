/*
 * Copyright (c) 2023 Marcel Licence
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
#include <caps_info.h>
#ifdef USE_ML_SYNTH_PRO
#include <ml_organ_pro.h>
#include <ml_system.h>
#else
#include <ml_organ.h>
void Organ_Process_Buf(int *buf, uint8_t len);
#endif
#ifdef REVERB_ENABLED
#include <ml_reverb.h>
#endif
#include <ml_delay.h>
#ifdef OLED_OSC_DISP_ENABLED
#include <ml_scope.h>
#endif

#include <ml_lfo.h>
#ifdef VIBRATO_ENABLED
#include <ml_vibrato.h>
#endif

#include <ml_types.h>
#include <ml_utils.h>


#define ML_SYNTH_INLINE_DECLARATION
#include <ml_inline.h>
#undef ML_SYNTH_INLINE_DECLARATION


#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
#include <Wire.h> /* todo remove, just for scanning */
#endif


const char shortName[] = "ML_Organ";


#ifdef VOLUME_CONTROL_ENABLED
float mainVolume = 1.0f;
float mainVolumeSet = 1.0f;
#endif


float lfo1_max = 1.0f;
float lfo1_max_soft = 1.0f;
float lfo1_buffer[SAMPLE_BUFFER_SIZE];
float lfo1_buffer_scale[SAMPLE_BUFFER_SIZE];
ML_LFO lfo1(SAMPLE_RATE, lfo1_buffer, SAMPLE_BUFFER_SIZE);

#ifdef VIBRATO_ENABLED
ML_Vibrato vibrato(SAMPLE_RATE);
#endif

#define SERIAL_WAIT_READY_MAX_DURATION_MS   5000

void setup()
{
    /*
     * this code runs once
     */
#ifdef BLINK_LED_PIN
    Blink_Setup();
    Blink_Fast(1);
#endif

#ifndef SWAP_SERIAL
    Serial.begin(115200);

    unsigned long start = millis();
    while (!Serial && (millis() - start < SERIAL_WAIT_READY_MAX_DURATION_MS))
    {
        // wait max 2 seconds
    }
    delay(3000);
#endif

    CapsPrintInfo();

#ifdef ML_BOARD_SETUP
    Board_Setup();
#else
#ifdef MIDI_USB_ENABLED
    Midi_Usb_Setup();
#endif
#endif

#ifdef ARDUINO_DAISY_SEED
    DaisySeed_Setup();
#endif

    delay(500);

#ifdef SWAP_SERIAL
    /* only one hw serial use this for ESP */
    Serial.begin(115200);
    delay(500);
#endif

    Serial.println();


    Serial.printf("Loading data\n");


#ifdef ESP32
#ifdef USE_ML_SYNTH_PRO
    char user[] = "";
    System_PrintInfo(user);
#endif
#endif

#ifndef ML_BOARD_SETUP
#ifdef ESP8266
    Midi_Setup();
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

#ifndef ESP8266 /* otherwise it will break audio output */
    Midi_Setup();
#endif
#endif

#endif
#endif /* #ifndef ML_BOARD_SETUP */

#ifdef USE_ML_SYNTH_PRO
#if !defined(SOC_CPU_HAS_FPU) && !defined(TEENSYDUINO) && !defined(__ARM_FEATURE_DSP)
    Serial.printf("Synth might not work because CPU does not have a FPU (floating point unit)\n");
#endif
    OrganPro_Setup(SAMPLE_RATE);
#else
    Organ_Setup(SAMPLE_RATE);
#endif

#ifdef REVERB_ENABLED
    /*
     * Initialize reverb
     * The buffer shall be static to ensure that
     * the memory will be exclusive available for the reverb module
     */
    //static float revBuffer[REV_BUFF_SIZE];
    static float *revBuffer = (float *)malloc(sizeof(float) * REV_BUFF_SIZE);
    Reverb_Setup(revBuffer);
#endif

#ifdef MAX_DELAY
    /*
     * Prepare a buffer which can be used for the delay
     */
    //static int16_t delBuffer1[MAX_DELAY];
    //static int16_t delBuffer2[MAX_DELAY];
    static int16_t *delBuffer1 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    static int16_t *delBuffer2 = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    Delay_Init2(delBuffer1, delBuffer2, MAX_DELAY);

    Delay_SetLength(0, 0.5f);
    Delay_SetOutputLevel(0, 0.0f);
    Delay_SetFeedback(0, 0.02f);
#endif

    lfo1.setPhase(0);


#ifdef MIDI_BLE_ENABLED
    midi_ble_setup();
#endif

#ifdef USB_HOST_ENABLED
    Usb_Host_Midi_setup();
#endif

#ifdef ESP32
    Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

    /* PSRAM will be fully used by the looper */
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
#endif

    Serial.printf("Firmware started successfully\n");

#ifdef NOTE_ON_AFTER_SETUP
#ifdef USE_ML_SYNTH_PRO
    OrganPro_NoteOn(0, 60, 127);
    OrganPro_SetLeslCtrl(127);
#else
    Organ_NoteOn(0, 60, 127);
    Organ_SetLeslCtrl(127);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
    Organ_PercussionSet(CTRL_ROTARY_ACTIVE);
#endif
#endif

#ifdef VIBRATO_ENABLED
#if 0
    Lfo1_SetDepth(0, 32);
#else
    Lfo1_SetDepth(0, 0);
#endif
    Lfo1_SetSpeed(0, 71);
#endif

#ifdef MIDI_STREAM_PLAYER_ENABLED
    MidiStreamPlayer_Init();

    char midiFile[] = "/song.mid";
    MidiStreamPlayer_PlayMidiFile_fromLittleFS(midiFile, 1);
#endif

#if (defined MIDI_VIA_USB_ENABLED) || (defined OLED_OSC_DISP_ENABLED)
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
void Core0TaskInit(void)
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

void Core0TaskSetup(void)
{
    /*
     * init your stuff for core0 here
     */

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Setup();
#endif
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
}

void Core0TaskLoop(void)
{
    /*
     * put your loop stuff for core0 here
     */
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Loop();
#endif

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Process();
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
#endif /* ESP32 */

void loop_1Hz(void)
{
#ifdef CYCLE_MODULE_ENABLED
    CyclePrint();
#endif
#ifdef BLINK_LED_PIN
    Blink_Process();
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
#ifdef MIDI_STREAM_PLAYER_ENABLED
    MidiStreamPlayer_Tick(SAMPLE_BUFFER_SIZE);
#endif

#ifdef MIDI_BLE_ENABLED
    midi_ble_loop();
#endif

#ifdef USB_HOST_ENABLED
    Usb_Host_Midi_loop();
#endif

#ifdef MIDI_USB_ENABLED
    Midi_Usb_Loop();
#endif

    /*
     * And finally the audio stuff
     */
#ifdef USE_ML_SYNTH_PRO
#if (defined ESP8266) || (defined ARDUINO_SEEED_XIAO_M0) || (defined ARDUINO_RASPBERRY_PI_PICO) || (defined ARDUINO_GENERIC_RP2040)
#error Configuration is not supported
#else
    float mono[SAMPLE_BUFFER_SIZE], left[SAMPLE_BUFFER_SIZE], right[SAMPLE_BUFFER_SIZE];
    OrganPro_Process_fl(mono, SAMPLE_BUFFER_SIZE);

    /* reduce output to avoid clipping */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] *= 0.125f;
    }

#ifdef VOLUME_CONTROL_ENABLED
    /* smooth main organ volume */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] *= mainVolume;
        mainVolume = (mainVolume * 0.9995f) + (mainVolumeSet * 0.0005f);
    }
#endif

    lfo1.Process(SAMPLE_BUFFER_SIZE);
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        lfo1_max_soft = lfo1_max * 0.01 + lfo1_max_soft * 0.99;
        lfo1_buffer_scale[i] = lfo1_buffer[i] * lfo1_max_soft;
    }

#ifdef VIBRATO_ENABLED
    vibrato.ProcessHQ(mono, lfo1_buffer_scale, mono, SAMPLE_BUFFER_SIZE);
#endif

#ifdef INPUT_TO_MIX
    Audio_Input(left, right);
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] += (left[i] * 0.5f + right[i] * 0.5f) * 32.0f;
    }
#endif

#ifdef REVERB_ENABLED
    Reverb_Process(mono, SAMPLE_BUFFER_SIZE);
#endif

    Rotary_Process(left, right, mono, SAMPLE_BUFFER_SIZE);

#ifdef MAX_DELAY
    /*
     * post process delay
     */
    Delay_Process_Buff2(left, right, SAMPLE_BUFFER_SIZE);
#endif

    /*
     * Output the audio
     */
    Audio_Output(left, right);

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_AddSamples(left, right, SAMPLE_BUFFER_SIZE);
#ifdef TEENSYDUINO
    static uint8_t x = 0;
    x++;
    if (x == 0)
    {
        ScopeOled_Process();
    }
#endif
#endif
#endif
#else
    int32_t mono[SAMPLE_BUFFER_SIZE];
    Organ_Process_Buf((int *)mono, SAMPLE_BUFFER_SIZE);

#ifdef VOLUME_CONTROL_ENABLED
    /* smooth main organ volume */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        float mono_f = mono[i];
        mono_f *= mainVolume;
        mainVolume = (mainVolume * 0.9995f) + (mainVolumeSet * 0.0005f);
        mono[i] = mono_f;
    }
#endif

#ifdef REVERB_ENABLED
    float mono_f[SAMPLE_BUFFER_SIZE];
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono_f[i] = mono[i];
    }
    Reverb_Process(mono_f, SAMPLE_BUFFER_SIZE); /* post reverb */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] = mono_f[i];
    }
#endif

#ifdef PICO_AUDIO_I2S
    /* we expect 32 bit audio and the buffer used only 16 results in no sounds */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono[i] <<= 16;
    }
#endif

    Audio_OutputMono(mono);
#endif
}

/*
 * MIDI via USB Host Module
 */
#ifdef MIDI_VIA_USB_ENABLED
void App_UsbMidiShortMsgReceived(uint8_t *msg)
{
#ifdef MIDI_TX2_PIN
    Midi_SendShortMessage(msg);
#endif
    Midi_HandleShortMsg(msg, 8);
}
#endif

/*
 * MIDI callbacks
 */
inline void App_MainVolume(uint8_t unused __attribute__((unused)), uint8_t value)
{
#ifdef VOLUME_CONTROL_ENABLED
    mainVolumeSet = log10fromU7(value, -2, 0);
    Status_ValueChangedFloat("Main Volume", mainVolumeSet);
#endif
}

void Lfo1_SetSpeed(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float f = log10fromU7(value, -2, 3);
    lfo1.setFrequency(f);
}

void Lfo1_SetDepth(uint8_t unused __attribute__((unused)), uint8_t value)
{
    if (value > 0)
    {
        lfo1_max = log2fromU7(value, -6, 0);
    }
    else
    {
        lfo1_max = 0;
    }
}

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

inline void Reverb_SetLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Reverb_SetLevel(unused, val);
#endif
}

inline void Delay_SetOutputLevelInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetOutputLevel(unused, val);
#endif
}

inline void Delay_SetFeedbackInt(uint8_t unused __attribute__((unused)), uint8_t value)
{
    float val = value;
    val /= 127.0f;
#ifdef REVERB_ENABLED
    Delay_SetFeedback(unused, val);
#endif
}

#if (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG)
void ScanI2C(void)
{
#ifdef ARDUINO_GENERIC_F407VGTX
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();//I2C_SDA, I2C_SCL);
#else
    Wire.begin();
#endif

    byte address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        byte r_error;
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
#endif /* (defined ARDUINO_GENERIC_F407VGTX) || (defined ARDUINO_DISCO_F407VG) */

