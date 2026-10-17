/*
 * Copyright (c) 2026 Marcel Licence
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
 * @date 21.03.2026
 *
 * @brief This is the main project file which handles the app startup and loop calls
 *        You will find the main code in app.cpp
 */


#include "app.h"


#if (defined ESP32) && (SOC_CPU_CORES_NUM > 1)
void Core0TaskInit(void);
void Core0Task(void *parameter);
#endif

#ifdef ARDUINO_RP2040
volatile bool g_setup_done = false;
#endif


void setup()
{
    Serial.begin(115200);
    App_Setup();

#ifdef ARDUINO_RP2040
    g_setup_done = true;
#endif

#if (defined ESP32) && (SOC_CPU_CORES_NUM > 1)
    Core0TaskInit();
#endif
}

void loop()
{
    App_Loop();
}

#ifdef ARDUINO_RP2040

void wait_until_setup_finished(void)
{
    while (!g_setup_done)
    {
        delay(1);
    }
}

void setup1()
{
    wait_until_setup_finished();
    App_Setup1();
}

void loop1()
{
    App_Loop1();
}
#endif


#if (defined ESP32) && (SOC_CPU_CORES_NUM > 1)
/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

void Core0TaskInit(void)
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 4, &Core0TaskHnd, 0);
}

void Core0TaskSetup(void)
{
    /*
     * init your stuff for core0 here
     */
    App_Setup1();
}

void Core0TaskLoop(void)
{
    App_Loop1();
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
    Organ_Process_Buf(mono, SAMPLE_BUFFER_SIZE);

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

#if (defined REVERB_ENABLED) || (defined OLED_OSC_DISP_ENABLED)
    float mono_f[SAMPLE_BUFFER_SIZE];
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        mono_f[i] = mono[i];
    }
#endif

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_AddSamples(mono_f, mono_f, SAMPLE_BUFFER_SIZE);
#endif

#ifdef REVERB_ENABLED
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
