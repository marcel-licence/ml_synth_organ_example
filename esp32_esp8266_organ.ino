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
#include <ml_organ.h>


#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
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

    Organ_Setup();



    WiFi.mode(WIFI_OFF);
#if 0 //ndef ESP8266
    btStop();
    esp_wifi_deinit();
#endif

    pinMode(LED_PIN, OUTPUT);


    setup_Serial2();

#ifdef I2S_NODAC
    I2S_init();
#else
    setup_i2s();
#endif

    pinMode(2, INPUT); //restore GPIOs taken by i2s
    pinMode(15, INPUT);


#if 0 /* set this to one to test the audio output with a noteOn event on startup */
    Organ_NoteOn(0, 60, 127);
#endif
}

void loop()
{
    static int midi_cnt = 0; /*!< used to reduce MIDI processing time */
    static int led_cnt = 0; /*!< used for delay of the blinking LED */

    /*
     * generates a signal of 44100/256 -> ~172 Hz. If lower than we have buffer underruns -> audio drop outs
     */
    if (led_cnt > 22050)
    {
        led_cnt = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));   // turn the LED on (HIGH is the voltage level)
    }
    led_cnt++;

#ifdef ESP8266
    // I2S_Wait();
    if (I2S_isNotFull())
    {
        int16_t sig = Organ_Process();
        sig *= 4;
        // Serial.printf("%d\n", sig);
        writeDAC(0x8000 + sig);
    }
#else
    int16_t sig = Organ_Process();
    //static int16_t sig = 0;
    //sig += 1024;
    i2s_write_stereo_samples_i16(&sig, &sig);
#endif

    midi_cnt++;
    if (midi_cnt > 64)
    {
#ifdef ESP8266
        Midi_CheckSerial2();
#else
        Midi_Process();
#endif
        midi_cnt = 0;
    }
}

/*
 * MIDI callbacks
 */

inline void Organ_PercSetMidi(uint8_t setting, uint8_t value)
{
    if (value > 0)
    {
        Organ_PercussionSet(setting);
    }
}

inline void Organ_SetDrawbarInv(uint8_t id, uint8_t value)
{
    Organ_SetDrawbar(id, value);
}

inline void Organ_SetCtrl(uint8_t unused __attribute__((unused)), uint8_t value)
{
    Organ_SetLeslCtrl(value);
}

inline void Organ_SetLeslieSpeedNorm(uint8_t unused __attribute__((unused)), uint8_t speed)
{
    Organ_SetLeslCtrl(speed);
}

inline void Organ_ModulationWheel(uint8_t unused __attribute__((unused)), uint8_t value)
{
    Organ_SetLeslCtrl(value);
}
