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
* @file audio_module.ino
* @author Marcel Licence
* @date 16.12.2021
*
* @brief  this file provides a general audio interface to make it easier working with different platforms
*/


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <I2S.h>
#include <i2s_reg.h>
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

#ifdef ARDUINO_RASPBERRY_PI_PICO
#include <I2S.h>
#endif

#ifdef ARDUINO_GENERIC_F407VGTX
/* this class is not available in the official arduino lib */
//#include <I2S.h>
//I2SClass I2S(SPI3, PC12 /*sd*/, PA4 /*ws*/, PC10 /*ck*/, PC7/* MCK*/);  // setup for STM32F4 Discovery, check your board schematic
#endif

void Audio_Setup(void)
{
#if (defined ESP8266) || (defined ESP32)
    WiFi.mode(WIFI_OFF);
#endif

#if 0 //ndef ESP8266
    btStop();
    esp_wifi_deinit();
#endif

#ifdef ESP32_AUDIO_KIT
#ifdef ES8388_ENABLED
    ES8388_Setup();
#else
    ac101_setup();
#endif
#endif

#ifdef ESP32
    setup_i2s();
#endif

#ifdef ESP8266
#if 0
    system_update_cpu_freq(160);
    i2s_begin();
    i2s_set_rate(SAMPLE_RATE);
#else
    I2S_init();
#endif
    pinMode(2, INPUT); //restore GPIOs taken by i2s
    pinMode(15, INPUT);
#endif


#ifdef TEENSYDUINO
    AudioMemory(4);
#endif

#ifdef ARDUINO_SEEED_XIAO_M0
    SAMD21_Synth_Init();
    pinMode(DAC0, OUTPUT);
#endif

#ifdef ARDUINO_RASPBERRY_PI_PICO
    if (!I2S.begin(SAMPLE_RATE))
    {
        Serial.println("Failed to initialize I2S!");
        while (1); // do nothing
    }
#endif

#ifdef ARDUINO_GENERIC_F407VGTX
    /*
     * Todo Implementation for the STM32F407VGT6
     * Can be found on the ST Discovery Board
     */

    /* reset the codec to allow access */
    pinMode(DAC_RESET, OUTPUT);

    digitalWrite(DAC_RESET, LOW);
    delay(100);
    digitalWrite(DAC_RESET, HIGH);
    delay(200);

    /* Following should list the connected codec which should be available after reset */
    ScanI2C();

    /* @see https://www.mouser.de/datasheet/2/76/CS43L22_F2-1142121.pdf */
    Serial.printf("Dev 0x%02x: 0x01 - 0x%02x\n", 0x1A, I2C_ReadReg(0x1A, 0x01)); /* not sure what this is - gyro? */
    Serial.printf("Dev 0x%02x: 0x01 - 0x%02x\n", 0x4A, I2C_ReadReg(0x4A, 0x01)); /* this should return Chip I.D. | Chip Revision e.g.: 0xe3 */

    /*
     * now it would be perfect to setup I2S and the line output of the DAC
     */
#if 0
    I2C_WriteReg(0x4A, 0x1E, 0xC0);





    /*
     * https://github.com/theCore-embedded/theCore/blob/develop/dev/cs43l22/export/dev/cs43l22.hpp
     */

    //ecl_cs43l22_instance_init();

    ecl_cs43l22_instance_power_up();


    // Set volumes, so audio can be heard.

    ecl_cs43l22_instance_set_master_volume(0x90);

    ecl_cs43l22_instance_set_headphone_volume(0xe0);

    ecl_cs43l22_instance_headphone_unmute();
#else
    codec_writeReg(0x02, 0x01); // power save registers -> all on
    codec_writeReg(0x00, 0x99);
    codec_writeReg(0x47, 0x80); //inits
    codec_writeReg(0x0d, 0x03); // playback ctrl
    codec_writeReg(0x32, (1 << 7)); // vol
    codec_writeReg(0x32, (0 << 7)); // vol
    codec_writeReg(0x00, 0x00); // inits
    codec_writeReg(0x04, 0xaf); // power ctrl
    codec_writeReg(0x0d, 0x70);
    codec_writeReg(0x05, 0x81); // clocking: auto speed is determined by the MCLK/LRCK ratio.
    codec_writeReg(0x06, 0x07); // DAC interface format, I²S 16 bit
    codec_writeReg(0x0a, 0x00);
    codec_writeReg(0x27, 0x00);
    codec_writeReg(0x80, 0x0a); // both channels on
    codec_writeReg(0x1f, 0x0f);
    codec_writeReg(0x02, 0x9e);
#endif

    I2C_WriteReg(0x4A, 0x1E, 0xC0); /* beep */

    for (uint8_t i = 0; i < 15; i++)
    {
        uint8_t val;
        register_read(i, val);
        Serial.printf("0x%02x: 0x%02x\n", i, val);
    }

    STM32_AudioInit();
    // I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16);

#endif
}

#ifdef ARDUINO_GENERIC_F407VGTX

#define CODEC_ADDR 0x4A

void codec_writeReg(unsigned char reg, unsigned char data)
{
    uint8_t error;
    Wire.beginTransmission(CODEC_ADDR);
    Wire.write(reg);
    Wire.write(data);
    error = Wire.endTransmission();
}

void CS43L22_STM32_setVolume(uint8_t volumeValue)
{
    int8_t vol;
    if (volumeValue > 100)
    {
        volumeValue = 100;
    }
    //strange mapping, see datasheet
    //vol=25; // -102 dB
    //vol=24  // +12dB

    vol = -90 + (float)80 * volumeValue / 100;

    codec_writeReg(0x20, vol);
    codec_writeReg(0x21, vol);
}

enum codec_register
{
    init_sequence_reg1      = 0x0, // used only during init sequence
    chip_id                 = 0x01,
    pwr_ctrl1               = 0x02,
    pwr_ctrl2               = 0x04,
    clk_ctrl                = 0x05,
    if_ctrl1                = 0x06,
    if_ctrl2                = 0x07,
    passthrough_a_select    = 0x08,
    passthrough_b_select    = 0x09,
    analog_set              = 0x0A,
    passthrough_gang_ctrl   = 0x0C,
    playback_ctrl1          = 0x0D,
    misc_ctrl               = 0x0E,
    playback_ctrl2          = 0x0F,
    passthrough_a_vol       = 0x14,
    passthrough_b_vol       = 0x15,
    pcm_a_vol               = 0x1A,
    pcm_b_vol               = 0x1B,
    beep_freq_ontime        = 0x1C,
    beep_vol_offtime        = 0x1D,
    beep_tone_cfg           = 0x1E,
    tone_ctrl               = 0x1F,
    master_a_vol            = 0x20,
    master_b_vol            = 0x21,
    hp_a_vol                = 0x22,
    hp_b_vol                = 0x23,
    speak_a_vol             = 0x24,
    speak_b_vol             = 0x25,
    ch_mix_swap             = 0x26,
    limit_ctrl1             = 0x27,
    limit_ctrl2             = 0x28,
    limit_attack            = 0x29,
    ovfl_clk_status         = 0x2E,
    batt_comp               = 0x2F,
    vp_batt_level           = 0x30,
    speak_status            = 0x31,
    init_sequence_reg2      = 0x32, // used only during init sequence
    charge_pump_freq        = 0x34,
    init_sequence_reg3      = 0x47, // used only during init sequence
};

uint8_t I2C_ReadReg(uint8_t dev, uint8_t reg)
{
    Wire.beginTransmission(dev);
    Wire.write(reg); // set MCP23017 memory pointer to reg address
    Wire.endTransmission();

    Wire.requestFrom(dev, 1); // request one byte of data from MCP20317
    return Wire.read(); // store the incoming byte into "inputs"
}

void I2C_WriteReg(uint8_t dev, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(dev);
    Wire.write(reg); // IODIRA register
    Wire.write(value);
    Wire.endTransmission();
}

void register_write(uint8_t reg, uint8_t value)
{
    I2C_WriteReg(0x4A, reg, value);
}

// Reads internal codec register over I2C
void register_read(uint8_t reg, uint8_t &value)
{
    value = I2C_ReadReg(0x4A, reg);
}

void ecl_cs43l22_instance_power_up(void)
{
    uint8_t value;
    // power up sequence according to RM
    register_write(pwr_ctrl1, 0x01);
    //TODO #113 add error checking.
    register_write(init_sequence_reg1, 0x99);
    register_write(init_sequence_reg3, 0x80);

    register_read(init_sequence_reg2, value);
    register_write(init_sequence_reg2, value | 0x80);

    register_read(init_sequence_reg2, value);
    register_write(init_sequence_reg2, value & (~0x80));

    register_write(init_sequence_reg1, 0x0);

    // Turn ON headphone channel A and B
    register_write(pwr_ctrl2, 0xAF);

    // clock auto-detect
    register_write(clk_ctrl, 0x81);

    // Set Audio Word Length to 16 bit, Audio format - I2S.
    register_write(if_ctrl1, 0x7);

    // power up
    register_write(pwr_ctrl1, 0x9e);
}

void ecl_cs43l22_instance_set_master_volume(uint8_t volume)
{
    // Represents 0dB volume according to RM
    const uint8_t zero_level = 0xCC;
    // Represents -102dB volume according to RM
    const uint8_t lowest_level = 0x34;

    // Volume to bits mapping, see RM for details
    uint8_t bitmask = 0xFF;
    if (volume >= zero_level)
    {
        bitmask = volume - zero_level;
    }
    else
    {
        bitmask = volume + lowest_level;
    }

    // channel::all
    {
        register_write(master_a_vol, bitmask);

        register_write(master_b_vol, bitmask);
    }
}

void ecl_cs43l22_instance_set_headphone_volume(uint8_t volume)
{
    // Volume to bits mapping, see RM for details
    uint8_t bitmask = 0xFF;
#if 0
    if (volume < max_headphone_volume)
    {
        bitmask = volume + 1;
    }
    else
#endif
    {
        bitmask = 0x0;
    }

    // channel::all
    {
        register_write(hp_a_vol, bitmask);

        register_write(hp_b_vol, bitmask);
    }
}

void ecl_cs43l22_instance_headphone_unmute(void)
{
    uint8_t value = 0;
    register_read(playback_ctrl2, value);


    // Reset HP mute bits. See RM for details.
    // channel::all
    {
        value &= ~0xC0;
    }

    register_write(playback_ctrl2, value);
}

#endif

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
    pinMode(ledPin, OUTPUT);
    Midi_Setup();
}

#endif /* TEENSYDUINO */

#ifdef ARDUINO_DAISY_SEED

static DaisyHardware hw;
static size_t num_channels;

volatile static  bool dataReady = false;
static float out_temp[2][48];
static float *outCh[2] = {out_temp[0], out_temp[1]};

void MyCallback(float **in, float **out, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {

        out[0][i] = out_temp[0][i];
        out[1][i] = out_temp[1][i];
        out_temp[0][i] = in[0][i];
        out_temp[1][i] = in[1][i];
    }
    dataReady = true;
}

void DaisySeed_Setup(void)
{
    float sample_rate;
    // Initialize for Daisy pod at 48kHz
    hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
    num_channels = hw.num_channels;
    sample_rate = DAISY.get_samplerate();

    DAISY.begin(MyCallback);
}
#endif /* ARDUINO_DAISY_SEED */

#ifdef ARDUINO_SEEED_XIAO_M0

static int32_t u32buf[SAMPLE_BUFFER_SIZE];

inline
void ProcessAudio(uint16_t *buff, size_t len)
{
    /* convert from u16 to u10 */
    for (size_t i = 0; i < len; i++)
    {
        const int32_t preDiv = 4194304; // 2 ^ (16 + 6)
        buff[i] = (uint16_t)(0x200 + (u32buf[i] / (preDiv)));
    }
}

#endif

void Audio_OutputMono(int32_t *samples)
{
#ifdef ESP8266
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        int32_t sig = samples[i];
        static uint16_t sig16 = 0;
        sig *= 4;
        sig16 = sig;
        while (!I2S_isNotFull())
        {
            /* wait for buffer is not full */
        }
        writeDAC(0x8000 + sig16);
    }
#endif

#ifdef ESP32
    float mono[SAMPLE_BUFFER_SIZE];
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        float sigf = samples[i];
        sigf /= INT16_MAX;
        mono[i] = sigf;
    }

    i2s_write_stereo_samples_buff(mono, mono, SAMPLE_BUFFER_SIZE);
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
                for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
                {
                    sampleBuffer[i] = (int16_t)((samples[i]));
                }

                memcpy(queueTransmitBuffer, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
                memcpy(queueTransmitBuffer2, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
            }
            queue1.playBuffer();
            queue2.playBuffer();
        }
    }
#endif /* TEENSYDUINO */

#ifdef ARDUINO_DAISY_SEED

    float sig_f[SAMPLE_BUFFER_SIZE];

#ifdef CYCLE_MODULE_ENABLED
    calcCycleCountPre();
#endif
    while (!dataReady)
    {
        /* just do nothing */
    }
#ifdef CYCLE_MODULE_ENABLED
    calcCycleCount();
#endif

    for (size_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        sig_f[i] = ((float)samples[i]) * (1.0f / ((float)INT16_MAX));
    }

    memcpy(out_temp[0], sig_f, sizeof(out_temp[0]));
    memcpy(out_temp[1], sig_f, sizeof(out_temp[1]));

    dataReady = false;
#endif /* ARDUINO_DAISY_SEED */

#ifdef ARDUINO_SEEED_XIAO_M0
#ifdef CYCLE_MODULE_ENABLED
    calcCycleCountPre();
#endif
    while (!SAMD21_Synth_Process(ProcessAudio))
    {
        /* just do nothing */
    }
#ifdef CYCLE_MODULE_ENABLED
    calcCycleCount();
#endif
    memcpy(u32buf, samples, sizeof(int32_t)*SAMPLE_BUFFER_SIZE);
#endif /* ARDUINO_SEEED_XIAO_M0 */

#ifdef ARDUINO_RASPBERRY_PI_PICO
    /*
     * @see https://arduino-pico.readthedocs.io/en/latest/i2s.html
     * @see https://www.waveshare.com/pico-audio.htm for connections
     */
    int16_t u16int[2 * SAMPLE_BUFFER_SIZE];

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        u16int[2 * i] = samples[i];
        u16int[(2 * i) + 1] = samples[i];
    }
#if 1
    for (int i = 0; i < SAMPLE_BUFFER_SIZE * 2; i++)
    {
        I2S.write(u16int[i]);
    }
#else
    /* this does not work, I do not know why :-/ */
    static int16_t u16int_buf[2 * SAMPLE_BUFFER_SIZE];
    memcpy(u16int_buf, u16int, sizeof(u16int));
    I2S.write(u16int_buf, sizeof(u16int));
#endif
#endif /* ARDUINO_RASPBERRY_PI_PICO */

#ifdef ARDUINO_GENERIC_F407VGTX
    /*
     * Todo Implementation for the STM32F407VGT6
     * Can be found on the ST Discovery Board
     */
#if 0
    int16_t u16int[2 * SAMPLE_BUFFER_SIZE];

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        u16int[2 * i] = samples[i];
        u16int[(2 * i) + 1] = samples[i];
    }
    for (int i = 0; i < SAMPLE_BUFFER_SIZE * 2; i++)
    {
        I2S.write(u16int[i]);
    }
#endif
#endif
}

#if (defined ESP32) || (defined TEENSYDUINO) || (defined ARDUINO_DAISY_SEED) || (defined ARDUINO_GENERIC_F407VGTX)
void Audio_Output(float *left, float *right)
{
#ifdef ESP32
    i2s_write_stereo_samples_buff(left, right, SAMPLE_BUFFER_SIZE);
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
                for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
                {
                    sampleBuffer[i] = (int16_t)(left[i] * INT16_MAX);
                    sampleBuffer2[i] = (int16_t)(right[i] * INT16_MAX);
                }

                memcpy(queueTransmitBuffer, sampleBuffer, AUDIO_BLOCK_SAMPLES * 2);
                memcpy(queueTransmitBuffer2, sampleBuffer2, AUDIO_BLOCK_SAMPLES * 2);
            }

            queue1.playBuffer();
            queue2.playBuffer();
        }
    }
#endif /* TEENSYDUINO */

#ifdef ARDUINO_DAISY_SEED

#ifdef CYCLE_MODULE_ENABLED
    calcCycleCountPre();
#endif
    while (!dataReady)
    {
        /* just do nothing */
    }
#ifdef CYCLE_MODULE_ENABLED
    calcCycleCount();
#endif

#if 0
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        left[i] = i * (1.0f / ((float)SAMPLE_BUFFER_SIZE));
        right[i] = i * (1.0f / ((float)SAMPLE_BUFFER_SIZE));
    }
#endif
    memcpy(out_temp[0], left, sizeof(out_temp[0]));
    memcpy(out_temp[1], right, sizeof(out_temp[1]));


    dataReady = false;

#endif /* ARDUINO_DAISY_SEED */

#ifdef ARDUINO_GENERIC_F407VGTX
    /*
     * Todo Implementation for the STM32F407VGT6
     * Can be found on the ST Discovery Board
     */
#endif
}
#endif /* (defined ESP32) || (defined TEENSYDUINO) || (defined ARDUINO_DAISY_SEED) || (defined ARDUINO_GENERIC_F407VGTX) */

