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
 * @file midi_interface.ino
 * @author Marcel Licence
 * @date 04.10.2021
 *
 * @brief This file contains an implementation of a simple MIDI interpreter to parse incoming messages
 *
 * MIDI_DUMP_Serial1_TO_SERIAL <- when active received data will be output as hex on serial(1)
 * MIDI_SERIAL1_BAUDRATE <- use define to override baud-rate for MIDI, otherwise default of 31250 will be used
 *
 * @see https://www.midi.org/specifications-old/item/table-1-summary-of-midi-message
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


/*
 * look for midi interface using 1N136
 * to convert the MIDI din signal to
 * a uart compatible signal
 */

#ifndef MIDI_SERIAL1_BAUDRATE
#define MIDI_SERIAL1_BAUDRATE   31250
#endif

#ifndef MIDI_SERIAL2_BAUDRATE
#define MIDI_SERIAL2_BAUDRATE   31250
#endif

/* use define to dump midi data */
//#define MIDI_DUMP_SERIAL2_TO_SERIAL

/*
 * structure is used to build the mapping table
 */
struct midiControllerMapping
{
    uint8_t channel;
    uint8_t data1;
    const char *desc;
    void(*callback_mid)(uint8_t ch, uint8_t data1, uint8_t data2);
#ifdef MIDI_FMT_INT
    void(*callback_val)(uint8_t userdata, uint8_t value);
#else
    void(*callback_val)(uint8_t userdata, float value);
#endif
    uint8_t user_data;
};

struct midiMapping_s
{
    void (*rawMsg)(uint8_t *msg);
#ifdef MIDI_FMT_INT
    void (*noteOn)(uint8_t ch, uint8_t note, uint8_t vel);
    void (*noteOff)(uint8_t ch, uint8_t note);
    void (*pitchBend)(uint8_t ch, uint8_t bend);
    void (*modWheel)(uint8_t ch, uint8_t value);
#else
    void (*noteOn)(uint8_t ch, uint8_t note, float vel);
    void (*noteOff)(uint8_t ch, uint8_t note);
    void (*pitchBend)(uint8_t ch, float bend);
    void (*modWheel)(uint8_t ch, float value);
#endif

    struct midiControllerMapping *controlMapping;
    int mapSize;
};

extern struct midiMapping_s midiMapping; /* definition in z_config.ino */

/* constant to normalize midi value to 0.0 - 1.0f */
#define NORM127MUL  0.007874f

inline void Midi_NoteOn(uint8_t ch, uint8_t note, uint8_t vel)
{
    if (vel > 127)
    {
        /* we will end up here in case of problems with the MIDI connection */
        vel = 127;
        Serial.printf("to loud note detected!!!!!!!!!!!!!!!!!!!!!!!\n");
    }

    if (midiMapping.noteOn != NULL)
    {
#ifdef MIDI_FMT_INT
        midiMapping.noteOn(ch, note, vel);
#else
        midiMapping.noteOn(ch, note, pow(2, ((vel * NORM127MUL) - 1.0f) * 6));
#endif
    }
}

inline void Midi_NoteOff(uint8_t ch, uint8_t note)
{
    if (midiMapping.noteOff != NULL)
    {
        midiMapping.noteOff(ch, note);
    }
}

/*
 * this function will be called when a control change message has been received
 */
inline void Midi_ControlChange(uint8_t channel, uint8_t data1, uint8_t data2)
{
    for (int i = 0; i < midiMapping.mapSize; i++)
    {
        if ((midiMapping.controlMapping[i].channel == channel) && (midiMapping.controlMapping[i].data1 == data1))
        {
            if (midiMapping.controlMapping[i].callback_mid != NULL)
            {
                midiMapping.controlMapping[i].callback_mid(channel, data1, data2);
            }
            if (midiMapping.controlMapping[i].callback_val != NULL)
            {
#ifdef MIDI_FMT_INT
                midiMapping.controlMapping[i].callback_val(midiMapping.controlMapping[i].user_data, data2);
#else
                midiMapping.controlMapping[i].callback_val(midiMapping.controlMapping[i].user_data, (float)data2 * NORM127MUL);
#endif
            }
        }
    }

    if (data1 == 1)
    {
        if (midiMapping.modWheel != NULL)
        {
#ifdef MIDI_FMT_INT
            midiMapping.modWheel(channel, data2);
#else
            midiMapping.modWheel(channel, (float)data2 * NORM127MUL);
#endif
        }
    }
}

inline void Midi_PitchBend(uint8_t ch, uint16_t bend)
{
    float value = ((float)bend - 8192.0f) * (1.0f / 8192.0f) - 1.0f;
    if (midiMapping.pitchBend != NULL)
    {
        midiMapping.pitchBend(ch, value);
    }
}

/*
 * function will be called when a short message has been received over midi
 */
inline void Midi_HandleShortMsg(uint8_t *data, uint8_t cable __attribute__((unused)))
{
    uint8_t ch = data[0] & 0x0F;

    switch (data[0] & 0xF0)
    {
    /* note on */
    case 0x90:
        if (data[2] > 0)
        {
            Midi_NoteOn(ch, data[1], data[2]);
        }
        else
        {
            Midi_NoteOff(ch, data[1]);
        }
        break;
    /* note off */
    case 0x80:
        Midi_NoteOff(ch, data[1]);
        break;
    case 0xb0:
        Midi_ControlChange(ch, data[1], data[2]);
        break;
    /* pitchbend */
    case 0xe0:
        Midi_PitchBend(ch, ((((uint16_t)data[1])) + ((uint16_t)data[2] << 8)));
        break;
    }
}

void Midi_Setup()
{
#ifdef TEENSYDUINO
    Serial1.begin(MIDI_SERIAL1_BAUDRATE);
#else
#ifdef MIDI_RX_PIN
#ifdef MIDI_TX_PIN
    Serial2.begin(MIDI_SERIAL2_BAUDRATE, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);
#else
    Serial2.begin(MIDI_SERIAL2_BAUDRATE, SERIAL_8N1, MIDI_RX_PIN);
#endif
    pinMode(MIDI_RX_PIN, INPUT_PULLUP); /* can be connected to open collector output */
#endif
#endif
}

void setup_Serial2()
{
#if 1
    //Serial2.begin(31250, SERIAL_8N1, MIDI_IN, TXD2);
#ifdef ESP8266
#ifdef SWAP_SERIAL
    Serial.printf("Switch Serial to Midi!\n");
    delay(20);
    Serial.end();
    Serial.begin(MIDI_BAUDRATE);
    Serial.swap();
    delay(20);
#else
    Serial2.begin(MIDI_BAUDRATE);
#endif
#else
    Serial2.begin(MIDI_BAUDRATE);
#endif
#endif
}

void Midi_CheckSerial2(void)
{
    /*
     * watchdog to avoid getting stuck by receiving incomplete or wrong data
     */
    static uint32_t inMsgWd = 0;
    static uint8_t inMsg[3];
    static uint8_t inMsgIndex = 0;

    //Choose Serial1 or Serial2 as required

#ifdef SWAP_SERIAL
    if (Serial.available())
#else
    if (Serial2.available())
#endif
    {
#ifdef SWAP_SERIAL
        uint8_t incomingByte = Serial.read();
#else
        uint8_t incomingByte = Serial2.read();
#endif

#ifdef SWAP_SERIAL
        Serial.write(incomingByte);
#else
        Serial.printf("%02x", incomingByte);
#endif
        /* ignore live messages */
        if ((incomingByte & 0xF0) == 0xF0)
        {
            return;
        }

        if (inMsgIndex == 0)
        {
            if ((incomingByte & 0x80) != 0x80)
            {
                inMsgIndex = 1;
            }
        }

        inMsg[inMsgIndex] = incomingByte;
        inMsgIndex += 1;

        if ((inMsgIndex >= 3) || (((inMsg[0] == 0xD0) || (inMsg[0] == 0xC0)) && inMsgIndex >= 2))
        {
#ifndef SWAP_SERIAL
            Serial.printf(">%02x %02x %02x\n", inMsg[0], inMsg[1], inMsg[2]);
#endif
            Midi_HandleShortMsg(inMsg, 0);
            inMsgIndex = 0;
        }

        /*
         * reset watchdog to allow new bytes to be received
         */
        inMsgWd = 0;
    }
    else
    {
        if (inMsgIndex > 0)
        {
            inMsgWd++;
            if (inMsgWd == 0xFFF)
            {
                inMsgIndex = 0;
            }
        }
    }
}

inline
void Midi_CheckSerial(HardwareSerial *ser)
{
    /*
     * watchdog to avoid getting stuck by receiving incomplete or wrong data
     */
    static uint32_t inMsgWd = 0;
    static uint8_t inMsg[3];
    static uint8_t inMsgIndex = 0;

    //Choose Serial1 or Serial2 as required

    if (ser->available())
    {
        while (ser->available())
        {
            uint8_t incomingByte = ser->read();

#ifdef SWAP_SERIAL
            ser->write(incomingByte);
#else
            ser->printf("%02x", incomingByte);
#endif
            /* ignore live messages */
            if ((incomingByte & 0xF0) == 0xF0)
            {
                return;
            }

            if (inMsgIndex == 0)
            {
                if ((incomingByte & 0x80) != 0x80)
                {
                    inMsgIndex = 1;
                }
            }

            inMsg[inMsgIndex] = incomingByte;
            inMsgIndex += 1;

            if ((inMsgIndex >= 3) || (((inMsg[0] == 0xD0) || (inMsg[0] == 0xC0)) && inMsgIndex >= 2))
            {
#ifndef SWAP_SERIAL
                ser->printf(">%02x %02x %02x\n", inMsg[0], inMsg[1], inMsg[2]);
#endif
                Midi_HandleShortMsg(inMsg, 0);
                inMsgIndex = 0;
            }

            /*
             * reset watchdog to allow new bytes to be received
             */
#if 0
            ser->print("I received: ");
            ser->println(incomingByte, DEC);
#endif
            inMsgWd = 0;
        }
    }
    else
    {
        if (inMsgIndex > 0)
        {
            inMsgWd++;
            if (inMsgWd == 0xFFF)
            {
                inMsgIndex = 0;
            }
        }
    }
}

/*
 * this function should be called continuously to ensure that incoming messages can be processed
 */
inline
void Midi_Process()
{
#ifdef MIDI_RX_PIN
    Midi_CheckSerial2();
#endif
#ifdef MIDI_RECV_FROM_SERIAL
    Midi_CheckSerial(&Serial);
#endif
#ifdef TEENSYDUINO
    Midi_CheckSerial(&Serial1);
#endif
}

#ifndef SWAP_SERIAL
void Midi_SendShortMessage(uint8_t *msg)
{
    Serial2.write(msg, 3);
}

void Midi_SendRaw(uint8_t *msg)
{
    /* sysex */
    if (msg[0] == 0xF0)
    {
        int i = 2;
        while (msg[i] != 0xF7)
        {
            i++;
        }
        Serial2.write(msg, i + 1);
    }
    else
    {
        Serial2.write(msg, 3);
    }
}
#endif
