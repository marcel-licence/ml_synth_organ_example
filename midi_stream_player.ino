/*
 * Copyright (c) 2022 Marcel Licence
 */

/*
 * sd_module.ino
 *
 *  Created on: 22.05.2022
 *      Author: PC
 */

#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


#ifdef MIDI_STEAM_PLAYER_ENABLED


#ifdef ARDUINO_DAISY_SEED
#include <STM32SD.h>

extern Sd2Card card;
extern SdFatFs fatFs;

#define SD_MMC  SD

#define FST SDClass
#else
#define FST fs::FS
#endif

#define MIDI_FS_LITTLE_FS	0
#define MIDI_FS_SD_MMC	1


#define FORMAT_LITTLEFS_IF_FAILED true


#include <FS.h>
#include <LITTLEFS.h>
#include <SD_MMC.h>

#include "ml_midi_file_stream.h"


static void listDir(FST &fs, const char *dirname, uint8_t levels);


struct file_access_f mdiCallbacks =
{
    MIDI_open,
    MIDI_read,
    MIDI_write,
    MIDI_close,
    0,
    MIDI_getc,
    MIDI_putc,
    MIDI_tell,
    MIDI_seek,
};

fs::File midiFile;

uint8_t MIDI_open(const char *path, const char *mode)
{
    if (!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED))
    {
        Serial.println("LITTLEFS Mount Failed");
        return 0;
    }
    midiFile = LITTLEFS.open(path);
    if (!midiFile)
    {
        Serial.println("- failed to open file");
        return 0;
    }
    else
    {
        Serial.printf("File opened: %s\n", path);
    }

    return 1;
}

int MIDI_read(void *buf, uint8_t unused, size_t size, struct file_access_f *ff)
{
    File *file = &midiFile;//ff->file;
    for (int i = 0; i < size; i++)
    {
        ((uint8_t *)buf)[i] = file->read();
        ff->file ++;
    }
    return size;
}

int MIDI_write(void *buf, uint8_t unused, size_t size, struct file_access_f *ff)
{
    return 0;
}

void MIDI_close(struct file_access_f *ff)
{
    File *file = &midiFile;//ff->file;
    file->close();
}

char MIDI_getc(struct file_access_f *ff)
{
    File file = midiFile;//ff->file;
    return file.read();
}

char MIDI_putc(char c, struct file_access_f *ff)
{
    return 0;
}

int MIDI_tell(struct file_access_f *ff)
{
#if 0
    File *file = &midiFile;//ff->file;
    return file->size();
#else
    return ff->file - 1;
#endif
}

char MIDI_seek(struct file_access_f *ff, int pos, uint8_t mode)
{
    File *file = &midiFile;//ff->file;
    if (mode == SEEK_SET)
    {
        file->seek(pos, SeekSet);
        ff->file = pos + 1;
    }
    else
    {
        file->seek(pos, SeekCur);
        ff->file += pos;
    }
    //ff->file = file->size() - pos;
    return 0;
}

void MidiStreamPlayer_Init()
{
    MidiStreamPlayer_ListFiles(MIDI_FS_LITTLE_FS);
    MidiStreamPlayer_ListFiles(MIDI_FS_SD_MMC);
}

void MidiStreamPlayer_PlayFile(char *midi_filename)
{
    MidiStreamPlayer_PlayMidiFile_fromLittleFS(midi_filename, NULL);
}

static void listDir(FST &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

static bool midiPlaying = false;

void MidiDataCallback(uint8_t *data, uint8_t data_len)
{
    printf("d:");
    for (uint8_t n; n < data_len; n++)
    {
        printf(" %02x", data[n]);
    }
    printf("\n");
}

long duration = 0;
struct midi_proc_s midi;

void MidiStreamPlayer_NoteOn(uint8_t ch, uint8_t note, uint8_t vel)
{
    Midi_NoteOn(ch, note, vel);
}

void MidiStreamPlayer_NoteOff(uint8_t ch, uint8_t note, uint8_t vel)
{
    Midi_NoteOff(ch, note);
}

void MidiStreamPlayer_ControlChange(uint8_t ch, uint8_t number, uint8_t value)
{
    Midi_ControlChange(ch, number, value);
}

void MidiStreamPlayer_PlayMidiFile_fromLittleFS(char *filename, uint8_t trackToPlay)
{
    midi.raw = MidiDataCallback;
    midi.noteOn = MidiStreamPlayer_NoteOn;
    midi.noteOff = MidiStreamPlayer_NoteOff;
    midi.controlChange = MidiStreamPlayer_ControlChange;
    midi.ff = &mdiCallbacks;

    midi_file_stream_load(filename, &midi);

    for (uint8_t n = 0; n < trackToPlay; n++)
    {
        MidiStreamSkipTrack(&midi);
    }

    MidiStreamReadTrackPrepare(&midi);

    midiPlaying = true;
    duration = 0;

    MidiStreamReadSingleEventTime(&midi, &duration);
    duration *= SAMPLE_RATE;
}

void MidiStreamPlayer_Tick(long ticks)
{
    static long tickCnt = 0;

    if (midiPlaying)
    {
        tickCnt += ticks * 1500;

        while ((tickCnt > duration) && midiPlaying)
        {
            tickCnt -= duration;

            midiPlaying &= MidiStreamReadSingleEvent(&midi);

            midiPlaying &= MidiStreamReadSingleEventTime(&midi, &duration);
            duration *= SAMPLE_RATE;
        }
    }
}

void MidiStreamPlayer_ListFiles(uint8_t filesystem)
{
    switch (filesystem)
    {
    case MIDI_FS_LITTLE_FS:
        {
            if (!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED))
            {
                Serial.println("LITTLEFS Mount Failed");
                return;
            }
            listDir(LITTLEFS, "/", 3);
            break;
        }
    case MIDI_FS_SD_MMC:
        {
            if (!SD_MMC.begin())
            {
                Serial.println("Card Mount Failed");
                return;
            }
            uint8_t cardType = SD_MMC.cardType();

            if (cardType == CARD_NONE)
            {
                Serial.println("No SD_MMC card attached");
                return;
            }

            Serial.print("SD_MMC Card Type: ");
            if (cardType == CARD_MMC)
            {
                Serial.println("MMC");
            }
            else if (cardType == CARD_SD)
            {
                Serial.println("SDSC");
            }
            else if (cardType == CARD_SDHC)
            {
                Serial.println("SDHC");
            }
            else
            {
                Serial.println("UNKNOWN");
            }

            uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
            Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

            listDir(SD_MMC, "/", 0);
        }
        break;
    }
}

#endif

