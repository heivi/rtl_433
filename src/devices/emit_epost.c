/** @file
    Emit ePost decoder

    Copyright (C) 2022 Heikki Virekunnas <heikki@virekunnas.fi>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/** @fn int emit_epost_decode(r_device *decoder, bitbuffer_t *bitbuffer)
Emit ePost decoder.

Note: work in progress, using CC1100 like marlec_solar, Archos-TBM.

- Modulation: FSK PCM
- Frequency: 868.000 - 868.800 MHz depending on the ePost code
- 104 us symbol/bit time
- based on TI CC1100

Payload format:
- Preamble          {32} 0xaaaaaaaa
- Syncword          {32} 0xd391d391
- Payload           {n}
- Checksum          {16} CRC16 poly=0x8005 init=0xffff

The application data is obfuscated/whitened by doing data[n] xor whitening[n].

Payload data format: 2h SENDNO: 2d 4d 8h EMITNO: <32d EPOSTCODE: 8d TIMEMS: <16d OVERFLOWS: 8d 8h 8h

Data layout:
    FF MM RRRR FFFFFFFF NNNNNNNN NNNNNNNN NNNNNNNN NNNNNNNN EEEEEEEE MMMMMMMM MMMMMMMM OOOOOOOO CCCCCCCC CCCCCCCC

- F: unknown
- M: send no 0-3?
- R: unknown, value 15 if resent, not real-time punch (sent about every 512s for some time)
- N: 24/32? bit little-endian Emit card number
- E: 8-bit Emit ePost punch unit number
- M: 16-bit little-endian milliseconds time
- O: 8-bit number of 16-bit millisecond overflows -> 65536*O + M => Emit time in ms
- C: 16-bit CRC-16, poly 0x8005, init 0xFFFF (TI DN502)

Test with ./rtl_433 -f 868.355M -s 250k -v -R 246 -g 30

*/

#include "decoder.h"
#include <time.h>

// TI Design Note DN509 - whitening PN9 generator
uint8_t whitening[18] = {0xff, 0xe1, 0x1d, 0x9a, 0xed, 0x85, 0x33, 0x24, 0xea,
        0x7a, 0xd2, 0x39, 0x70, 0x97, 0x57, 0x0a, 0x54, 0x7d};

static int emit_epost_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{

    struct timespec tms;
    unsigned long long millis = 0;

    /* POSIX.1-2008 way */
    if (clock_gettime(CLOCK_REALTIME, &tms)) {
        return -1;
    }

    /* seconds, multiplied to millisecs */
    millis = tms.tv_sec * 1000;
    /* Add full milliseconds */
    millis += tms.tv_nsec / 1000000;
    /* round up if necessary */
    if (tms.tv_nsec % 1000000 >= 500) {
        ++millis;
    }
    char timestamp[20];
    sprintf(timestamp, "%llu", millis);

    uint8_t const preamble[] = {
            /*0xaa, 0xaa, */ 0xaa, 0xaa, // preamble
            0xd3, 0x91, 0xd3, 0x91       // sync word
    };

    data_t *data;

    if (bitbuffer->num_rows < 1) {
        decoder_logf(decoder, 0, __func__, "ePost: Not 1 row\n");
        return DECODE_ABORT_EARLY;
    }

    int row = 0;
    // Validate message and reject it as fast as possible : check for preamble
    unsigned start_pos = bitbuffer_search(bitbuffer, row, 0, preamble, sizeof(preamble) * 8);

    if (start_pos == bitbuffer->bits_per_row[row]) {
        decoder_logf(decoder, 2, __func__, "ePost: No preamble detected\n");
        return DECODE_ABORT_EARLY; // no preamble detected
    }

    // check min length
    if (bitbuffer->bits_per_row[row] < (10 + 8) * 8) { // sync(4) + preamble(4) + data(12)
        decoder_logf(decoder, 0, __func__, "ePost: Min length failed\n");
        return DECODE_ABORT_LENGTH;
    }

    uint8_t frame[20] = {0}; // TODO check max size
    // Get frame
    bitbuffer_extract_bytes(bitbuffer, row,
            start_pos + (sizeof(preamble)) * 8,
            &frame[0], 12 * 8);

    char nonw_frame_str[16 * 2 + 1] = {0};
    for (int i = 0; i < 12; ++i)
        sprintf(&nonw_frame_str[i * 2], "%02x", frame[i]);

    // data whitening
    for (unsigned i = 0; i < sizeof(frame); ++i) {
        frame[i] ^= whitening[i];
    }

    uint8_t msgno = ((frame[0] & 0x30) >> 4);

    if (msgno > 3) {
        decoder_logf(decoder, 1, __func__, "Message no. too large (%d), drop it\n", msgno);
        return DECODE_FAIL_SANITY;
    }

    uint32_t emitcode = frame[5] << 24 | frame[4] << 16 | frame[3] << 8 | frame[2];

    uint8_t epostcode = frame[6];

    uint16_t time_ms       = frame[7] | frame[8] << 8;
    uint8_t time_overflows = frame[9];

    uint32_t combitime_ms = time_overflows * 65536 + time_ms;
    uint16_t ms           = combitime_ms % 1000;
    uint16_t secs         = (combitime_ms - ms) / 1000 % 60;
    uint16_t mins         = ((combitime_ms - ms) / 1000) / 60;

    uint8_t resend = (frame[0] & 0x0F) == 15;

    uint16_t crc = crc16(frame, 10, 0x8005, 0xffff);

    if ((frame[9 + 1] << 8 | frame[9 + 2]) != crc) {
        decoder_logf(decoder, 1, __func__, "CRC invalid %04x != %04x\n", frame[10 + 1] << 8 | frame[10 + 2], crc);
        return DECODE_FAIL_MIC;
    }

    char frame_str[16 * 2 + 1] = {0};
    for (int i = 0; i < 12; ++i)
        sprintf(&frame_str[i * 2], "%02x", frame[i]);

    /* clang-format off */
    data = data_make(
            "model",        "",                   DATA_STRING, "Emit-ePost",
            "raw",          "Raw data",           DATA_STRING, frame_str,
            "nonw_raw",     "Whitened raw data",  DATA_STRING, nonw_frame_str,
            "emitcode",     "Emit card code",     DATA_INT,    emitcode,
            "epostcode",    "Emit ePost code",    DATA_INT,    epostcode,
            "timemins",     "Emit time minutes",  DATA_INT,    mins,
            "timesecs",     "Emit time secs",     DATA_INT,    secs,
            "timems",       "Emit time millisecs",DATA_INT,    ms,
            "resend",       "Resent data",        DATA_INT,    resend,
            // TODO: check time format for contribution
            "time",         "Received time",      DATA_STRING, timestamp,
            "mic",          "Integrity",          DATA_STRING, "CRC",
            NULL);
    /* clang-format on */
    decoder_output_data(decoder, data);
    return 1;
}

static char *output_fields[] = {
        "model",
        "raw",
        "emitcode",
        "epostcode",
        "timemins",
        "timesecs",
        "timems",
        "resend",
        "time",
        "mic",
        NULL,
};

r_device emit_epost = {
        .name        = "Emit ePost",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 104,
        .long_width  = 104,
        .reset_limit = 5000,
        .decode_fn   = &emit_epost_decode,
        //.disabled    = 1,
        .fields = output_fields,
};
