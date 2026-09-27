/* Host harness for auto_trigger.c (not part of the firmware image).
 *
 *   auto_trigger_host <dump> end_lo end_hi n_frames z_min_x10 z_cap_x10
 *                     slope_min_q slope_max_q delay_frames
 *
 * Pushes every frame of a timed capture, in dump order, as stored (IQ8 codes
 * without their scale, or IQ16 values) and prints one line per frame:
 *
 *   frame result has_detection det_frame det_end det_slope_q det_score
 *
 * tests/test_iwr6843_auto_trigger_firmware.py compares it with autotrigger.py.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../auto_trigger.h"

#define HEADER_BYTES      20U
#define TEMPERATURE_BYTES 24U
#define FMT_IQ16_TIMED    4U
#define FMT_IQ8_TIMED     5U
#define MAX_DUMP_FRAMES   64U
#define MAX_FRAME_VALUES  (2U * AT_MAX_LOOPS * 3U * 4U * AT_MAX_WINDOW_BINS)

static at_state_t g_state;
static int16_t g_frame[MAX_FRAME_VALUES];

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint8_t *read_file(const char *path, size_t *size)
{
    FILE *file = fopen(path, "rb");
    uint8_t *data;
    long length;

    if (file == NULL || fseek(file, 0, SEEK_END) != 0 || (length = ftell(file)) < 0 ||
        fseek(file, 0, SEEK_SET) != 0) {
        return NULL;
    }
    data = (uint8_t *)malloc((size_t)length);
    if (data == NULL || fread(data, 1U, (size_t)length, file) != (size_t)length) {
        return NULL;
    }
    fclose(file);
    *size = (size_t)length;
    return data;
}

static int fail(const char *message)
{
    fprintf(stderr, "auto_trigger_host: %s\n", message);
    return 2;
}

int main(int argc, char **argv)
{
    static uint8_t bin_start[MAX_DUMP_FRAMES], bin_count[MAX_DUMP_FRAMES];
    at_config_t cfg;
    uint8_t *raw;
    size_t size, offset;
    uint16_t version, n_frames, chirps, frame;
    uint8_t fmt, n_tx, n_rx, iq8;

    if (argc != 10) {
        return fail("usage: <dump> end_lo end_hi n_frames z_min_x10 z_cap_x10 "
                    "slope_min_q slope_max_q delay_frames");
    }
    cfg.end_lo = (uint8_t)atoi(argv[2]);
    cfg.end_hi = (uint8_t)atoi(argv[3]);
    cfg.n_frames = (uint8_t)atoi(argv[4]);
    cfg.z_min_x10 = (uint32_t)atol(argv[5]);
    cfg.z_cap_x10 = (uint32_t)atol(argv[6]);
    cfg.slope_min_q = (uint8_t)atoi(argv[7]);
    cfg.slope_max_q = (uint8_t)atoi(argv[8]);
    cfg.delay_frames = (uint8_t)atoi(argv[9]);
    if (at_check_config(&cfg) != AT_OK) {
        return fail("bad configuration");
    }

    raw = read_file(argv[1], &size);
    if (raw == NULL || size < HEADER_BYTES || memcmp(raw, "ILD1", 4) != 0) {
        return fail("unreadable dump");
    }
    version = u16(raw + 4);
    n_frames = u16(raw + 6);
    chirps = u16(raw + 8);
    n_tx = raw[10];
    n_rx = raw[11];
    fmt = raw[14];
    if (fmt != FMT_IQ16_TIMED && fmt != FMT_IQ8_TIMED) {
        return fail("not a timed capture");
    }
    iq8 = (uint8_t)(fmt == FMT_IQ8_TIMED);
    if (n_frames == 0U || n_frames > MAX_DUMP_FRAMES) {
        return fail("unsupported frame count");
    }
    offset = HEADER_BYTES + ((version == 7U) ? TEMPERATURE_BYTES : 0U);
    for (frame = 0U; frame < n_frames; frame++, offset += 4U) {
        bin_start[frame] = raw[offset];
        bin_count[frame] = raw[offset + 1U];
    }
    if (iq8) {
        offset += 2U * n_frames; /* per-frame scales: normalisation cancels them */
    }

    at_reset(&g_state);
    for (frame = 0U; frame < n_frames; frame++) {
        uint32_t values = 2U * (uint32_t)chirps * n_rx * bin_count[frame];
        uint32_t i;
        int32_t result;

        if (values > MAX_FRAME_VALUES || offset + values * (iq8 ? 1U : 2U) > size) {
            return fail("frame does not fit");
        }
        for (i = 0U; i < values; i++) {
            g_frame[i] = iq8 ? (int16_t)(int8_t)raw[offset + i]
                             : (int16_t)u16(raw + offset + 2U * i);
        }
        offset += values * (iq8 ? 1U : 2U);
        result = at_push_frame(&g_state, &cfg, g_frame, chirps, n_tx, n_rx,
                               bin_start[frame], bin_count[frame]);
        if (result < 0) {
            fprintf(stderr, "auto_trigger_host: frame %u error %d\n", (unsigned)frame,
                    (int)result);
            return 2;
        }
        printf("%u %d %u %lu %u %u %.17g\n", (unsigned)frame, (int)result,
               (unsigned)g_state.has_detection, (unsigned long)g_state.det_frame,
               (unsigned)g_state.det_end, (unsigned)g_state.det_slope_q,
               g_state.det_score);
    }
    return 0;
}
