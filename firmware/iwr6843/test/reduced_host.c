/* Host harness for reduced_overview.c (not part of the firmware image).
 *
 *   reduced_host overview <dump> <gate_lo> <gate_hi>   overview bytes -> stdout
 *   reduced_host strips   <dump> <hex request>          strip bytes    -> stdout
 *
 * <dump> is a full timed capture as l3dump streams it. The output starts with
 * the dump's own header and temperature report, as the firmware's does.
 * tests/test_iwr6843_reduced_firmware.py compares it with reduced.py.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../reduced_overview.h"

#define HEADER_BYTES      20U
#define TEMPERATURE_BYTES 24U
#define FMT_IQ16_TIMED    4U
#define FMT_IQ8_TIMED     5U

static ro_work_t g_work;

static void sink_stdout(void *ctx, const uint8_t *bytes, uint32_t count)
{
    (void)ctx;
    if (fwrite(bytes, 1U, count, stdout) != count) {
        exit(3);
    }
}

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
    fprintf(stderr, "reduced_host: %s\n", message);
    return 2;
}

int main(int argc, char **argv)
{
    static uint8_t bin_start[RO_MAX_FRAMES], bin_count[RO_MAX_FRAMES];
    static uint16_t delta_us[RO_MAX_FRAMES], scale[RO_MAX_FRAMES];
    static const uint8_t *frames[RO_MAX_FRAMES];
    static ro_window_t request[RO_MAX_FRAMES];
    ro_capture_t cap;
    uint8_t *raw;
    size_t size, prefix, offset;
    uint16_t version, frame;
    uint8_t fmt;
    int32_t status;

    if (argc != 4 && argc != 5) {
        return fail("usage: overview <dump> <lo> <hi> | strips <dump> <hex>");
    }
    raw = read_file(argv[2], &size);
    if (raw == NULL || size < HEADER_BYTES || memcmp(raw, "ILD1", 4) != 0) {
        return fail("unreadable dump");
    }
    version = u16(raw + 4);
    fmt = raw[14];
    if (fmt != FMT_IQ16_TIMED && fmt != FMT_IQ8_TIMED) {
        return fail("not a timed capture");
    }
    memset(&cap, 0, sizeof(cap));
    cap.n_frames = u16(raw + 6);
    cap.chirps_per_frame = u16(raw + 8);
    cap.n_tx = raw[10];
    cap.n_rx = raw[11];
    cap.iq8 = (uint8_t)(fmt == FMT_IQ8_TIMED);
    if (cap.n_frames == 0U || cap.n_frames > RO_MAX_FRAMES) {
        return fail("unsupported frame count");
    }
    prefix = HEADER_BYTES + ((version == 7U) ? TEMPERATURE_BYTES : 0U);
    offset = prefix;
    for (frame = 0U; frame < cap.n_frames; frame++, offset += 4U) {
        bin_start[frame] = raw[offset];
        bin_count[frame] = raw[offset + 1U];
        delta_us[frame] = u16(raw + offset + 2U);
    }
    if (cap.iq8) {
        for (frame = 0U; frame < cap.n_frames; frame++, offset += 2U) {
            scale[frame] = u16(raw + offset);
        }
    }
    for (frame = 0U; frame < cap.n_frames; frame++) {
        frames[frame] = raw + offset;
        offset += (size_t)bin_count[frame] * cap.chirps_per_frame * cap.n_rx *
                  (cap.iq8 ? 2U : 4U);
    }
    if (offset != size) {
        return fail("dump size does not match its frame tables");
    }
    cap.bin_start = bin_start;
    cap.bin_count = bin_count;
    cap.delta_us = delta_us;
    cap.iq8_scale = scale;
    cap.frame = frames;

    sink_stdout(NULL, raw, (uint32_t)prefix);
    if (strcmp(argv[1], "overview") == 0 && argc == 5) {
        status = ro_write_overview(&cap, (uint16_t)atoi(argv[3]), (uint16_t)atoi(argv[4]),
                                   &g_work, sink_stdout, NULL);
    } else if (strcmp(argv[1], "strips") == 0 && argc == 4) {
        status = ro_parse_strip_request(argv[3], cap.n_frames, request);
        if (status == RO_OK) {
            status = ro_write_strips(&cap, request, sink_stdout, NULL);
        }
    } else {
        return fail("unknown command");
    }
    if (status != RO_OK) {
        fprintf(stderr, "reduced_host: error %d\n", (int)status);
        return 2;
    }
    return 0;
}
