/* Reduced-transfer overview and strips (plans/iwr6843-on-chip-reduction.md).
 *
 * Portable C with no TI headers: the firmware links it, and the host test
 * harness (firmware/iwr6843/test/reduced_host.c) compiles the same file so
 * tests/test_iwr6843_reduced_firmware.py can check it against the Python
 * reference model (src/openflight/iwr6843/reduced.py).
 *
 * The frozen capture is described in dump order (the order l3dump streams
 * frames). Each frame's payload is laid out exactly as streamed: for each
 * chirp, each RX, each local range bin, one complex sample in TI's
 * imaginary-then-real order, as int8 (IQ8, times the frame scale) or
 * little-endian int16 (IQ16).
 *
 * The maths is integer where it can be: for stored codes c over L loops,
 * burst-scope MTI power is s^2 |L c - sum c|^2 / L^2 and window-scope power
 * is |n s c - T|^2 / n^2, each rounded once. reduced.py uses the same
 * formulation, so the bytes match exactly. Hosts must build with
 * -ffp-contract=off: a fused multiply-add would round differently (the
 * radar's VFPv3 has none).
 *
 * Output goes through a byte sink so nothing large is buffered. The caller
 * writes the dump header and temperature report first; these functions write
 * everything after it.
 */
#ifndef REDUCED_OVERVIEW_H
#define REDUCED_OVERVIEW_H

#include <stdint.h>

#define RO_OVERVIEW_MAGIC        "ILOV"
#define RO_OVERVIEW_VERSION      1U
#define RO_OVERVIEW_HEADER_BYTES 24U
#define RO_MAX_FRAMES            64U
#define RO_MAX_RX                4U
#define RO_MAX_LOOPS             16U   /* L3_MAX_LOOPS in l3_dump.c */
#define RO_MAX_FRAME_BINS        64U   /* L3_RING_MAX_BINS in l3_dump.c */
#define RO_FRAME_VALUES          (2U * RO_MAX_LOOPS * RO_MAX_RX * RO_MAX_FRAME_BINS)
#define RO_BIN_SPACE             256U  /* absolute range bins are uint8 */
#define RO_HISTOGRAM_BITS        12U
#define RO_MAX_CANDIDATES        2048U
#define RO_ZERO_POWER_CODE       (-32768)
#define RO_LOG_STEPS_PER_OCTAVE  256
/* Frames per l3strip request: 4 hex digits each on a 255-character CLI line. */
#define RO_MAX_STRIP_REQUEST_FRAMES 61U

#define RO_OK             0
#define RO_ERR_ARGUMENT  (-1)
#define RO_ERR_GATE      (-2)
#define RO_ERR_WINDOW    (-3)
#define RO_ERR_NOISE     (-4)
#define RO_ERR_REQUEST   (-5)

typedef void (*ro_sink_t)(void *ctx, const uint8_t *bytes, uint32_t count);

typedef struct {
    uint16_t n_frames;
    uint16_t chirps_per_frame;
    uint8_t  n_tx;
    uint8_t  n_rx;
    uint8_t  iq8;                     /* 1: int8 x frame scale; 0: int16 */
    const uint8_t  *bin_start;        /* [n_frames] absolute first bin */
    const uint8_t  *bin_count;        /* [n_frames] bins stored */
    const uint16_t *delta_us;         /* [n_frames] descriptor delta; [0] is 0 */
    const uint16_t *iq8_scale;        /* [n_frames] when iq8 */
    const uint8_t *const *frame;      /* [n_frames] payload pointers */
} ro_capture_t;

/* One frame of a strip request; count 0 means "not requested". */
typedef struct {
    uint8_t start;
    uint8_t count;
} ro_window_t;

/* Working memory for the overview (~104 KB; keep it off the stack). */
typedef struct {
    /* Window scope: scaled code sums T and loop counts n per absolute bin. */
    int64_t  total_re[2][RO_MAX_RX][RO_BIN_SPACE];
    int64_t  total_im[2][RO_MAX_RX][RO_BIN_SPACE];
    uint32_t total_count[RO_BIN_SPACE];
    double   inv_nn[RO_BIN_SPACE];               /* 1 / n^2 */
    /* One frame's vertical-pair codes [pair][loop][rx][bin] and loop sums. */
    int16_t  code_re[RO_FRAME_VALUES];
    int16_t  code_im[RO_FRAME_VALUES];
    int32_t  sum_re[2][RO_MAX_RX][RO_MAX_FRAME_BINS];
    int32_t  sum_im[2][RO_MAX_RX][RO_MAX_FRAME_BINS];
    uint32_t histogram[1U << RO_HISTOGRAM_BITS];
    double   candidates[RO_MAX_CANDIDATES];
    uint8_t  row[2U * RO_BIN_SPACE];
    /* Set by ro_prepare_overview for ro_stream_overview. */
    double   noise_burst;
    double   noise_window;
    uint16_t prepared_gate_lo;
    uint16_t prepared_gate_hi;
    uint8_t  prepared;
} ro_work_t;

/* Per-frame descriptors (start, count, delta) and, for IQ8, the scale table.
 * windows == NULL writes the capture's own windows. */
void ro_write_frame_tables(const ro_capture_t *cap, const ro_window_t *windows,
                           ro_sink_t sink, void *ctx);

/* RO_OK when the overview accepts the capture and gate. The only later
 * failure is RO_ERR_NOISE, from ro_prepare_overview, before any output. */
int32_t ro_check_overview(const ro_capture_t *cap, uint16_t gate_lo, uint16_t gate_hi);

/* RO_OK when ro_write_strips would accept the request. */
int32_t ro_check_strips(const ro_capture_t *cap, const ro_window_t *request);

/* Window sums and both noise medians; writes nothing. */
int32_t ro_prepare_overview(const ro_capture_t *cap, uint16_t gate_lo,
                            uint16_t gate_hi, ro_work_t *work);

/* Frame tables, overview header, both power maps, and window means, for a
 * capture and gate ro_prepare_overview accepted. */
int32_t ro_stream_overview(const ro_capture_t *cap, uint16_t gate_lo,
                           uint16_t gate_hi, ro_work_t *work,
                           ro_sink_t sink, void *ctx);

/* ro_prepare_overview then ro_stream_overview. */
int32_t ro_write_overview(const ro_capture_t *cap, uint16_t gate_lo,
                          uint16_t gate_hi, ro_work_t *work,
                          ro_sink_t sink, void *ctx);

/* Frame tables for the request, then each frame's requested bins as stored.
 * An unrequested frame carries the first bin of its window. */
int32_t ro_write_strips(const ro_capture_t *cap, const ro_window_t *request,
                        ro_sink_t sink, void *ctx);

/* "SSCC..." (start, count bytes per frame, hex) -> request[n_frames]. */
int32_t ro_parse_strip_request(const char *hex, uint16_t n_frames,
                               ro_window_t *request);

#endif /* REDUCED_OVERVIEW_H */
