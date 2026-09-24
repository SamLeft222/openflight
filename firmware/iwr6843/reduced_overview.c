/* Reduced-transfer overview and strips -- see reduced_overview.h.
 *
 * Mirrors src/openflight/iwr6843/reduced.py (build_overview, pack_overview,
 * serve_strips). Floating-point sums may differ from numpy in the last ulp;
 * tests/test_iwr6843_reduced_firmware.py bounds the effect on the wire.
 */
#include <string.h>

#include "reduced_overview.h"
#include "reduced_log_table.h"

#define RO_SCOPE_BURST  0U
#define RO_SCOPE_WINDOW 1U

typedef void (*ro_visit_t)(void *ctx, double value);

/* -- little-endian output ------------------------------------------------ */

static void ro_write_u8(ro_sink_t sink, void *ctx, uint8_t value)
{
    sink(ctx, &value, 1U);
}

static void ro_put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8U);
}

static void ro_write_u16(ro_sink_t sink, void *ctx, uint16_t value)
{
    uint8_t out[2];

    ro_put_u16(out, value);
    sink(ctx, out, 2U);
}

static void ro_write_f32(ro_sink_t sink, void *ctx, double value)
{
    float    single = (float)value;
    uint32_t bits;
    uint8_t  out[4];

    memcpy(&bits, &single, sizeof(bits));
    out[0] = (uint8_t)(bits & 0xFFU);
    out[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    out[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    out[3] = (uint8_t)(bits >> 24U);
    sink(ctx, out, 4U);
}

/* -- capture access ------------------------------------------------------ */

static uint32_t ro_loops(const ro_capture_t *cap)
{
    return (uint32_t)cap->chirps_per_frame / (uint32_t)cap->n_tx;
}

/* The vertical TX pair: TX0 and TX2 of a 3-TX capture, else TX0 and TX1. */
static uint32_t ro_pair_tx(const ro_capture_t *cap, uint32_t pair)
{
    if (pair == 0U) {
        return 0U;
    }
    return (cap->n_tx == 3U) ? 2U : 1U;
}

static uint32_t ro_sample_bytes(const ro_capture_t *cap)
{
    return cap->iq8 ? 2U : 4U;
}

static void ro_sample(const ro_capture_t *cap, uint32_t frame, uint32_t chirp,
                      uint32_t rx, uint32_t bin, double *re, double *im)
{
    uint32_t count = cap->bin_count[frame];
    uint32_t index = ((chirp * cap->n_rx + rx) * count + bin) * ro_sample_bytes(cap);
    const uint8_t *p = cap->frame[frame] + index;

    if (cap->iq8) {
        double scale = (double)cap->iq8_scale[frame];
        *im = (double)(int8_t)p[0] * scale;
        *re = (double)(int8_t)p[1] * scale;
    } else {
        *im = (double)(int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
        *re = (double)(int16_t)(uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8U));
    }
}

static int32_t ro_check_capture(const ro_capture_t *cap)
{
    uint32_t frame;

    if (cap == 0 || cap->n_frames == 0U || cap->n_frames > RO_MAX_FRAMES ||
        (cap->n_tx != 2U && cap->n_tx != 3U) || cap->n_rx == 0U ||
        cap->n_rx > RO_MAX_RX || cap->chirps_per_frame % cap->n_tx != 0U ||
        ro_loops(cap) == 0U || ro_loops(cap) > RO_MAX_LOOPS) {
        return RO_ERR_ARGUMENT;
    }
    for (frame = 0U; frame < cap->n_frames; frame++) {
        if (cap->bin_count[frame] == 0U ||
            (uint32_t)cap->bin_start[frame] + cap->bin_count[frame] > RO_BIN_SPACE ||
            (cap->iq8 && cap->iq8_scale[frame] == 0U)) {
            return RO_ERR_WINDOW;
        }
    }
    return RO_OK;
}

/* -- frame tables ---------------------------------------------------------- */

void ro_write_frame_tables(const ro_capture_t *cap, const ro_window_t *windows,
                           ro_sink_t sink, void *ctx)
{
    uint32_t frame;

    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint8_t start = (windows != 0) ? windows[frame].start : cap->bin_start[frame];
        uint8_t count = (windows != 0) ? windows[frame].count : cap->bin_count[frame];

        ro_write_u8(sink, ctx, start);
        ro_write_u8(sink, ctx, count);
        ro_write_u16(sink, ctx, (frame == 0U) ? 0U : cap->delta_us[frame]);
    }
    if (cap->iq8) {
        for (frame = 0U; frame < cap->n_frames; frame++) {
            ro_write_u16(sink, ctx, cap->iq8_scale[frame]);
        }
    }
}

/* -- window-scope static means --------------------------------------------- */

static void ro_window_sums(const ro_capture_t *cap, ro_work_t *work)
{
    uint32_t loops = ro_loops(cap);
    uint32_t frame, pair, rx, bin, loop;

    memset(work->sum_re, 0, sizeof(work->sum_re));
    memset(work->sum_im, 0, sizeof(work->sum_im));
    memset(work->sum_count, 0, sizeof(work->sum_count));
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t start = cap->bin_start[frame];
        for (pair = 0U; pair < 2U; pair++) {
            uint32_t tx = ro_pair_tx(cap, pair);
            for (rx = 0U; rx < cap->n_rx; rx++) {
                for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
                    double re_sum = 0.0;
                    double im_sum = 0.0;
                    for (loop = 0U; loop < loops; loop++) {
                        double re, im;
                        ro_sample(cap, frame, loop * cap->n_tx + tx, rx, bin, &re, &im);
                        re_sum += re;
                        im_sum += im;
                    }
                    work->sum_re[pair][rx][start + bin] += re_sum;
                    work->sum_im[pair][rx][start + bin] += im_sum;
                }
            }
        }
        for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
            work->sum_count[start + bin] += loops;
        }
    }
}

static void ro_window_mean(const ro_work_t *work, uint32_t pair, uint32_t rx,
                           uint32_t bin, double *re, double *im)
{
    double count = (work->sum_count[bin] == 0U) ? 1.0 : (double)work->sum_count[bin];

    *re = work->sum_re[pair][rx][bin] / count;
    *im = work->sum_im[pair][rx][bin] / count;
}

/* Cache the static means one frame's MTI subtracts: the mean over its loops
 * (burst scope) or over every frame holding the bin (window scope). */
static void ro_load_frame_means(const ro_capture_t *cap, ro_work_t *work,
                                uint32_t scope, uint32_t frame)
{
    uint32_t loops = ro_loops(cap);
    uint32_t pair, rx, bin, loop;

    for (pair = 0U; pair < 2U; pair++) {
        uint32_t tx = ro_pair_tx(cap, pair);
        for (rx = 0U; rx < cap->n_rx; rx++) {
            for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
                double *re = &work->mean_re[pair][rx][bin];
                double *im = &work->mean_im[pair][rx][bin];
                if (scope == RO_SCOPE_WINDOW) {
                    ro_window_mean(work, pair, rx, cap->bin_start[frame] + bin, re, im);
                } else {
                    double re_sum = 0.0;
                    double im_sum = 0.0;
                    for (loop = 0U; loop < loops; loop++) {
                        double sample_re, sample_im;
                        ro_sample(cap, frame, loop * cap->n_tx + tx, rx, bin,
                                  &sample_re, &sample_im);
                        re_sum += sample_re;
                        im_sum += sample_im;
                    }
                    *re = re_sum / (double)loops;
                    *im = im_sum / (double)loops;
                }
            }
        }
    }
}

/* Visit |MTI|^2 of every vertical-pair element of the capture. */
static void ro_visit_scope(const ro_capture_t *cap, ro_work_t *work,
                           uint32_t scope, ro_visit_t visit, void *ctx)
{
    uint32_t loops = ro_loops(cap);
    uint32_t frame, pair, rx, bin, loop;

    for (frame = 0U; frame < cap->n_frames; frame++) {
        ro_load_frame_means(cap, work, scope, frame);
        for (pair = 0U; pair < 2U; pair++) {
            uint32_t tx = ro_pair_tx(cap, pair);
            for (rx = 0U; rx < cap->n_rx; rx++) {
                for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
                    double mean_re = work->mean_re[pair][rx][bin];
                    double mean_im = work->mean_im[pair][rx][bin];
                    for (loop = 0U; loop < loops; loop++) {
                        double re, im;
                        ro_sample(cap, frame, loop * cap->n_tx + tx, rx, bin, &re, &im);
                        re -= mean_re;
                        im -= mean_im;
                        visit(ctx, re * re + im * im);
                    }
                }
            }
        }
    }
}

/* -- exact median: radix selection on the bits of non-negative doubles ---- */

static uint64_t ro_key(double value)
{
    uint64_t key;

    memcpy(&key, &value, sizeof(key));
    return key;
}

static double ro_from_key(uint64_t key)
{
    double value;

    memcpy(&value, &key, sizeof(value));
    return value;
}

typedef struct {
    ro_work_t *work;
    uint64_t   prefix;      /* resolved high bits */
    uint32_t   high_shift;  /* keys match when (key >> high_shift) == prefix */
    uint32_t   shift;       /* digit being histogrammed */
    uint32_t   digit_mask;
    uint32_t   n_candidates;
} ro_select_t;

static uint32_t ro_matches(const ro_select_t *s, uint64_t key)
{
    return (s->high_shift >= 64U) ? 1U : (uint32_t)((key >> s->high_shift) == s->prefix);
}

static void ro_visit_histogram(void *ctx, double value)
{
    ro_select_t *s = (ro_select_t *)ctx;
    uint64_t key = ro_key(value);

    if (ro_matches(s, key)) {
        s->work->histogram[(uint32_t)(key >> s->shift) & s->digit_mask]++;
    }
}

static void ro_visit_collect(void *ctx, double value)
{
    ro_select_t *s = (ro_select_t *)ctx;
    uint64_t key = ro_key(value);

    /* Only a bucket of at most RO_MAX_CANDIDATES values is collected. */
    if (ro_matches(s, key)) {
        s->work->candidates[s->n_candidates++] = value;
    }
}

static void ro_sort(double *values, uint32_t count)
{
    uint32_t i, j;

    for (i = 1U; i < count; i++) {
        double value = values[i];
        j = i;
        while (j > 0U && values[j - 1U] > value) {
            values[j] = values[j - 1U];
            j--;
        }
        values[j] = value;
    }
}

/* Digit of the histogram bucket holding ``rank``; ``below`` counts earlier values. */
static uint32_t ro_rank_digit(const uint32_t *histogram, uint32_t mask, uint32_t rank,
                              uint32_t *below)
{
    uint32_t digit;

    *below = 0U;
    for (digit = 0U; digit < mask; digit++) {
        if (rank < *below + histogram[digit]) {
            break;
        }
        *below += histogram[digit];
    }
    return digit;
}

/* The rank_a-th and rank_b-th smallest (0-based) |MTI|^2 of one scope.
 * Both ranks share every pass while they fall in the same bucket, which the
 * two middle ranks of a median almost always do. */
static void ro_select_ranks(const ro_capture_t *cap, ro_work_t *work, uint32_t scope,
                            uint32_t rank_a, uint32_t rank_b, double *value_a,
                            double *value_b)
{
    ro_select_t s;
    uint32_t resolved = 0U;
    uint32_t first_a = rank_a;
    uint32_t first_b = rank_b;

    s.work = work;
    s.prefix = 0U;
    s.high_shift = 64U;
    while (resolved < 64U) {
        uint32_t bits = (64U - resolved < RO_HISTOGRAM_BITS) ? (64U - resolved)
                                                             : RO_HISTOGRAM_BITS;
        uint32_t digit_a, digit_b, below_a, below_b;

        s.shift = 64U - resolved - bits;
        s.digit_mask = (1U << bits) - 1U;
        memset(work->histogram, 0, sizeof(uint32_t) << bits);
        ro_visit_scope(cap, work, scope, ro_visit_histogram, &s);
        digit_a = ro_rank_digit(work->histogram, s.digit_mask, rank_a, &below_a);
        digit_b = ro_rank_digit(work->histogram, s.digit_mask, rank_b, &below_b);
        if (digit_a != digit_b) {
            /* Different buckets: resolve each rank on its own. */
            ro_select_ranks(cap, work, scope, first_a, first_a, value_a, value_a);
            ro_select_ranks(cap, work, scope, first_b, first_b, value_b, value_b);
            return;
        }
        rank_a -= below_a;
        rank_b -= below_b;
        s.prefix = (s.prefix << bits) | digit_a;
        resolved += bits;
        s.high_shift = 64U - resolved;
        if (resolved < 64U && work->histogram[digit_a] <= RO_MAX_CANDIDATES) {
            s.n_candidates = 0U;
            ro_visit_scope(cap, work, scope, ro_visit_collect, &s);
            ro_sort(work->candidates, s.n_candidates);
            *value_a = work->candidates[rank_a];
            *value_b = work->candidates[rank_b];
            return;
        }
    }
    *value_a = ro_from_key(s.prefix);
    *value_b = *value_a;
}

/* np.median over every element of one scope. */
static double ro_noise(const ro_capture_t *cap, ro_work_t *work, uint32_t scope,
                       uint32_t total)
{
    double lower, upper;

    if (total % 2U == 1U) {
        ro_select_ranks(cap, work, scope, total / 2U, total / 2U, &lower, &upper);
        return lower;
    }
    ro_select_ranks(cap, work, scope, total / 2U - 1U, total / 2U, &lower, &upper);
    return (lower + upper) / 2.0;
}

static uint32_t ro_element_count(const ro_capture_t *cap)
{
    uint32_t frame;
    uint32_t total = 0U;

    for (frame = 0U; frame < cap->n_frames; frame++) {
        total += 2U * cap->n_rx * ro_loops(cap) * cap->bin_count[frame];
    }
    return total;
}

/* -- log-power code ---------------------------------------------------------- */

/* round(log2(power) * 256) as int16; exactly zero power has its own code. */
static int16_t ro_log_power_code(double power)
{
    uint64_t key;
    int32_t exponent, code;
    double mantissa;
    uint32_t lo = 0U, hi = RO_LOG_TABLE_SIZE;

    if (power <= 0.0) {
        return (int16_t)RO_ZERO_POWER_CODE;
    }
    key = ro_key(power);
    exponent = (int32_t)((key >> 52U) & 0x7FFU);
    if (exponent == 0) {
        return (int16_t)(RO_ZERO_POWER_CODE + 1);  /* subnormal: far below the clip */
    }
    /* mantissa in [1, 2): same fraction bits, exponent of 1.0 */
    mantissa = ro_from_key((key & 0x000FFFFFFFFFFFFFULL) | 0x3FF0000000000000ULL);
    while (lo < hi) {  /* thresholds at or below the mantissa */
        uint32_t mid = (lo + hi) / 2U;
        if (ro_log_thresholds[mid] <= mantissa) {
            lo = mid + 1U;
        } else {
            hi = mid;
        }
    }
    code = (exponent - 1023) * RO_LOG_STEPS_PER_OCTAVE + (int32_t)lo;
    if (code < RO_ZERO_POWER_CODE + 1) {
        code = RO_ZERO_POWER_CODE + 1;
    }
    if (code > 32767) {
        code = 32767;
    }
    return (int16_t)code;
}

/* -- overview ------------------------------------------------------------------ */

static void ro_gate_columns(const ro_capture_t *cap, uint32_t frame, uint32_t gate_lo,
                            uint32_t gate_hi, uint32_t *a, uint32_t *b)
{
    uint32_t start = cap->bin_start[frame];
    uint32_t end = start + cap->bin_count[frame];
    uint32_t lo = (gate_lo > start) ? gate_lo : start;
    uint32_t hi = (gate_hi < end) ? gate_hi : end;

    *a = lo - start;
    *b = (hi > lo) ? hi - start : *a;
}

static void ro_write_power_map(const ro_capture_t *cap, ro_work_t *work, uint32_t scope,
                               uint32_t gate_lo, uint32_t gate_hi,
                               ro_sink_t sink, void *ctx)
{
    uint32_t loops = ro_loops(cap);
    uint32_t frame, loop, bin, pair, rx;

    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t a, b;

        ro_gate_columns(cap, frame, gate_lo, gate_hi, &a, &b);
        if (b == a) {
            continue;
        }
        ro_load_frame_means(cap, work, scope, frame);
        for (loop = 0U; loop < loops; loop++) {
            for (bin = a; bin < b; bin++) {
                double power = 0.0;
                for (pair = 0U; pair < 2U; pair++) {
                    uint32_t tx = ro_pair_tx(cap, pair);
                    for (rx = 0U; rx < cap->n_rx; rx++) {
                        double re, im;
                        ro_sample(cap, frame, loop * cap->n_tx + tx, rx, bin, &re, &im);
                        re -= work->mean_re[pair][rx][bin];
                        im -= work->mean_im[pair][rx][bin];
                        power += re * re + im * im;
                    }
                }
                ro_put_u16(&work->row[2U * (bin - a)], (uint16_t)ro_log_power_code(power));
            }
            sink(ctx, work->row, 2U * (b - a));
        }
    }
}

int32_t ro_check_overview(const ro_capture_t *cap, uint16_t gate_lo, uint16_t gate_hi)
{
    int32_t status = ro_check_capture(cap);

    if (status != RO_OK) {
        return status;
    }
    if (gate_lo >= gate_hi || gate_hi > RO_BIN_SPACE) {
        return RO_ERR_GATE;
    }
    return RO_OK;
}

int32_t ro_write_overview(const ro_capture_t *cap, uint16_t gate_lo,
                          uint16_t gate_hi, ro_work_t *work,
                          ro_sink_t sink, void *ctx)
{
    uint32_t frame, pair, rx, bin, total;
    uint32_t means_lo = RO_BIN_SPACE, means_hi = 0U;
    double noise_burst, noise_window;
    int32_t status = ro_check_overview(cap, gate_lo, gate_hi);

    if (status != RO_OK || work == 0 || sink == 0) {
        return (status != RO_OK) ? status : RO_ERR_ARGUMENT;
    }
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t start = cap->bin_start[frame];
        uint32_t end = start + cap->bin_count[frame];
        means_lo = (start < means_lo) ? start : means_lo;
        means_hi = (end > means_hi) ? end : means_hi;
    }

    ro_window_sums(cap, work);
    total = ro_element_count(cap);
    noise_burst = ro_noise(cap, work, RO_SCOPE_BURST, total);
    noise_window = ro_noise(cap, work, RO_SCOPE_WINDOW, total);
    if (!(noise_burst > 0.0) || !(noise_window > 0.0)) {
        return RO_ERR_NOISE;
    }

    ro_write_frame_tables(cap, 0, sink, ctx);
    sink(ctx, (const uint8_t *)RO_OVERVIEW_MAGIC, 4U);
    ro_write_u16(sink, ctx, (uint16_t)RO_OVERVIEW_VERSION);
    ro_write_u16(sink, ctx, gate_lo);
    ro_write_u16(sink, ctx, gate_hi);
    ro_write_u16(sink, ctx, (uint16_t)ro_loops(cap));
    ro_write_f32(sink, ctx, noise_burst);
    ro_write_f32(sink, ctx, noise_window);
    ro_write_u16(sink, ctx, (uint16_t)means_lo);
    ro_write_u16(sink, ctx, (uint16_t)means_hi);

    ro_write_power_map(cap, work, RO_SCOPE_BURST, gate_lo, gate_hi, sink, ctx);
    ro_write_power_map(cap, work, RO_SCOPE_WINDOW, gate_lo, gate_hi, sink, ctx);

    for (pair = 0U; pair < 2U; pair++) {
        for (rx = 0U; rx < cap->n_rx; rx++) {
            for (bin = means_lo; bin < means_hi; bin++) {
                double re, im;
                ro_window_mean(work, pair, rx, bin, &re, &im);
                ro_write_f32(sink, ctx, re);
                ro_write_f32(sink, ctx, im);
            }
        }
    }
    return RO_OK;
}

/* -- strips --------------------------------------------------------------------- */

int32_t ro_check_strips(const ro_capture_t *cap, const ro_window_t *request)
{
    uint32_t frame;
    int32_t status = ro_check_capture(cap);

    if (status != RO_OK || request == 0) {
        return (status != RO_OK) ? status : RO_ERR_ARGUMENT;
    }
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t start = cap->bin_start[frame];
        uint32_t count = cap->bin_count[frame];

        if (request[frame].count != 0U &&
            (request[frame].start < start ||
             (uint32_t)request[frame].start + request[frame].count > start + count)) {
            return RO_ERR_WINDOW;
        }
    }
    return RO_OK;
}

int32_t ro_write_strips(const ro_capture_t *cap, const ro_window_t *request,
                        ro_sink_t sink, void *ctx)
{
    ro_window_t windows[RO_MAX_FRAMES];
    uint32_t bytes;
    uint32_t frame, chirp, rx;
    int32_t status = ro_check_strips(cap, request);

    if (status != RO_OK || sink == 0) {
        return (status != RO_OK) ? status : RO_ERR_ARGUMENT;
    }
    bytes = ro_sample_bytes(cap);
    for (frame = 0U; frame < cap->n_frames; frame++) {
        if (request[frame].count == 0U) {
            windows[frame].start = cap->bin_start[frame];
            windows[frame].count = 1U;
        } else {
            windows[frame] = request[frame];
        }
    }
    ro_write_frame_tables(cap, windows, sink, ctx);
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t local = (uint32_t)windows[frame].start - cap->bin_start[frame];
        uint32_t stored = cap->bin_count[frame];

        for (chirp = 0U; chirp < cap->chirps_per_frame; chirp++) {
            for (rx = 0U; rx < cap->n_rx; rx++) {
                const uint8_t *row =
                    cap->frame[frame] + ((chirp * cap->n_rx + rx) * stored + local) * bytes;
                sink(ctx, row, (uint32_t)windows[frame].count * bytes);
            }
        }
    }
    return RO_OK;
}

static int32_t ro_hex_digit(char c)
{
    if (c >= '0' && c <= '9') {
        return (int32_t)(c - '0');
    }
    if (c >= 'a' && c <= 'f') {
        return (int32_t)(c - 'a') + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return (int32_t)(c - 'A') + 10;
    }
    return -1;
}

int32_t ro_parse_strip_request(const char *hex, uint16_t n_frames,
                               ro_window_t *request)
{
    uint32_t frame, i;

    if (hex == 0 || request == 0 || n_frames == 0U ||
        n_frames > RO_MAX_STRIP_REQUEST_FRAMES || strlen(hex) != 4U * n_frames) {
        return RO_ERR_REQUEST;
    }
    for (frame = 0U; frame < n_frames; frame++) {
        uint8_t bytes[2];
        for (i = 0U; i < 2U; i++) {
            int32_t high = ro_hex_digit(hex[4U * frame + 2U * i]);
            int32_t low = ro_hex_digit(hex[4U * frame + 2U * i + 1U]);
            if (high < 0 || low < 0) {
                return RO_ERR_REQUEST;
            }
            bytes[i] = (uint8_t)(high * 16 + low);
        }
        request[frame].start = bytes[0];
        request[frame].count = bytes[1];
    }
    return RO_OK;
}
