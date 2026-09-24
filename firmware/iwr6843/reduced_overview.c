/* Reduced-transfer overview and strips -- see reduced_overview.h.
 *
 * Mirrors src/openflight/iwr6843/reduced.py (build_overview, pack_overview,
 * serve_strips) bit-for-bit; tests/test_iwr6843_reduced_firmware.py checks it.
 *
 * Every pass decodes one frame of the vertical TX pair into fast memory
 * (work->code_*) with its loop sums, then works on integers: the frozen ring
 * sits in slow L3 and is read once per pass.
 */
#include <string.h>

#include "reduced_overview.h"
#include "reduced_log_table.h"

#define RO_SCOPE_BURST  0U
#define RO_SCOPE_WINDOW 1U

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

static uint32_t ro_scale(const ro_capture_t *cap, uint32_t frame)
{
    return cap->iq8 ? (uint32_t)cap->iq8_scale[frame] : 1U;
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
        if (cap->bin_count[frame] == 0U || cap->bin_count[frame] > RO_MAX_FRAME_BINS ||
            (uint32_t)cap->bin_start[frame] + cap->bin_count[frame] > RO_BIN_SPACE ||
            (cap->iq8 && cap->iq8_scale[frame] == 0U)) {
            return RO_ERR_WINDOW;
        }
    }
    return RO_OK;
}

/* Index of (pair, loop, rx, bin) in work->code_* for a frame of `count` bins. */
static uint32_t ro_code_index(const ro_capture_t *cap, uint32_t count, uint32_t pair,
                              uint32_t loop, uint32_t rx)
{
    return ((pair * ro_loops(cap) + loop) * cap->n_rx + rx) * count;
}

/* Decode one frame's vertical-pair codes into fast memory, with loop sums. */
static void ro_load_frame(const ro_capture_t *cap, ro_work_t *work, uint32_t frame)
{
    uint32_t loops = ro_loops(cap);
    uint32_t count = cap->bin_count[frame];
    uint32_t bytes = ro_sample_bytes(cap);
    uint32_t pair, loop, rx, bin;

    for (pair = 0U; pair < 2U; pair++) {
        uint32_t tx = ro_pair_tx(cap, pair);
        for (rx = 0U; rx < cap->n_rx; rx++) {
            int32_t *sum_re = work->sum_re[pair][rx];
            int32_t *sum_im = work->sum_im[pair][rx];
            for (bin = 0U; bin < count; bin++) {
                sum_re[bin] = 0;
                sum_im[bin] = 0;
            }
            for (loop = 0U; loop < loops; loop++) {
                uint32_t chirp = loop * cap->n_tx + tx;
                const uint8_t *src = cap->frame[frame] + (chirp * cap->n_rx + rx) * count * bytes;
                uint32_t index = ro_code_index(cap, count, pair, loop, rx);
                int16_t *re = &work->code_re[index];
                int16_t *im = &work->code_im[index];
                for (bin = 0U; bin < count; bin++) {
                    if (cap->iq8) {
                        im[bin] = (int16_t)(int8_t)src[2U * bin];
                        re[bin] = (int16_t)(int8_t)src[2U * bin + 1U];
                    } else {
                        const uint8_t *p = src + 4U * bin;
                        im[bin] = (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
                        re[bin] = (int16_t)(uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8U));
                    }
                    sum_re[bin] += re[bin];
                    sum_im[bin] += im[bin];
                }
            }
        }
    }
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

/* -- MTI power of one element -------------------------------------------- */

/* s^2 / L^2: burst-scope power is |L c - sum c|^2 times this. */
static double ro_burst_factor(const ro_capture_t *cap, uint32_t frame)
{
    uint32_t scale = ro_scale(cap, frame);
    uint32_t loops = ro_loops(cap);

    return (double)(scale * scale) / (double)(loops * loops);
}

/* Exact |L c - sum c|^2 for one element of the loaded frame. */
static int64_t ro_burst_residual(const ro_capture_t *cap, const ro_work_t *work,
                                 uint32_t index, uint32_t pair, uint32_t rx, uint32_t bin)
{
    int32_t loops = (int32_t)ro_loops(cap);
    int32_t re = loops * work->code_re[index + bin] - work->sum_re[pair][rx][bin];
    int32_t im = loops * work->code_im[index + bin] - work->sum_im[pair][rx][bin];

    return (int64_t)re * re + (int64_t)im * im;
}

/* |n s c - T|^2 (rounded once) for one element; power is this times 1/n^2. */
static double ro_window_residual(const ro_work_t *work, uint32_t index, uint32_t pair,
                                 uint32_t rx, uint32_t bin, uint32_t absolute_bin,
                                 int64_t n_scale)
{
    double re = (double)(n_scale * work->code_re[index + bin] -
                         work->total_re[pair][rx][absolute_bin]);
    double im = (double)(n_scale * work->code_im[index + bin] -
                         work->total_im[pair][rx][absolute_bin]);

    return re * re + im * im;
}

/* -- window-scope sums ------------------------------------------------------- */

static void ro_window_sums(const ro_capture_t *cap, ro_work_t *work)
{
    uint32_t frame, pair, rx, bin;

    memset(work->total_re, 0, sizeof(work->total_re));
    memset(work->total_im, 0, sizeof(work->total_im));
    memset(work->total_count, 0, sizeof(work->total_count));
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t start = cap->bin_start[frame];
        int64_t scale = (int64_t)ro_scale(cap, frame);

        ro_load_frame(cap, work, frame);
        for (pair = 0U; pair < 2U; pair++) {
            for (rx = 0U; rx < cap->n_rx; rx++) {
                for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
                    work->total_re[pair][rx][start + bin] += scale * work->sum_re[pair][rx][bin];
                    work->total_im[pair][rx][start + bin] += scale * work->sum_im[pair][rx][bin];
                }
            }
        }
        for (bin = 0U; bin < cap->bin_count[frame]; bin++) {
            work->total_count[start + bin] += ro_loops(cap);
        }
    }
    for (bin = 0U; bin < RO_BIN_SPACE; bin++) {
        uint32_t n = work->total_count[bin];
        work->inv_nn[bin] = (n == 0U) ? 0.0 : 1.0 / (double)(n * n);
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
    uint64_t   prefix;       /* resolved high bits */
    uint32_t   high_shift;   /* keys match when (key >> high_shift) == prefix */
    uint32_t   shift;        /* digit being histogrammed */
    uint32_t   digit_mask;
    uint32_t   collect;      /* 0: histogram the digit; 1: collect matching values */
    uint32_t   n_candidates;
} ro_select_t;

static void ro_select_add(ro_select_t *s, double value)
{
    uint64_t key = ro_key(value);

    if (s->high_shift < 64U && (key >> s->high_shift) != s->prefix) {
        return;
    }
    if (s->collect) {
        /* Only a bucket of at most RO_MAX_CANDIDATES values is collected. */
        s->work->candidates[s->n_candidates++] = value;
    } else {
        s->work->histogram[(uint32_t)(key >> s->shift) & s->digit_mask]++;
    }
}

/* Feed |MTI|^2 of every vertical-pair element of one scope to the selection. */
static void ro_scan_scope(const ro_capture_t *cap, ro_work_t *work, uint32_t scope,
                          ro_select_t *s)
{
    uint32_t loops = ro_loops(cap);
    uint32_t frame, pair, loop, rx, bin;

    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t count = cap->bin_count[frame];
        uint32_t start = cap->bin_start[frame];
        double k = ro_burst_factor(cap, frame);
        int64_t scale = (int64_t)ro_scale(cap, frame);

        ro_load_frame(cap, work, frame);
        for (pair = 0U; pair < 2U; pair++) {
            for (loop = 0U; loop < loops; loop++) {
                for (rx = 0U; rx < cap->n_rx; rx++) {
                    uint32_t index = ro_code_index(cap, count, pair, loop, rx);
                    for (bin = 0U; bin < count; bin++) {
                        double value;
                        if (scope == RO_SCOPE_BURST) {
                            value = (double)ro_burst_residual(cap, work, index, pair, rx, bin) * k;
                        } else {
                            int64_t n_scale = (int64_t)work->total_count[start + bin] * scale;
                            value = ro_window_residual(work, index, pair, rx, bin, start + bin,
                                                       n_scale) *
                                    work->inv_nn[start + bin];
                        }
                        ro_select_add(s, value);
                    }
                }
            }
        }
    }
}

/* Shell sort (candidates are few; C89 has no qsort guarantee on speed). */
static void ro_sort(double *values, uint32_t count)
{
    uint32_t gap, i, j;

    for (gap = count / 2U; gap > 0U; gap /= 2U) {
        for (i = gap; i < count; i++) {
            double value = values[i];
            for (j = i; j >= gap && values[j - gap] > value; j -= gap) {
                values[j] = values[j - gap];
            }
            values[j] = value;
        }
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
        s.collect = 0U;
        memset(work->histogram, 0, sizeof(uint32_t) << bits);
        ro_scan_scope(cap, work, scope, &s);
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
            s.collect = 1U;
            s.n_candidates = 0U;
            ro_scan_scope(cap, work, scope, &s);
            ro_sort(work->candidates, s.n_candidates);
            *value_a = work->candidates[rank_a];
            *value_b = work->candidates[rank_b];
            return;
        }
    }
    *value_a = ro_from_key(s.prefix);
    *value_b = *value_a;
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

/* np.median over every element of one scope. */
static double ro_noise(const ro_capture_t *cap, ro_work_t *work, uint32_t scope)
{
    uint32_t total = ro_element_count(cap);
    double lower, upper;

    if (total % 2U == 1U) {
        ro_select_ranks(cap, work, scope, total / 2U, total / 2U, &lower, &upper);
        return lower;
    }
    ro_select_ranks(cap, work, scope, total / 2U - 1U, total / 2U, &lower, &upper);
    return (lower + upper) / 2.0;
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
        uint32_t count = cap->bin_count[frame];
        uint32_t start = cap->bin_start[frame];
        double k = ro_burst_factor(cap, frame);
        int64_t scale = (int64_t)ro_scale(cap, frame);
        uint32_t a, b;

        ro_gate_columns(cap, frame, gate_lo, gate_hi, &a, &b);
        if (b == a) {
            continue;
        }
        ro_load_frame(cap, work, frame);
        for (loop = 0U; loop < loops; loop++) {
            for (bin = a; bin < b; bin++) {
                double power;
                if (scope == RO_SCOPE_BURST) {
                    int64_t residual = 0;
                    for (pair = 0U; pair < 2U; pair++) {
                        for (rx = 0U; rx < cap->n_rx; rx++) {
                            residual += ro_burst_residual(
                                cap, work, ro_code_index(cap, count, pair, loop, rx), pair, rx, bin);
                        }
                    }
                    power = (double)residual * k;
                } else {
                    int64_t n_scale = (int64_t)work->total_count[start + bin] * scale;
                    double residual = 0.0;
                    for (pair = 0U; pair < 2U; pair++) {
                        for (rx = 0U; rx < cap->n_rx; rx++) {
                            residual += ro_window_residual(
                                work, ro_code_index(cap, count, pair, loop, rx), pair, rx, bin,
                                start + bin, n_scale);
                        }
                    }
                    power = residual * work->inv_nn[start + bin];
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

int32_t ro_prepare_overview(const ro_capture_t *cap, uint16_t gate_lo,
                            uint16_t gate_hi, ro_work_t *work)
{
    int32_t status = ro_check_overview(cap, gate_lo, gate_hi);

    if (status != RO_OK || work == 0) {
        return (status != RO_OK) ? status : RO_ERR_ARGUMENT;
    }
    work->prepared = 0U;
    ro_window_sums(cap, work);
    work->noise_burst = ro_noise(cap, work, RO_SCOPE_BURST);
    work->noise_window = ro_noise(cap, work, RO_SCOPE_WINDOW);
    if (!(work->noise_burst > 0.0) || !(work->noise_window > 0.0)) {
        return RO_ERR_NOISE;
    }
    work->prepared_gate_lo = gate_lo;
    work->prepared_gate_hi = gate_hi;
    work->prepared = 1U;
    return RO_OK;
}

int32_t ro_stream_overview(const ro_capture_t *cap, uint16_t gate_lo,
                           uint16_t gate_hi, ro_work_t *work,
                           ro_sink_t sink, void *ctx)
{
    uint32_t frame, pair, rx, bin;
    uint32_t means_lo = RO_BIN_SPACE, means_hi = 0U;

    if (work == 0 || sink == 0 || !work->prepared || work->prepared_gate_lo != gate_lo ||
        work->prepared_gate_hi != gate_hi || ro_check_overview(cap, gate_lo, gate_hi) != RO_OK) {
        return RO_ERR_ARGUMENT;
    }
    for (frame = 0U; frame < cap->n_frames; frame++) {
        uint32_t start = cap->bin_start[frame];
        uint32_t end = start + cap->bin_count[frame];
        means_lo = (start < means_lo) ? start : means_lo;
        means_hi = (end > means_hi) ? end : means_hi;
    }

    ro_write_frame_tables(cap, 0, sink, ctx);
    sink(ctx, (const uint8_t *)RO_OVERVIEW_MAGIC, 4U);
    ro_write_u16(sink, ctx, (uint16_t)RO_OVERVIEW_VERSION);
    ro_write_u16(sink, ctx, gate_lo);
    ro_write_u16(sink, ctx, gate_hi);
    ro_write_u16(sink, ctx, (uint16_t)ro_loops(cap));
    ro_write_f32(sink, ctx, work->noise_burst);
    ro_write_f32(sink, ctx, work->noise_window);
    ro_write_u16(sink, ctx, (uint16_t)means_lo);
    ro_write_u16(sink, ctx, (uint16_t)means_hi);

    ro_write_power_map(cap, work, RO_SCOPE_BURST, gate_lo, gate_hi, sink, ctx);
    ro_write_power_map(cap, work, RO_SCOPE_WINDOW, gate_lo, gate_hi, sink, ctx);

    for (pair = 0U; pair < 2U; pair++) {
        for (rx = 0U; rx < cap->n_rx; rx++) {
            for (bin = means_lo; bin < means_hi; bin++) {
                uint32_t n = work->total_count[bin];
                double denominator = (n == 0U) ? 1.0 : (double)n;
                ro_write_f32(sink, ctx, (double)work->total_re[pair][rx][bin] / denominator);
                ro_write_f32(sink, ctx, (double)work->total_im[pair][rx][bin] / denominator);
            }
        }
    }
    work->prepared = 0U;
    return RO_OK;
}

int32_t ro_write_overview(const ro_capture_t *cap, uint16_t gate_lo,
                          uint16_t gate_hi, ro_work_t *work,
                          ro_sink_t sink, void *ctx)
{
    int32_t status = ro_prepare_overview(cap, gate_lo, gate_hi, work);

    if (status != RO_OK) {
        return status;
    }
    return ro_stream_overview(cap, gate_lo, gate_hi, work, sink, ctx);
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
