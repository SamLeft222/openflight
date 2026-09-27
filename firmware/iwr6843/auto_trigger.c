/* On-radar shot detection; see auto_trigger.h and autotrigger.py. */
#include <string.h>

#include "auto_trigger.h"

int32_t at_check_config(const at_config_t *cfg)
{
    if (cfg == NULL || cfg->end_lo > cfg->end_hi ||
        cfg->n_frames < 2U || cfg->n_frames > AT_MAX_FRAMES ||
        cfg->slope_min_q < 1U || cfg->slope_min_q > cfg->slope_max_q ||
        cfg->slope_max_q > AT_MAX_SLOPE_Q ||
        cfg->z_min_x10 < 1UL || cfg->z_min_x10 > cfg->z_cap_x10 ||
        cfg->z_cap_x10 > AT_MAX_Z_X10 || cfg->delay_frames > AT_MAX_DELAY) {
        return AT_ERR_CONFIG;
    }
    return AT_OK;
}

void at_reset(at_state_t *st)
{
    st->next = 0U;
    st->filled = 0U;
    st->frames_seen = 0U;
    st->countdown = 0U;
    st->pending = 0U;
    st->fired = 0U;
    st->has_detection = 0U;
    st->det_end = 0U;
    st->det_slope_q = 0U;
    st->det_frame = 0U;
    st->det_score = 0.0;
}

/* Floor of value / 4 for any sign (C89 division truncates toward zero). */
static int32_t at_floor_div4(int32_t value)
{
    return (value >= 0) ? (value / 4) : -((-value + 3) / 4);
}

/* Median by insertion sort: the middle value, or the mean of the middle two. */
static double at_median(const double *values, uint32_t n, double *sorted)
{
    uint32_t i, j;

    if (n == 0U) {
        return 0.0;
    }
    for (i = 0U; i < n; i++) {
        double value = values[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }
    if (n % 2U) {
        return sorted[n / 2U];
    }
    return (sorted[n / 2U - 1U] + sorted[n / 2U]) * 0.5;
}

/* MTI power per bin of the first TX, into st->work. Every sum is an exact
 * integer below 2^53 (|code| <= 32768, loops <= 16), divided once. */
static void at_frame_power(at_state_t *st, const int16_t *frame, uint32_t loops,
                           uint8_t n_tx, uint8_t n_rx, uint8_t bin_count)
{
    uint32_t bin, rx, loop;
    uint32_t chirp_values = (uint32_t)n_rx * bin_count * 2U;

    for (bin = 0U; bin < bin_count; bin++) {
        double numerator = 0.0;
        for (rx = 0U; rx < n_rx; rx++) {
            double sum_sq = 0.0;
            int32_t sum_im = 0, sum_re = 0;
            for (loop = 0U; loop < loops; loop++) {
                const int16_t *x = frame + (loop * n_tx) * chirp_values +
                                   ((uint32_t)rx * bin_count + bin) * 2U;
                sum_im += x[0];
                sum_re += x[1];
                sum_sq += (double)x[0] * (double)x[0] + (double)x[1] * (double)x[1];
            }
            numerator += (double)loops * sum_sq -
                         ((double)sum_im * (double)sum_im + (double)sum_re * (double)sum_re);
        }
        st->work[bin] = numerator / (double)loops;
    }
}

/* z of row k (0 = oldest of the last n) at an absolute bin; 0 outside it. */
static double at_row_z(const at_state_t *st, uint32_t n, uint32_t k, int32_t absolute_bin)
{
    uint32_t row = (st->next + AT_MAX_FRAMES - n + k) % AT_MAX_FRAMES;
    int32_t offset = absolute_bin - (int32_t)st->start[row];

    if (offset < 0 || offset >= (int32_t)st->count[row]) {
        return 0.0;
    }
    return st->z[row][offset];
}

/* Best track over the last n rows; 1 when one reaches z_min in every row. */
static uint8_t at_best_track(at_state_t *st, const at_config_t *cfg)
{
    uint32_t n = cfg->n_frames;
    double z_min = (double)cfg->z_min_x10 / 10.0;
    uint8_t found = 0U;
    uint32_t slope_q, end, k;

    for (slope_q = cfg->slope_min_q; slope_q <= cfg->slope_max_q; slope_q++) {
        for (end = cfg->end_lo; end <= cfg->end_hi; end++) {
            uint32_t lit = 0U;
            double score = 0.0;
            for (k = 0U; k < n; k++) {
                int32_t position_q = 4 * (int32_t)end - (int32_t)(slope_q * (n - 1U - k));
                double lo = at_row_z(st, n, k, at_floor_div4(position_q));
                double hi = at_row_z(st, n, k, -at_floor_div4(-position_q));
                double value = (lo >= hi) ? lo : hi;
                score += value;
                if (value >= z_min) {
                    lit++;
                }
            }
            if (lit == n && (!found || score > st->det_score)) {
                found = 1U;
                st->det_end = (uint8_t)end;
                st->det_slope_q = (uint8_t)slope_q;
                st->det_score = score;
            }
        }
    }
    return found;
}

int32_t at_push_frame(at_state_t *st, const at_config_t *cfg, const int16_t *frame,
                      uint16_t chirps, uint8_t n_tx, uint8_t n_rx,
                      uint8_t bin_start, uint8_t bin_count)
{
    uint32_t loops, bin, row, frame_index;
    double level, z_cap;

    if (at_check_config(cfg) != AT_OK) {
        return AT_ERR_CONFIG;
    }
    if (frame == NULL || n_tx == 0U || n_rx == 0U || chirps % n_tx != 0U ||
        bin_count == 0U || bin_count > AT_MAX_WINDOW_BINS ||
        (uint32_t)bin_start + bin_count > AT_BIN_SPACE) {
        return AT_ERR_FRAME;
    }
    loops = chirps / n_tx;
    if (loops < 2U || loops > AT_MAX_LOOPS) {
        return AT_ERR_FRAME;
    }

    at_frame_power(st, frame, loops, n_tx, n_rx, bin_count);
    level = at_median(st->work, bin_count, st->sorted);
    z_cap = (double)cfg->z_cap_x10 / 10.0;
    row = st->next;
    for (bin = 0U; bin < bin_count; bin++) {
        double z = (level > 0.0) ? st->work[bin] / level : 0.0;
        st->z[row][bin] = (z < z_cap) ? z : z_cap;
    }
    st->start[row] = bin_start;
    st->count[row] = bin_count;
    st->next = (row + 1U) % AT_MAX_FRAMES;
    if (st->filled < cfg->n_frames) {
        st->filled++;
    }
    frame_index = st->frames_seen++;

    if (st->fired) {
        return 0;
    }
    if (st->pending) {
        st->countdown--;
        st->fired = (uint8_t)(st->countdown == 0U);
        return st->fired;
    }
    if (st->filled < cfg->n_frames || !at_best_track(st, cfg)) {
        return 0;
    }
    st->has_detection = 1U;
    st->det_frame = frame_index;
    st->countdown = cfg->delay_frames;
    st->pending = (uint8_t)(st->countdown > 0U);
    st->fired = (uint8_t)(st->countdown == 0U);
    return st->fired;
}
