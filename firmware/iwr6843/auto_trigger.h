/* On-radar shot detection (src/openflight/iwr6843/autotrigger.py is the
 * reference model; tests/test_iwr6843_auto_trigger_firmware.py holds the two
 * bit-identical).
 *
 * With the OPS243 on its internal trigger there is no sound edge, and the Pi
 * learns of a shot too late to freeze the 72 ms ring itself. The radar
 * watches the frames it records before impact for the club head's approach:
 * an outward track at club speed that reaches the tee.
 *
 * Per frame, first TX only: MTI power per bin over the loops (summed over
 * RX), normalised by the frame median and capped. Over the last n_frames
 * rows, fire when a straight outward track (slope_min_q..slope_max_q quarter
 * bins per frame, ending at a bin in [end_lo, end_hi]) reaches z_min in every
 * frame; request the freeze delay_frames later.
 *
 * Portable C89 with no libm: exact integers held in doubles until one
 * division per value, and the line search in integer quarter bins, so the
 * radar and reference agree bit for bit. Hosts must build with
 * -ffp-contract=off (the radar's VFPv3 has no fused multiply-add).
 */
#ifndef AUTO_TRIGGER_H
#define AUTO_TRIGGER_H

#include <stdint.h>

#define AT_MAX_FRAMES      8U
#define AT_MAX_WINDOW_BINS 64U
#define AT_MAX_LOOPS       16U
#define AT_BIN_SPACE       256U
#define AT_MAX_SLOPE_Q     64U
#define AT_MAX_DELAY       32U
#define AT_MAX_Z_X10       100000UL

#define AT_OK           0
#define AT_ERR_CONFIG  (-1)
#define AT_ERR_FRAME   (-2)

typedef struct {
    uint8_t  end_lo;        /* absolute bins, inclusive */
    uint8_t  end_hi;
    uint8_t  n_frames;      /* 2..AT_MAX_FRAMES */
    uint8_t  slope_min_q;   /* quarter bins per frame */
    uint8_t  slope_max_q;
    uint8_t  delay_frames;  /* 0..AT_MAX_DELAY */
    uint32_t z_min_x10;     /* tenths */
    uint32_t z_cap_x10;
} at_config_t;

typedef struct {
    double   z[AT_MAX_FRAMES][AT_MAX_WINDOW_BINS];
    uint8_t  start[AT_MAX_FRAMES];
    uint8_t  count[AT_MAX_FRAMES];
    double   work[AT_MAX_WINDOW_BINS];   /* power, then its sorted copy */
    double   sorted[AT_MAX_WINDOW_BINS];
    uint32_t next;          /* row the next frame is written to */
    uint32_t filled;        /* rows held, up to n_frames */
    uint32_t frames_seen;
    uint32_t countdown;
    uint8_t  pending;       /* detected; counting down to the freeze */
    uint8_t  fired;
    /* The detection, once made. */
    uint8_t  has_detection;
    uint8_t  det_end;
    uint8_t  det_slope_q;
    uint32_t det_frame;
    double   det_score;
} at_state_t;

/* AT_OK when the configuration is usable. */
int32_t at_check_config(const at_config_t *cfg);

/* Forget history and any pending or fired detection. */
void at_reset(at_state_t *st);

/* Push one frame as stored: int16 (imag, real) pairs, [chirp][rx][bin], with
 * chirps = loops * n_tx and the first TX's chirps at 0, n_tx, 2 n_tx, ...
 * Returns 1 when the freeze should be requested now, 0 otherwise, or an
 * AT_ERR_* code. Fires once until at_reset. */
int32_t at_push_frame(at_state_t *st, const at_config_t *cfg, const int16_t *frame,
                      uint16_t chirps, uint8_t n_tx, uint8_t n_rx,
                      uint8_t bin_start, uint8_t bin_count);

#endif /* AUTO_TRIGGER_H */
