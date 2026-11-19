#ifndef HEXASTORM_VECTOR_CORE_H
#define HEXASTORM_VECTOR_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    HX_FILL_EVENODD = 0,
    HX_FILL_NONZERO = 1
};

enum {
    HX_OK = 0,
    HX_ERR_NOMEM = -1,
    HX_ERR_STEP_PERIOD = -2,
    HX_ERR_TOO_MANY_EVENTS = -3,
    HX_ERR_INVALID_ARG = -4
};

typedef struct {
    const double *u0;
    const double *v0;
    const double *u1;
    const double *v1;
    const double *du;
    int32_t n_edges;

    const int32_t *edge_start;
    const int32_t *edge_count;
    const double *poly_umin;
    const double *poly_umax;
    const double *poly_vmin;
    const double *poly_vmax;
    const uint8_t *fill_rule;
    int32_t n_shapes;

    const double *u_samples;
    const double *tick_samples;
    int32_t n_lut;
    int32_t invert_hw_scan_dir;
    int32_t total_ticks;

    const double *dx_facet;
    const double *dy_facet;
    int32_t n_facets;

    double u_size_mm;
    double v_size_mm;
    double lanewidth;
    double raw_u_last;
    double xstart_val;
    double slope_den;
    double y_shift_per_facet;
    double v_prism_start;
    double v_prism_end;
    double v_stage_start;
    double v_stage_end;
    double steps_per_line;
    int32_t bits_in_scanline;
    int32_t positiveresist;
    uint8_t default_intensity;
    uint8_t scanline_cmd;
    uint8_t spi_write_cmd;
} hx_scene_t;

typedef struct {
    int32_t lane_idx;
    int32_t facets_inlane;
} hx_lane_params_t;

/**
 * Compile all scanlines for one lane into framed 9-byte SPI write packets.
 *
 * Caller owns `*out_buf` and must release it with `free(*out_buf)`.
 */
int hx_slice_lane_spi(
    const hx_scene_t *scene,
    const hx_lane_params_t *lp,
    uint8_t **out_buf,
    size_t *out_len,
    int32_t *out_active_lines
);

#ifdef __cplusplus
}
#endif

#endif /* HEXASTORM_VECTOR_CORE_H */
