#include "py/mpconfig.h"
#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include <stdint.h>
#include <stdlib.h>

#include "vector_core.h"

static size_t acquire_contig_1d(
    mp_obj_t obj,
    mp_buffer_info_t *out,
    size_t expected_itemsize,
    ssize_t expected_len,
    const char *name
) {
    mp_get_buffer_raise(obj, out, MP_BUFFER_READ);
    if ((out->len % expected_itemsize) != 0) {
        mp_raise_msg_varg(
            &mp_type_ValueError,
            MP_ERROR_TEXT("%s buffer size (%u) is not a multiple of %u bytes"),
            name,
            (unsigned)out->len,
            (unsigned)expected_itemsize
        );
    }
    size_t count = out->len / expected_itemsize;
    if (expected_len >= 0 && count != (size_t)expected_len) {
        mp_raise_msg_varg(
            &mp_type_ValueError,
            MP_ERROR_TEXT("%s length (%u) does not match expected length (%u)"),
            name,
            (unsigned)count,
            (unsigned)expected_len
        );
    }
    return count;
}

static uint8_t get_u8_arg(mp_obj_t obj, const char *name) {
    mp_int_t val = mp_obj_get_int(obj);
    if (val < 0 || val > 255) {
        mp_raise_msg_varg(
            &mp_type_ValueError,
            MP_ERROR_TEXT("%s value (%d) out of uint8 range"),
            name,
            (int)val
        );
    }
    return (uint8_t)val;
}

static mp_obj_t mod_vector_core_slice_lane_spi(size_t n_args, const mp_obj_t *args) {
    (void)n_args;

    mp_buffer_info_t bufs[16];
    size_t n_edges = acquire_contig_1d(args[0], &bufs[0], sizeof(double), -1, "u0");
    acquire_contig_1d(args[1], &bufs[1], sizeof(double), (ssize_t)n_edges, "v0");
    acquire_contig_1d(args[2], &bufs[2], sizeof(double), (ssize_t)n_edges, "u1");
    acquire_contig_1d(args[3], &bufs[3], sizeof(double), (ssize_t)n_edges, "v1");
    acquire_contig_1d(args[4], &bufs[4], sizeof(double), (ssize_t)n_edges, "du");

    size_t n_shapes = acquire_contig_1d(args[5], &bufs[5], sizeof(int32_t), -1, "edge_start");
    acquire_contig_1d(args[6], &bufs[6], sizeof(int32_t), (ssize_t)n_shapes, "edge_count");
    acquire_contig_1d(args[7], &bufs[7], sizeof(double), (ssize_t)n_shapes, "poly_umin");
    acquire_contig_1d(args[8], &bufs[8], sizeof(double), (ssize_t)n_shapes, "poly_umax");
    acquire_contig_1d(args[9], &bufs[9], sizeof(double), (ssize_t)n_shapes, "poly_vmin");
    acquire_contig_1d(args[10], &bufs[10], sizeof(double), (ssize_t)n_shapes, "poly_vmax");
    acquire_contig_1d(args[11], &bufs[11], sizeof(uint8_t), (ssize_t)n_shapes, "fill_rule");

    size_t n_lut = acquire_contig_1d(args[12], &bufs[12], sizeof(double), -1, "u_samples");
    acquire_contig_1d(args[13], &bufs[13], sizeof(double), (ssize_t)n_lut, "tick_samples");

    size_t n_facets = acquire_contig_1d(args[14], &bufs[14], sizeof(double), -1, "dx_facet");
    acquire_contig_1d(args[15], &bufs[15], sizeof(double), (ssize_t)n_facets, "dy_facet");

    const int32_t *edge_start_ptr = (const int32_t *)bufs[5].buf;
    const int32_t *edge_count_ptr = (const int32_t *)bufs[6].buf;
    for (size_t si = 0; si < n_shapes; ++si) {
        int32_t es = edge_start_ptr[si];
        int32_t ec = edge_count_ptr[si];
        if (es < 0 || ec < 0 || (size_t)es + (size_t)ec > n_edges) {
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid shape edge_start/edge_count bounds"));
        }
    }

    hx_scene_t scene;
    scene.u0 = (const double *)bufs[0].buf;
    scene.v0 = (const double *)bufs[1].buf;
    scene.u1 = (const double *)bufs[2].buf;
    scene.v1 = (const double *)bufs[3].buf;
    scene.du = (const double *)bufs[4].buf;
    scene.n_edges = (int32_t)n_edges;

    scene.edge_start = edge_start_ptr;
    scene.edge_count = edge_count_ptr;
    scene.poly_umin = (const double *)bufs[7].buf;
    scene.poly_umax = (const double *)bufs[8].buf;
    scene.poly_vmin = (const double *)bufs[9].buf;
    scene.poly_vmax = (const double *)bufs[10].buf;
    scene.fill_rule = (const uint8_t *)bufs[11].buf;
    scene.n_shapes = (int32_t)n_shapes;

    scene.u_samples = (const double *)bufs[12].buf;
    scene.tick_samples = (const double *)bufs[13].buf;
    scene.n_lut = (int32_t)n_lut;
    scene.invert_hw_scan_dir = (int32_t)mp_obj_get_int(args[16]);
    scene.total_ticks = (int32_t)mp_obj_get_int(args[17]);

    scene.dx_facet = (const double *)bufs[14].buf;
    scene.dy_facet = (const double *)bufs[15].buf;
    scene.n_facets = (int32_t)n_facets;

    hx_lane_params_t lp;
    lp.lane_idx = (int32_t)mp_obj_get_int(args[18]);
    lp.facets_inlane = (int32_t)mp_obj_get_int(args[19]);

    scene.steps_per_line = (double)mp_obj_get_float(args[20]);
    scene.u_size_mm = (double)mp_obj_get_float(args[21]);
    scene.v_size_mm = (double)mp_obj_get_float(args[22]);
    scene.lanewidth = (double)mp_obj_get_float(args[23]);
    scene.raw_u_last = (double)mp_obj_get_float(args[24]);
    scene.xstart_val = (double)mp_obj_get_float(args[25]);
    scene.slope_den = (double)mp_obj_get_float(args[26]);
    scene.y_shift_per_facet = (double)mp_obj_get_float(args[27]);
    scene.v_prism_start = (double)mp_obj_get_float(args[28]);
    scene.v_prism_end = (double)mp_obj_get_float(args[29]);
    scene.v_stage_start = (double)mp_obj_get_float(args[30]);
    scene.v_stage_end = (double)mp_obj_get_float(args[31]);
    scene.positiveresist = mp_obj_is_true(args[32]) ? 1 : 0;
    scene.default_intensity = get_u8_arg(args[33], "default_intensity");
    scene.scanline_cmd = get_u8_arg(args[34], "scanline_cmd");
    scene.spi_write_cmd = get_u8_arg(args[35], "spi_write_cmd");

    uint8_t *out_buf = NULL;
    size_t out_len = 0;
    int32_t active_lines = 0;
    int rc = hx_slice_lane_spi(&scene, &lp, &out_buf, &out_len, &active_lines);

    if (rc != HX_OK) {
        free(out_buf);
        if (rc == HX_ERR_NOMEM) {
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Out of memory in slice_lane_spi"));
        } else if (rc == HX_ERR_STEP_PERIOD) {
            mp_raise_ValueError(MP_ERROR_TEXT("Steps per line cannot be achieved (period < 1)"));
        } else if (rc == HX_ERR_TOO_MANY_EVENTS) {
            mp_raise_ValueError(MP_ERROR_TEXT("Too many events in scanline"));
        } else {
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid arguments to slice_lane_spi"));
        }
    }

    MP_DEFINE_NLR_JUMP_CALLBACK_FUNCTION_1(free_ctx, free, out_buf);
    nlr_push_jump_callback(&free_ctx.callback, mp_call_function_1_from_nlr_jump_callback);
    mp_obj_t bytes_obj = mp_obj_new_bytes(
        out_buf ? out_buf : (const uint8_t *)"",
        out_len
    );
    nlr_pop_jump_callback(true);

    mp_obj_t tuple[2] = {
        bytes_obj,
        mp_obj_new_int(active_lines),
    };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_vector_core_slice_lane_spi_obj,
    36,
    36,
    mod_vector_core_slice_lane_spi
);

static const mp_rom_map_elem_t vector_core_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__vector_core) },
    { MP_ROM_QSTR(MP_QSTR_slice_lane_spi), MP_ROM_PTR(&mod_vector_core_slice_lane_spi_obj) },
};
static MP_DEFINE_CONST_DICT(vector_core_module_globals, vector_core_module_globals_table);

const mp_obj_module_t vector_core_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&vector_core_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__vector_core, vector_core_user_cmodule);
