#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdlib.h>
#include <string.h>

#include "vector_core.h"

typedef struct {
    Py_buffer view;
    int acquired;
} hx_buf_t;

static int acquire_contig_1d(
    PyObject *obj,
    hx_buf_t *out,
    Py_ssize_t expected_itemsize,
    Py_ssize_t expected_len,
    const char *name
) {
    if (PyObject_GetBuffer(obj, &out->view, PyBUF_SIMPLE | PyBUF_C_CONTIGUOUS) != 0) {
        return -1;
    }
    out->acquired = 1;
    if ((out->view.len % expected_itemsize) != 0) {
        PyErr_Format(
            PyExc_ValueError,
            "%s buffer size (%zd) is not a multiple of %zd bytes",
            name,
            out->view.len,
            expected_itemsize
        );
        return -1;
    }
    Py_ssize_t count = out->view.len / expected_itemsize;
    if (expected_len >= 0 && count != expected_len) {
        PyErr_Format(
            PyExc_ValueError,
            "%s length (%zd) does not match expected length (%zd)",
            name,
            count,
            expected_len
        );
        return -1;
    }
    return 0;
}

static PyObject *py_slice_lane_spi(PyObject *self, PyObject *args, PyObject *kwargs) {
    (void)self;

    static char *kwlist[] = {
        "u0",
        "v0",
        "u1",
        "v1",
        "du",
        "edge_start",
        "edge_count",
        "poly_umin",
        "poly_umax",
        "poly_vmin",
        "poly_vmax",
        "fill_rule",
        "u_samples",
        "tick_samples",
        "dx_facet",
        "dy_facet",
        "invert_hw_scan_dir",
        "total_ticks",
        "lane_idx",
        "facets_inlane",
        "bits_in_scanline",
        "steps_per_line",
        "u_size_mm",
        "v_size_mm",
        "lanewidth",
        "raw_u_last",
        "xstart_val",
        "slope_den",
        "y_shift_per_facet",
        "v_prism_start",
        "v_prism_end",
        "v_stage_start",
        "v_stage_end",
        "positiveresist",
        "default_intensity",
        "scanline_cmd",
        "spi_write_cmd",
        NULL
    };

    PyObject *u0_obj = NULL, *v0_obj = NULL, *u1_obj = NULL, *v1_obj = NULL, *du_obj = NULL;
    PyObject *es_obj = NULL, *ec_obj = NULL;
    PyObject *pumin_obj = NULL, *pumax_obj = NULL, *pvmin_obj = NULL, *pvmax_obj = NULL, *fr_obj = NULL;
    PyObject *usamp_obj = NULL, *tsamp_obj = NULL;
    PyObject *dxf_obj = NULL, *dyf_obj = NULL;

    int invert_hw_scan_dir = 1;
    int total_ticks = 0;
    int lane_idx = 0;
    int facets_inlane = 0;
    int bits_in_scanline = 0;
    double steps_per_line = 0.0;
    double u_size_mm = 0.0;
    double v_size_mm = 0.0;
    double lanewidth = 0.0;
    double raw_u_last = 0.0;
    double xstart_val = 0.0;
    double slope_den = 0.0;
    double y_shift_per_facet = 0.0;
    double v_prism_start = 0.0;
    double v_prism_end = 0.0;
    double v_stage_start = 0.0;
    double v_stage_end = 0.0;
    int positiveresist = 0;
    unsigned char default_intensity = 255;
    unsigned char scanline_cmd = 3;
    unsigned char spi_write_cmd = 2;

    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OOOOOOOOOOOOOOOOiiiiiddddddddddddpBBB",
            kwlist,
            &u0_obj,
            &v0_obj,
            &u1_obj,
            &v1_obj,
            &du_obj,
            &es_obj,
            &ec_obj,
            &pumin_obj,
            &pumax_obj,
            &pvmin_obj,
            &pvmax_obj,
            &fr_obj,
            &usamp_obj,
            &tsamp_obj,
            &dxf_obj,
            &dyf_obj,
            &invert_hw_scan_dir,
            &total_ticks,
            &lane_idx,
            &facets_inlane,
            &bits_in_scanline,
            &steps_per_line,
            &u_size_mm,
            &v_size_mm,
            &lanewidth,
            &raw_u_last,
            &xstart_val,
            &slope_den,
            &y_shift_per_facet,
            &v_prism_start,
            &v_prism_end,
            &v_stage_start,
            &v_stage_end,
            &positiveresist,
            &default_intensity,
            &scanline_cmd,
            &spi_write_cmd
        )) {
        return NULL;
    }

    hx_buf_t bufs[16];
    memset(bufs, 0, sizeof(bufs));
    PyObject *result = NULL;

    if (acquire_contig_1d(u0_obj, &bufs[0], (Py_ssize_t)sizeof(double), -1, "u0") != 0) {
        goto cleanup;
    }
    Py_ssize_t n_edges = bufs[0].view.len / (Py_ssize_t)sizeof(double);
    if (acquire_contig_1d(v0_obj, &bufs[1], (Py_ssize_t)sizeof(double), n_edges, "v0") != 0 ||
        acquire_contig_1d(u1_obj, &bufs[2], (Py_ssize_t)sizeof(double), n_edges, "u1") != 0 ||
        acquire_contig_1d(v1_obj, &bufs[3], (Py_ssize_t)sizeof(double), n_edges, "v1") != 0 ||
        acquire_contig_1d(du_obj, &bufs[4], (Py_ssize_t)sizeof(double), n_edges, "du") != 0) {
        goto cleanup;
    }

    if (acquire_contig_1d(es_obj, &bufs[5], (Py_ssize_t)sizeof(int32_t), -1, "edge_start") != 0) {
        goto cleanup;
    }
    Py_ssize_t n_shapes = bufs[5].view.len / (Py_ssize_t)sizeof(int32_t);
    if (acquire_contig_1d(ec_obj, &bufs[6], (Py_ssize_t)sizeof(int32_t), n_shapes, "edge_count") != 0 ||
        acquire_contig_1d(pumin_obj, &bufs[7], (Py_ssize_t)sizeof(double), n_shapes, "poly_umin") != 0 ||
        acquire_contig_1d(pumax_obj, &bufs[8], (Py_ssize_t)sizeof(double), n_shapes, "poly_umax") != 0 ||
        acquire_contig_1d(pvmin_obj, &bufs[9], (Py_ssize_t)sizeof(double), n_shapes, "poly_vmin") != 0 ||
        acquire_contig_1d(pvmax_obj, &bufs[10], (Py_ssize_t)sizeof(double), n_shapes, "poly_vmax") != 0 ||
        acquire_contig_1d(fr_obj, &bufs[11], (Py_ssize_t)sizeof(uint8_t), n_shapes, "fill_rule") != 0) {
        goto cleanup;
    }

    if (acquire_contig_1d(usamp_obj, &bufs[12], (Py_ssize_t)sizeof(double), -1, "u_samples") != 0) {
        goto cleanup;
    }
    Py_ssize_t n_lut = bufs[12].view.len / (Py_ssize_t)sizeof(double);
    if (acquire_contig_1d(tsamp_obj, &bufs[13], (Py_ssize_t)sizeof(double), n_lut, "tick_samples") != 0) {
        goto cleanup;
    }

    if (acquire_contig_1d(dxf_obj, &bufs[14], (Py_ssize_t)sizeof(double), -1, "dx_facet") != 0) {
        goto cleanup;
    }
    Py_ssize_t n_facets = bufs[14].view.len / (Py_ssize_t)sizeof(double);
    if (acquire_contig_1d(dyf_obj, &bufs[15], (Py_ssize_t)sizeof(double), n_facets, "dy_facet") != 0) {
        goto cleanup;
    }

    hx_scene_t scene;
    scene.u0 = (const double *)bufs[0].view.buf;
    scene.v0 = (const double *)bufs[1].view.buf;
    scene.u1 = (const double *)bufs[2].view.buf;
    scene.v1 = (const double *)bufs[3].view.buf;
    scene.du = (const double *)bufs[4].view.buf;
    scene.n_edges = (int32_t)n_edges;

    scene.edge_start = (const int32_t *)bufs[5].view.buf;
    scene.edge_count = (const int32_t *)bufs[6].view.buf;
    scene.poly_umin = (const double *)bufs[7].view.buf;
    scene.poly_umax = (const double *)bufs[8].view.buf;
    scene.poly_vmin = (const double *)bufs[9].view.buf;
    scene.poly_vmax = (const double *)bufs[10].view.buf;
    scene.fill_rule = (const uint8_t *)bufs[11].view.buf;
    scene.n_shapes = (int32_t)n_shapes;

    scene.u_samples = (const double *)bufs[12].view.buf;
    scene.tick_samples = (const double *)bufs[13].view.buf;
    scene.n_lut = (int32_t)n_lut;
    scene.invert_hw_scan_dir = invert_hw_scan_dir;
    scene.total_ticks = total_ticks;

    scene.dx_facet = (const double *)bufs[14].view.buf;
    scene.dy_facet = (const double *)bufs[15].view.buf;
    scene.n_facets = (int32_t)n_facets;

    scene.u_size_mm = u_size_mm;
    scene.v_size_mm = v_size_mm;
    scene.lanewidth = lanewidth;
    scene.raw_u_last = raw_u_last;
    scene.xstart_val = xstart_val;
    scene.slope_den = slope_den;
    scene.y_shift_per_facet = y_shift_per_facet;
    scene.v_prism_start = v_prism_start;
    scene.v_prism_end = v_prism_end;
    scene.v_stage_start = v_stage_start;
    scene.v_stage_end = v_stage_end;
    scene.steps_per_line = steps_per_line;
    scene.bits_in_scanline = bits_in_scanline;
    scene.positiveresist = positiveresist;
    scene.default_intensity = default_intensity;
    scene.scanline_cmd = scanline_cmd;
    scene.spi_write_cmd = spi_write_cmd;

    hx_lane_params_t lp;
    lp.lane_idx = lane_idx;
    lp.facets_inlane = facets_inlane;

    uint8_t *out_buf = NULL;
    size_t out_len = 0;
    int32_t active_lines = 0;
    int rc;

    Py_BEGIN_ALLOW_THREADS
    rc = hx_slice_lane_spi(&scene, &lp, &out_buf, &out_len, &active_lines);
    Py_END_ALLOW_THREADS

    if (rc != HX_OK) {
        free(out_buf);
        if (rc == HX_ERR_NOMEM) {
            PyErr_NoMemory();
        } else if (rc == HX_ERR_STEP_PERIOD) {
            PyErr_SetString(
                PyExc_ValueError,
                "Steps per line cannot be achieved (period < 1)"
            );
        } else if (rc == HX_ERR_TOO_MANY_EVENTS) {
            PyErr_SetString(PyExc_ValueError, "Too many events in scanline");
        } else {
            PyErr_SetString(PyExc_ValueError, "Invalid arguments to slice_lane_spi");
        }
        goto cleanup;
    }

    PyObject *bytes_obj = PyBytes_FromStringAndSize(
        out_buf ? (const char *)out_buf : "",
        (Py_ssize_t)out_len
    );
    free(out_buf);
    if (!bytes_obj) {
        goto cleanup;
    }

    result = Py_BuildValue("(Ni)", bytes_obj, (int)active_lines);

cleanup:
    for (int i = 0; i < 16; ++i) {
        if (bufs[i].acquired) {
            PyBuffer_Release(&bufs[i].view);
        }
    }
    return result;
}

static PyMethodDef VectorCoreMethods[] = {
    {
        "slice_lane_spi",
        (PyCFunction)py_slice_lane_spi,
        METH_VARARGS | METH_KEYWORDS,
        "Slice one lane into raw framed SPI scanline packets."
    },
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef vector_core_module = {
    PyModuleDef_HEAD_INIT,
    "_vector_core",
    "Portable C99 scanline-polygon intersection kernel for Hexastorm LDI.",
    -1,
    VectorCoreMethods
};

PyMODINIT_FUNC PyInit__vector_core(void) {
    return PyModule_Create(&vector_core_module);
}
