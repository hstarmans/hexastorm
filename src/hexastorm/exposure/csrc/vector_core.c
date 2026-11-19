#include "vector_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HX_EPS 1e-9
#define HX_LANE_EPS 1e-6
#define HX_SENTINEL_TICK 0xFFFFFFu

typedef struct {
    double u;
    int32_t wind;
    int32_t idx;
} hx_hit_t;

typedef struct {
    double a;
    double b;
} hx_interval_d_t;

typedef struct {
    int32_t a;
    int32_t b;
} hx_interval_i_t;

typedef struct {
    uint32_t tick;
    uint8_t intensity;
} hx_event_t;

static int ensure_cap(void **buf, size_t *cap, size_t needed, size_t elem_size) {
    if (needed <= *cap) {
        return 0;
    }
    size_t new_cap = (*cap == 0) ? 64 : *cap;
    while (new_cap < needed) {
        if (new_cap > (SIZE_MAX / 2)) {
            return -1;
        }
        new_cap *= 2;
    }
    if (new_cap > (SIZE_MAX / elem_size)) {
        return -1;
    }
    void *tmp = realloc(*buf, new_cap * elem_size);
    if (!tmp) {
        return -1;
    }
    *buf = tmp;
    *cap = new_cap;
    return 0;
}

static int cmp_hit(const void *pa, const void *pb) {
    const hx_hit_t *a = (const hx_hit_t *)pa;
    const hx_hit_t *b = (const hx_hit_t *)pb;
    if (a->u < b->u) {
        return -1;
    }
    if (a->u > b->u) {
        return 1;
    }
    return (a->idx < b->idx) ? -1 : ((a->idx > b->idx) ? 1 : 0);
}

static void sort_hits(hx_hit_t *hits, size_t n) {
    if (n <= 1) {
        return;
    }
    if (n <= 32) {
        for (size_t i = 1; i < n; ++i) {
            hx_hit_t key = hits[i];
            size_t j = i;
            while (j > 0 && hits[j - 1].u > key.u) {
                hits[j] = hits[j - 1];
                --j;
            }
            hits[j] = key;
        }
        return;
    }
    qsort(hits, n, sizeof(hx_hit_t), cmp_hit);
}

static int cmp_iv_d(const void *pa, const void *pb) {
    const hx_interval_d_t *x = (const hx_interval_d_t *)pa;
    const hx_interval_d_t *y = (const hx_interval_d_t *)pb;
    if (x->a < y->a) {
        return -1;
    }
    if (x->a > y->a) {
        return 1;
    }
    return 0;
}

static void sort_intervals_d(hx_interval_d_t *ivs, size_t n) {
    if (n <= 1) {
        return;
    }
    if (n <= 32) {
        for (size_t i = 1; i < n; ++i) {
            hx_interval_d_t key = ivs[i];
            size_t j = i;
            while (j > 0 && ivs[j - 1].a > key.a) {
                ivs[j] = ivs[j - 1];
                --j;
            }
            ivs[j] = key;
        }
        return;
    }
    qsort(ivs, n, sizeof(hx_interval_d_t), cmp_iv_d);
}

static int cmp_iv_i(const void *pa, const void *pb) {
    const hx_interval_i_t *x = (const hx_interval_i_t *)pa;
    const hx_interval_i_t *y = (const hx_interval_i_t *)pb;
    if (x->a != y->a) {
        return (x->a < y->a) ? -1 : 1;
    }
    if (x->b != y->b) {
        return (x->b < y->b) ? -1 : 1;
    }
    return 0;
}

static void sort_intervals_i(hx_interval_i_t *ivs, size_t n) {
    if (n <= 1) {
        return;
    }
    if (n <= 32) {
        for (size_t i = 1; i < n; ++i) {
            hx_interval_i_t key = ivs[i];
            size_t j = i;
            while (j > 0 && cmp_iv_i(&ivs[j - 1], &key) > 0) {
                ivs[j] = ivs[j - 1];
                --j;
            }
            ivs[j] = key;
        }
        return;
    }
    qsort(ivs, n, sizeof(hx_interval_i_t), cmp_iv_i);
}

static int32_t lut_u_to_tick(const hx_scene_t *s, double u_base_mm) {
    const int32_t n = s->n_lut;
    const double *u_arr = s->u_samples;
    const double *t_arr = s->tick_samples;
    double t;

    if (u_base_mm <= u_arr[0]) {
        t = t_arr[0];
    } else if (u_base_mm >= u_arr[n - 1]) {
        t = t_arr[n - 1];
    } else {
        int32_t lo = 0;
        int32_t hi = n - 1;
        while (hi - lo > 1) {
            int32_t mid = lo + ((hi - lo) >> 1);
            if (u_arr[mid] <= u_base_mm) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        double u_lo = u_arr[lo];
        double u_hi = u_arr[lo + 1];
        double denom = u_hi - u_lo;
        double frac = (denom > 0.0) ? ((u_base_mm - u_lo) / denom) : 0.0;
        t = t_arr[lo] + frac * (t_arr[lo + 1] - t_arr[lo]);
    }

    if (!s->invert_hw_scan_dir) {
        t = (double)s->total_ticks - t;
    }

    long long tick = llrint(t);
    if (tick < 0) {
        tick = 0;
    } else if (tick > (long long)s->total_ticks) {
        tick = (long long)s->total_ticks;
    }
    return (int32_t)tick;
}

static void write_spi_word64_be(uint8_t *dst, uint8_t write_cmd, uint64_t word64) {
    dst[0] = write_cmd;
    dst[1] = (uint8_t)((word64 >> 56) & 0xFFu);
    dst[2] = (uint8_t)((word64 >> 48) & 0xFFu);
    dst[3] = (uint8_t)((word64 >> 40) & 0xFFu);
    dst[4] = (uint8_t)((word64 >> 32) & 0xFFu);
    dst[5] = (uint8_t)((word64 >> 24) & 0xFFu);
    dst[6] = (uint8_t)((word64 >> 16) & 0xFFu);
    dst[7] = (uint8_t)((word64 >> 8) & 0xFFu);
    dst[8] = (uint8_t)(word64 & 0xFFu);
}

int hx_slice_lane_spi(
    const hx_scene_t *scene,
    const hx_lane_params_t *lp,
    uint8_t **out_buf,
    size_t *out_len,
    int32_t *out_active_lines
) {
    if (!scene || !lp || !out_buf || !out_len || !out_active_lines) {
        return HX_ERR_INVALID_ARG;
    }
    if (scene->n_lut < 2 || scene->n_facets <= 0 || lp->facets_inlane < 0) {
        return HX_ERR_INVALID_ARG;
    }

    int rc = HX_OK;
    *out_buf = NULL;
    *out_len = 0;
    *out_active_lines = 0;

    uint64_t half_period;
    if (scene->steps_per_line <= 0.0) {
        half_period = (scene->bits_in_scanline > 1) ? (uint64_t)scene->bits_in_scanline : 1ULL;
    } else {
        double hp = floor((double)(scene->bits_in_scanline - 1) / (scene->steps_per_line * 2.0));
        if (hp < 1.0) {
            return HX_ERR_STEP_PERIOD;
        }
        half_period = (uint64_t)hp;
    }

    int32_t *lane_shapes = NULL;
    uint32_t *shape_stamp = NULL;
    int32_t *bucket_counts = NULL;
    int32_t *bucket_offsets = NULL;
    int32_t *bucket_items = NULL;

    hx_hit_t *hits = NULL;
    size_t hits_cap = 0;
    hx_interval_d_t *all_ivs = NULL;
    size_t all_ivs_cap = 0;
    hx_interval_d_t *merged_ivs = NULL;
    size_t merged_ivs_cap = 0;
    hx_interval_i_t *tick_ivs = NULL;
    size_t tick_ivs_cap = 0;
    hx_event_t *events = NULL;
    size_t events_cap = 0;

    uint8_t *spi_buf = NULL;
    size_t spi_len = 0;
    size_t spi_cap = 0;

    const int32_t lane_idx = lp->lane_idx;
    const int32_t facets_inlane = lp->facets_inlane;
    const int is_forward = (lane_idx % 2 == 0);
    const uint64_t direction = is_forward ? 1ULL : 0ULL;
    const double dir_sign = is_forward ? 1.0 : -1.0;
    const double lane_u_offset = (double)lane_idx * scene->lanewidth;
    const double u_base_origin = scene->raw_u_last + scene->xstart_val + lane_u_offset;

    /* 1. Lane culling on U */
    double min_dx = scene->dx_facet[0];
    double max_dx = scene->dx_facet[0];
    for (int32_t f = 1; f < scene->n_facets; ++f) {
        if (scene->dx_facet[f] < min_dx) min_dx = scene->dx_facet[f];
        if (scene->dx_facet[f] > max_dx) max_dx = scene->dx_facet[f];
    }
    double min_u_s = u_base_origin + min_dx;
    double max_u_s = u_base_origin + max_dx;
    double lane_u_min = (scene->slope_den >= 0.0) ? min_u_s : (min_u_s + scene->slope_den);
    double lane_u_max = (scene->slope_den >= 0.0) ? (max_u_s + scene->slope_den) : max_u_s;
    if (lane_u_min < 0.0) lane_u_min = 0.0;
    if (lane_u_max > scene->u_size_mm) lane_u_max = scene->u_size_mm;

    int32_t n_lane_shapes = 0;
    double max_u_overhang = 0.0;
    if (scene->n_shapes > 0 && lane_u_max > lane_u_min) {
        lane_shapes = (int32_t *)malloc((size_t)scene->n_shapes * sizeof(int32_t));
        if (!lane_shapes) {
            rc = HX_ERR_NOMEM;
            goto cleanup;
        }
        for (int32_t si = 0; si < scene->n_shapes; ++si) {
            double smin = scene->poly_umin[si];
            double smax = scene->poly_umax[si];
            if (smax < lane_u_min - HX_LANE_EPS || smin > lane_u_max + HX_LANE_EPS) {
                continue;
            }
            lane_shapes[n_lane_shapes++] = si;
            double ext_l = lane_u_min - smin;
            double ext_r = smax - lane_u_max;
            if (ext_l > max_u_overhang) max_u_overhang = ext_l;
            if (ext_r > max_u_overhang) max_u_overhang = ext_r;
        }
    }

    /* 2. Build vertical V-bucket spatial index over surviving lane shapes */
    int32_t n_buckets = 1;
    double inv_bucket_v = 0.0;
    if (n_lane_shapes > 0 && scene->v_size_mm > 0.0) {
        n_buckets = facets_inlane / 16;
        if (n_buckets < 1) n_buckets = 1;
        if (n_buckets > 1024) n_buckets = 1024;
        inv_bucket_v = (double)n_buckets / scene->v_size_mm;

        shape_stamp = (uint32_t *)calloc((size_t)n_lane_shapes, sizeof(uint32_t));
        bucket_counts = (int32_t *)calloc((size_t)n_buckets, sizeof(int32_t));
        bucket_offsets = (int32_t *)malloc((size_t)(n_buckets + 1) * sizeof(int32_t));
        if (!shape_stamp || !bucket_counts || !bucket_offsets) {
            rc = HX_ERR_NOMEM;
            goto cleanup;
        }

        for (int32_t li = 0; li < n_lane_shapes; ++li) {
            int32_t si = lane_shapes[li];
            int32_t b0 = (int32_t)floor(scene->poly_vmin[si] * inv_bucket_v);
            int32_t b1 = (int32_t)floor(scene->poly_vmax[si] * inv_bucket_v);
            if (b0 < 0) b0 = 0;
            if (b1 >= n_buckets) b1 = n_buckets - 1;
            for (int32_t b = b0; b <= b1; ++b) {
                bucket_counts[b] += 1;
            }
        }

        bucket_offsets[0] = 0;
        for (int32_t b = 0; b < n_buckets; ++b) {
            bucket_offsets[b + 1] = bucket_offsets[b] + bucket_counts[b];
            bucket_counts[b] = 0;
        }

        int32_t total_refs = bucket_offsets[n_buckets];
        if (total_refs > 0) {
            bucket_items = (int32_t *)malloc((size_t)total_refs * sizeof(int32_t));
            if (!bucket_items) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            for (int32_t li = 0; li < n_lane_shapes; ++li) {
                int32_t si = lane_shapes[li];
                int32_t b0 = (int32_t)floor(scene->poly_vmin[si] * inv_bucket_v);
                int32_t b1 = (int32_t)floor(scene->poly_vmax[si] * inv_bucket_v);
                if (b0 < 0) b0 = 0;
                if (b1 >= n_buckets) b1 = n_buckets - 1;
                for (int32_t b = b0; b <= b1; ++b) {
                    int32_t pos = bucket_offsets[b] + bucket_counts[b]++;
                    bucket_items[pos] = li;
                }
            }
        }
    }

    if (ensure_cap((void **)&spi_buf, &spi_cap, (size_t)facets_inlane * 27 + 64, 1) != 0) {
        rc = HX_ERR_NOMEM;
        goto cleanup;
    }

    int32_t active_lines = 0;
    uint32_t stamp = 0;

    for (int32_t f_lane_idx = 0; f_lane_idx < facets_inlane; ++f_lane_idx) {
        int32_t facet_idx = f_lane_idx % scene->n_facets;
        double dx_f = scene->dx_facet[facet_idx];
        double dy_f = scene->dy_facet[facet_idx];

        double ystart_mm = is_forward
            ? ((double)f_lane_idx * scene->y_shift_per_facet)
            : ((double)(facets_inlane - f_lane_idx) * scene->y_shift_per_facet);

        double u_shift = u_base_origin + dx_f;
        double u_start = u_shift;
        double u_end = u_shift + scene->slope_den;
        double win_u_min = (u_start > 0.0) ? u_start : 0.0;
        double win_u_max = (u_end < scene->u_size_mm) ? u_end : scene->u_size_mm;

        double v_start = scene->v_prism_start + dy_f + dir_sign * scene->v_stage_start + ystart_mm;
        double v_end = scene->v_prism_end + dy_f + dir_sign * scene->v_stage_end + ystart_mm;
        double slope = (fabs(scene->slope_den) > 1e-12) ? ((v_end - v_start) / scene->slope_den) : 0.0;

        double v_scan_min = (v_start < v_end) ? v_start : v_end;
        double v_scan_max = (v_start > v_end) ? v_start : v_end;
        int in_v_bounds = (v_scan_max >= 0.0) && (v_scan_min < scene->v_size_mm);

        size_t n_all_ivs = 0;
        if (win_u_max > win_u_min && (in_v_bounds || scene->positiveresist) && n_lane_shapes > 0 && bucket_items != NULL) {
            ++stamp;
            double v_pad = fabs(slope) * (fabs(scene->slope_den) + max_u_overhang) + 1e-6;
            int32_t b0 = (int32_t)floor((v_scan_min - v_pad) * inv_bucket_v);
            int32_t b1 = (int32_t)floor((v_scan_max + v_pad) * inv_bucket_v);
            if (b0 < 0) b0 = 0;
            if (b1 >= n_buckets) b1 = n_buckets - 1;

            for (int32_t b = b0; b <= b1; ++b) {
                int32_t start_pos = bucket_offsets[b];
                int32_t end_pos = bucket_offsets[b + 1];
                for (int32_t k = start_pos; k < end_pos; ++k) {
                    int32_t li = bucket_items[k];
                    if (shape_stamp[li] == stamp) {
                        continue;
                    }
                    shape_stamp[li] = stamp;

                    int32_t si = lane_shapes[li];
                    double v_u0 = v_start + slope * (scene->poly_umin[si] - u_start);
                    double v_u1 = v_start + slope * (scene->poly_umax[si] - u_start);
                    double vmin_line = (v_u0 < v_u1) ? v_u0 : v_u1;
                    double vmax_line = (v_u0 > v_u1) ? v_u0 : v_u1;
                    if (scene->poly_vmax[si] < vmin_line - HX_EPS || scene->poly_vmin[si] > vmax_line + HX_EPS) {
                        continue;
                    }

                    int32_t e0 = scene->edge_start[si];
                    int32_t ec = scene->edge_count[si];
                    if (ec <= 0) {
                        continue;
                    }
                    if (ensure_cap((void **)&hits, &hits_cap, (size_t)ec, sizeof(hx_hit_t)) != 0) {
                        rc = HX_ERR_NOMEM;
                        goto cleanup;
                    }

                    size_t n_hits = 0;
                    for (int32_t ei = 0; ei < ec; ++ei) {
                        int32_t idx = e0 + ei;
                        double w0 = (scene->v0[idx] - v_start) - slope * (scene->u0[idx] - u_start);
                        double w1 = (scene->v1[idx] - v_start) - slope * (scene->u1[idx] - u_start);
                        int c0 = (w0 <= 0.0);
                        int c1 = (w1 <= 0.0);
                        if (c0 != c1) {
                            double dw = w1 - w0;
                            double alpha = -w0 / dw;
                            hits[n_hits].u = scene->u0[idx] + alpha * scene->du[idx];
                            hits[n_hits].wind = c0 ? 1 : -1;
                            hits[n_hits].idx = (int32_t)n_hits;
                            ++n_hits;
                        }
                    }

                    if (n_hits < 2) {
                        continue;
                    }
                    sort_hits(hits, n_hits);

                    if (scene->fill_rule[si] == HX_FILL_EVENODD) {
                        size_t pairs = (n_hits / 2) * 2;
                        if (ensure_cap((void **)&all_ivs, &all_ivs_cap, n_all_ivs + pairs / 2, sizeof(hx_interval_d_t)) != 0) {
                            rc = HX_ERR_NOMEM;
                            goto cleanup;
                        }
                        for (size_t hi = 0; hi < pairs; hi += 2) {
                            double a = hits[hi].u;
                            double b_val = hits[hi + 1].u;
                            if (b_val > a + HX_EPS) {
                                all_ivs[n_all_ivs].a = a;
                                all_ivs[n_all_ivs].b = b_val;
                                ++n_all_ivs;
                            }
                        }
                    } else {
                        int32_t acc = 0;
                        double enter_u = 0.0;
                        if (ensure_cap((void **)&all_ivs, &all_ivs_cap, n_all_ivs + n_hits, sizeof(hx_interval_d_t)) != 0) {
                            rc = HX_ERR_NOMEM;
                            goto cleanup;
                        }
                        for (size_t hi = 0; hi < n_hits; ++hi) {
                            int32_t prev_acc = acc;
                            acc += hits[hi].wind;
                            if (prev_acc == 0 && acc != 0) {
                                enter_u = hits[hi].u;
                            } else if (prev_acc != 0 && acc == 0 && hits[hi].u > enter_u + HX_EPS) {
                                all_ivs[n_all_ivs].a = enter_u;
                                all_ivs[n_all_ivs].b = hits[hi].u;
                                ++n_all_ivs;
                            }
                        }
                    }
                }
            }
        }

        /* Merge scanline intervals */
        size_t n_merged = 0;
        if (n_all_ivs > 0) {
            sort_intervals_d(all_ivs, n_all_ivs);
            if (ensure_cap((void **)&merged_ivs, &merged_ivs_cap, n_all_ivs, sizeof(hx_interval_d_t)) != 0) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            merged_ivs[0] = all_ivs[0];
            n_merged = 1;
            for (size_t ii = 1; ii < n_all_ivs; ++ii) {
                double s_val = all_ivs[ii].a;
                double e_val = all_ivs[ii].b;
                if (s_val <= merged_ivs[n_merged - 1].b + HX_EPS) {
                    if (e_val > merged_ivs[n_merged - 1].b) {
                        merged_ivs[n_merged - 1].b = e_val;
                    }
                } else {
                    merged_ivs[n_merged].a = s_val;
                    merged_ivs[n_merged].b = e_val;
                    ++n_merged;
                }
            }
        }

        /* Optional positiveresist inversion against [0.0, u_size_mm] */
        if (scene->positiveresist) {
            if (ensure_cap((void **)&all_ivs, &all_ivs_cap, n_merged + 2, sizeof(hx_interval_d_t)) != 0) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            size_t n_inv = 0;
            double cursor = 0.0;
            for (size_t ii = 0; ii < n_merged; ++ii) {
                double a = merged_ivs[ii].a;
                double b_val = merged_ivs[ii].b;
                if (b_val <= cursor) {
                    continue;
                }
                if (a > cursor) {
                    double right = (a < scene->u_size_mm) ? a : scene->u_size_mm;
                    if (right > cursor + HX_EPS) {
                        all_ivs[n_inv].a = cursor;
                        all_ivs[n_inv].b = right;
                        ++n_inv;
                    }
                }
                if (b_val > cursor) {
                    cursor = b_val;
                }
                if (cursor >= scene->u_size_mm) {
                    break;
                }
            }
            if (cursor < scene->u_size_mm - HX_EPS) {
                all_ivs[n_inv].a = cursor;
                all_ivs[n_inv].b = scene->u_size_mm;
                ++n_inv;
            }
            if (ensure_cap((void **)&merged_ivs, &merged_ivs_cap, n_inv, sizeof(hx_interval_d_t)) != 0) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            if (n_inv > 0) {
                memcpy(merged_ivs, all_ivs, n_inv * sizeof(hx_interval_d_t));
            }
            n_merged = n_inv;
        }

        /* Convert intervals to ticks */
        size_t n_ticks = 0;
        if (n_merged > 0 && win_u_max > win_u_min) {
            if (ensure_cap((void **)&tick_ivs, &tick_ivs_cap, n_merged, sizeof(hx_interval_i_t)) != 0) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            for (size_t ii = 0; ii < n_merged; ++ii) {
                double ca = (merged_ivs[ii].a > win_u_min) ? merged_ivs[ii].a : win_u_min;
                double cb = (merged_ivs[ii].b < win_u_max) ? merged_ivs[ii].b : win_u_max;
                if (cb <= ca) {
                    continue;
                }
                int32_t ta = lut_u_to_tick(scene, ca - u_shift);
                int32_t tb = lut_u_to_tick(scene, cb - u_shift);
                int32_t t_on = (ta <= tb) ? ta : tb;
                int32_t t_off = (ta <= tb) ? tb : ta;
                if (t_off > t_on) {
                    tick_ivs[n_ticks].a = t_on;
                    tick_ivs[n_ticks].b = t_off;
                    ++n_ticks;
                }
            }
        }

        /* Merge tick intervals and emit events */
        size_t n_events = 0;
        if (n_ticks > 0) {
            sort_intervals_i(tick_ivs, n_ticks);
            if (ensure_cap((void **)&events, &events_cap, n_ticks * 2, sizeof(hx_event_t)) != 0) {
                rc = HX_ERR_NOMEM;
                goto cleanup;
            }
            int32_t cur_on = tick_ivs[0].a;
            int32_t cur_off = tick_ivs[0].b;
            for (size_t ti = 1; ti < n_ticks; ++ti) {
                if (tick_ivs[ti].a <= cur_off) {
                    if (tick_ivs[ti].b > cur_off) {
                        cur_off = tick_ivs[ti].b;
                    }
                } else {
                    events[n_events].tick = (uint32_t)cur_on;
                    events[n_events].intensity = scene->default_intensity;
                    ++n_events;
                    events[n_events].tick = (uint32_t)cur_off;
                    events[n_events].intensity = 0;
                    ++n_events;
                    cur_on = tick_ivs[ti].a;
                    cur_off = tick_ivs[ti].b;
                }
            }
            events[n_events].tick = (uint32_t)cur_on;
            events[n_events].intensity = scene->default_intensity;
            ++n_events;
            events[n_events].tick = (uint32_t)cur_off;
            events[n_events].intensity = 0;
            ++n_events;
        }

        if (n_events > 0) {
            ++active_lines;
        }

        size_t event_words = (n_events + 1) / 2;
        if (event_words > 0xFFFFu) {
            rc = HX_ERR_TOO_MANY_EVENTS;
            goto cleanup;
        }

        size_t line_bytes = (1 + event_words) * 9;
        if (ensure_cap((void **)&spi_buf, &spi_cap, spi_len + line_bytes, 1) != 0) {
            rc = HX_ERR_NOMEM;
            goto cleanup;
        }

        uint64_t hdr_word = ((half_period & ((1ULL << 39) - 1ULL)) << 25)
            | (((uint64_t)event_words & 0xFFFFULL) << 9)
            | ((direction & 1ULL) << 8)
            | (uint64_t)scene->scanline_cmd;
        write_spi_word64_be(spi_buf + spi_len, scene->spi_write_cmd, hdr_word);
        spi_len += 9;

        for (size_t wi = 0; wi < event_words; ++wi) {
            uint64_t ev0 = (((uint64_t)events[2 * wi].tick & 0xFFFFFFULL) << 8)
                | (uint64_t)events[2 * wi].intensity;
            uint64_t ev1;
            if (2 * wi + 1 < n_events) {
                ev1 = (((uint64_t)events[2 * wi + 1].tick & 0xFFFFFFULL) << 8)
                    | (uint64_t)events[2 * wi + 1].intensity;
            } else {
                ev1 = ((uint64_t)HX_SENTINEL_TICK) << 8;
            }
            uint64_t word64 = ev0 | (ev1 << 32);
            write_spi_word64_be(spi_buf + spi_len, scene->spi_write_cmd, word64);
            spi_len += 9;
        }
    }

    *out_buf = spi_buf;
    *out_len = spi_len;
    *out_active_lines = active_lines;
    spi_buf = NULL;

cleanup:
    free(lane_shapes);
    free(shape_stamp);
    free(bucket_counts);
    free(bucket_offsets);
    free(bucket_items);
    free(hits);
    free(all_ivs);
    free(merged_ivs);
    free(tick_ivs);
    free(events);
    free(spi_buf);
    return rc;
}
