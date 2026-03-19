import math
from pathlib import Path
import json
import logging
from datetime import datetime

import cv2 as cv
import numpy as np

from ..config import Camera, PlatformConfig

logger = logging.getLogger(__name__)


def dicts_are_close(d1, d2, rel_tol=1e-5, abs_tol=1e-5):
    """Recursively checks if two dictionaries are almost equal, ignoring minor float differences."""
    if isinstance(d1, dict) and isinstance(d2, dict):
        if d1.keys() != d2.keys():
            return False
        return all(dicts_are_close(d1[k], d2[k], rel_tol, abs_tol) for k in d1)
    elif isinstance(d1, (int, float)) and isinstance(d2, (int, float)):
        return math.isclose(d1, d2, rel_tol=rel_tol, abs_tol=abs_tol)
    else:
        return d1 == d2


def append_history_record(filename, payload_dict, max_records=10000):
    """
    Appends a dictionary payload to a JSON history file.
    Automatically adds a timestamp and checks for duplicate data.
    """
    cfg = PlatformConfig(test=False)
    history_file = cfg.paths["calibration"] / filename

    # 1. Load existing history
    history = []
    if history_file.exists():
        try:
            with open(history_file, "r") as f:
                content = f.read()
                if content:
                    history = json.loads(content)
        except Exception as e:
            logger.error(f"Could not read existing history in {filename}: {e}")

    # 2. Check for Duplicate Content
    if history:
        # Copy the last record and remove the timestamp for a clean comparison
        last_record = history[-1].copy()
        last_record.pop("timestamp", None)

        if dicts_are_close(payload_dict, last_record):
            logger.warning(
                f"Result is identical to the last record in {filename}. Skipping save."
            )
            return

    # 3. Create the new record with a timestamp
    new_record = {"timestamp": datetime.now().isoformat()}
    new_record.update(payload_dict)

    # 4. Append, cap the limit, and save
    history.append(new_record)

    if len(history) > max_records:
        history = history[-max_records:]

    with open(history_file, "w") as f:
        json.dump(history, f, indent=2)

    logger.info(f"History updated: {history_file}")


def update_statistical_history(stats_report):
    """
    Appends the statistical calibration results to 'calibration_history.json'.
    Converts mean um shifts to mm for the slicer, while keeping the std/ptp
    data for quality tracking.
    """
    new_facets_data = {}

    for facet_key, data in stats_report.items():
        if "scan_mean_um" in data:
            new_facets_data[facet_key] = {
                # The slicer uses these core keys (converted to mm)
                "scan": round(data["scan_mean_um"] / 1000.0, 6),
                "orth": round(data["orth_mean_um"] / 1000.0, 6),
                "rotation_angle": data.get("rotation_angle_mean", 0.0),
                # Keep the stats for hardware health tracking
                "scan_std_um": data["scan_std_um"],
                "orth_std_um": data["orth_std_um"],
                "scan_ptp_um": data["scan_ptp_um"],
                "orth_ptp_um": data["orth_ptp_um"],
                # Add the spot metrics
                "spot_size_um": data.get("spot_size_mean", 0),
                "eccentricity": data.get("eccentricity_mean", 0),
                "samples": data["samples"],
            }

    payload = {
        "type": "statistical_calibration",
        "iterations": list(stats_report.values())[0]["samples"],
        "facets": new_facets_data,
    }

    append_history_record("calibration_history.json", payload)


def get_dots(
    img,
    pixelsize=Camera.DEFAULT_PIXEL_SIZE_UM,
    debug=False,
    min_diameter_um=10.0,
    max_diameter_um=200.0,
):
    """
    Analyzes an image to find laser dots.
    img: numpy array (loaded image).
    """
    if img is None:
        raise ValueError("Image data is None")

    if len(img.shape) == 2:
        gray = img
    else:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Ask Otsu to calculate the ideal threshold mathematically
    otsu_val, _ = cv.threshold(gray, 0, 255, cv.THRESH_OTSU)
    # Enforce a minimum floor (e.g., 50) so we don't pick up sensor noise
    final_val = max(otsu_val, 50)

    # 1. Use an adaptive threshold or a higher fixed one
    _, thresh = cv.threshold(gray, final_val, 255, cv.THRESH_BINARY)

    # Dynamic Morphological Closing ---
    # Calculate the minimum diameter in pixels
    min_diameter_px = min_diameter_um / pixelsize

    # Kernel size must be an odd integer (e.g., 3, 5, 7)
    kernel_size = int(np.ceil(min_diameter_px))
    if kernel_size % 2 == 0:
        kernel_size += 1

    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    # Apply the closing operation
    thresh = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)

    # 2. Find connected components
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Convert to pixel radius: (um_diameter / pixelsize) / 2
    min_radius_px = (min_diameter_um / pixelsize) / 2.0
    max_radius_px = (max_diameter_um / pixelsize) / 2.0

    # Calculate Area limits: A = pi * r^2
    min_area_px = np.pi * (min_radius_px**2)
    max_area_px = np.pi * (max_radius_px**2)

    # 3. Filter by size (area) to remove noise
    valid_dots = []
    dot_metrics = []
    for cnt in contours:
        area = cv.contourArea(cnt)

        # Check against calculated pixel area limits
        if min_area_px < area < max_area_px:
            # 1. Get the Centroid (Moment-based)
            M = cv.moments(cnt)
            if M["m00"] == 0:
                continue
            center = np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])

            # 2. Fit Ellipse (requires at least 5 points)
            if len(cnt) >= 5:
                el = cv.fitEllipse(cnt)
                axes = np.array(el[1]) * pixelsize  # Convert to micrometers
            else:
                # Fallback for very tiny dots (approximate from area)
                d_px = 2 * np.sqrt(area / np.pi)
                axes = np.array([d_px, d_px]) * pixelsize

            valid_dots.append(center)
            dot_metrics.append(
                {
                    "axes": axes,
                    "major_axis": max(axes),
                    "minor_axis": min(axes),
                    "eccentricity": max(axes) / min(axes),
                }
            )

    dots = np.array(valid_dots)

    if len(dots) == 0:
        return np.array([]), {}

    # 4. Sort by X-coordinate (Time-axis)
    dots = dots[dots[:, 0].argsort()]

    if debug:
        vis = img.copy()
        for i, (x, y) in enumerate(dots):
            cv.circle(vis, (int(x), int(y)), 4, (0, 255, 0), -1)
            cv.putText(
                vis,
                str(i),
                (int(x), int(y) - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1,
            )

        window_name = "dots_debug"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        h, w = img.shape[:2]
        cv.resizeWindow(window_name, int(w * 0.6), int(h * 0.6))
        cv.imshow(window_name, vis)
        logger.info("Debug image displayed. Click window and press key to continue.")
        cv.waitKey(0)
        cv.destroyAllWindows()

    if dot_metrics:
        facet_avg_size = round(np.mean([m["major_axis"] for m in dot_metrics]), 3)
        facet_avg_ecc = round(np.mean([m["eccentricity"] for m in dot_metrics]), 3)
        return dots, {
            "mean_spot_size_um": facet_avg_size,
            "mean_eccentricity": facet_avg_ecc,
        }
    else:
        return dots, {}


def compute_angle(dots):
    """Calculates the rotation angle of the dot line."""
    if len(dots) < 2:
        raise ValueError("At least 2 dots are required to compute an angle.")
    m, c = np.polyfit(dots[:, 0], dots[:, 1], 1)
    angle_rad = np.arctan(m)
    return np.degrees(angle_rad)


def rotate_dots(dots, angle_deg):
    """Rotates dots by the specified angle to align with the X axis."""
    if len(dots) == 0:
        raise ValueError("No dots to rotate.")
    angle_rad = np.radians(angle_deg)

    # We rotate by -angle_rad to flatten the line to y=0
    cos_a = np.cos(-angle_rad)
    sin_a = np.sin(-angle_rad)
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    return dots @ rotation_matrix.T


def calculate_facet_shifts(rect_dots_list):
    """
    Calculates shifts against a 'virtual reference'.
    which is the spatial median position to prevent highly unstable facets
    from skewing the baseline.
    """
    num_facets = len(rect_dots_list)
    if num_facets == 0 or any(len(f) < 3 for f in rect_dots_list):
        logger.error("Not enough valid dots across facets to calculate shifts.")
        return None, None

    # Physical constraints
    max_shift_um = 200.0
    max_shift_px = max_shift_um / abs(Camera.DEFAULT_PIXEL_SIZE_UM)

    # 1. Build the Virtual Reference Facet
    # Pick the facet with the most dots as the anchor.
    anchor_idx = max(range(num_facets), key=lambda i: len(rect_dots_list[i]))
    anchor_facet = rect_dots_list[anchor_idx]

    virtual_reference = []
    # Keep track of the aligned dots for each facet so we can compute math easily
    facet_matched_dots = {i: [] for i in range(num_facets)}

    for anchor_dot in anchor_facet:
        group = []
        matched_for_this_dot = {}
        is_valid_group = True

        # Find the corresponding dot in ALL facets (including the anchor itself)
        for i in range(num_facets):
            facet = rect_dots_list[i]
            if len(facet) == 0:
                is_valid_group = False
                break

            distances = np.linalg.norm(facet - anchor_dot, axis=1)
            closest_idx = np.argmin(distances)

            if distances[closest_idx] <= max_shift_px:
                matched_pt = facet[closest_idx]
                group.append(matched_pt)
                matched_for_this_dot[i] = matched_pt
            else:
                is_valid_group = False  # Dot drifted too far or is missing
                break

        if is_valid_group:
            # If a facet is wildly off, the median stays with the tight cluster of 1, 2, and 3.
            ideal_dot = np.median(group, axis=0)
            virtual_reference.append(ideal_dot)
            for i in range(num_facets):
                facet_matched_dots[i].append(matched_for_this_dot[i])

    virtual_reference = np.array(virtual_reference)

    if len(virtual_reference) < 3:
        logger.error(
            "Failed to build virtual reference: too few matching dots across all facets."
        )
        return None, None

    # 2. Calculate Shifts against the Virtual Reference
    facet_data = {}
    for i in range(num_facets):
        target_dots = np.array(facet_matched_dots[i])

        # Deviations from the ideal median
        x_diffs = target_dots[:, 0] - virtual_reference[:, 0]
        y_diffs = target_dots[:, 1] - virtual_reference[:, 1]

        facet_data[str(i)] = {
            "Scan_shift_um": round(
                float(np.mean(x_diffs) * Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
            "Orth_shift_um": round(
                float(np.mean(y_diffs) * Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
            "std_scan_um": round(
                float(np.std(x_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM)), 3
            ),
            "std_orth_um": round(
                float(np.std(y_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM)), 3
            ),
            "dots_matched": len(virtual_reference),
        }

    return facet_data, virtual_reference


def verify_calibration(
    image_paths,
    master_report,
    avg_angle_deg,
    rect_dots,
    virtual_ref,
    visual_debug=True,
):
    # 1. Create a Stacked Image (Average of all facets)
    base_img = None
    loaded_count = 0
    for path in image_paths:
        img = cv.imread(str(path), cv.IMREAD_GRAYSCALE)
        if img is None:
            continue

        if base_img is None:
            base_img = img.astype(np.float32)
        else:
            base_img += img.astype(np.float32)
        loaded_count += 1

    if base_img is None or loaded_count == 0:
        logger.warning("No images loaded for verification.")
        return

    base_img = (base_img / loaded_count).astype(np.uint8)

    # 2. Setup Inverse Rotation (De-rectification)
    angle_rad = np.radians(avg_angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    inv_rot_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    # 3. Create Visualization
    vis = (
        cv.cvtColor(base_img, cv.COLOR_GRAY2BGR)
        if len(base_img.shape) == 2
        else base_img.copy()
    )

    detected_colors = [(255, 255, 0), (0, 255, 0), (0, 165, 255), (0, 255, 255)]
    outlier_color = (0, 0, 255)
    predicted_color = (255, 0, 255)

    # --- Plot DETECTED dots ---
    for i in range(len(rect_dots)):
        if i >= len(detected_colors):
            break

        report_key = str(i)
        valid_mask = master_report.get(report_key, {}).get(
            "valid_mask", [True] * len(rect_dots[i])
        )

        # De-rectify: Rotate back to original image space
        detected_dots_img_space = rect_dots[i] @ inv_rot_matrix.T

        for pt_idx, pt in enumerate(detected_dots_img_space):
            is_valid = valid_mask[pt_idx] if pt_idx < len(valid_mask) else True

            if is_valid:
                cv.circle(vis, (int(pt[0]), int(pt[1])), 6, detected_colors[i], 1)
            else:
                cv.circle(vis, (int(pt[0]), int(pt[1])), 6, outlier_color, 2)
                cv.drawMarker(
                    vis,
                    (int(pt[0]), int(pt[1])),
                    outlier_color,
                    cv.MARKER_CROSS,
                    markerSize=10,
                    thickness=2,
                )

    # --- Plot PREDICTED dots (Against the Virtual Reference) ---
    if virtual_ref is not None:
        for i in range(len(rect_dots)):
            report_key = str(i)
            if "Scan_shift_um" not in master_report.get(report_key, {}):
                continue

            shifted_rect = virtual_ref.copy()
            px_size = Camera.DEFAULT_PIXEL_SIZE_UM

            # Re-apply the shift to the virtual mean to predict the spot
            shifted_rect[:, 0] += master_report[report_key]["Scan_shift_um"] / px_size
            shifted_rect[:, 1] += master_report[report_key]["Orth_shift_um"] / px_size

            predicted_dots = shifted_rect @ inv_rot_matrix.T

            for pt in predicted_dots:
                cv.drawMarker(
                    vis,
                    (int(pt[0]), int(pt[1])),
                    predicted_color,
                    cv.MARKER_TILTED_CROSS,
                    markerSize=8,
                    thickness=1,
                )

    if visual_debug:
        window_name = "Calibration Verification"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        h, w = vis.shape[:2]
        cv.resizeWindow(window_name, int(w * 0.7), int(h * 0.7))
        cv.imshow(window_name, vis)
        logger.info(
            "Verification image displayed. Press any key in the window to close."
        )
        cv.waitKey(0)
        cv.destroyAllWindows()
        cv.imwrite("calibration_verification.jpg", vis)


def calibration(
    image_dir=None,
    filename_pattern="facet{}.jpg",
    debug=False,
    min_diameter_mm=0.01,
    max_diameter_mm=0.20,
    compute_rotation=True,  # Toggle: True for 1D lines, False for 2D grids
):
    """
    Orchestrates the loading, analysis, and verification of laser calibration images.

    This function processes images containing either a 1D line or a 2D grid of laser dots
    to calculate spatial deviations for each facet of the laser system. The resulting
    calibration data can then be used by the slicer to correct physical distortions.

    **Calibration Flow:**
    1. **Dot Detection:** Loads one image per facet (via `filename_pattern`) and extracts
       dot centroids by finding contours within the `min_diameter_mm` to `max_diameter_mm` range.
    2. **Rotation Alignment (1D Mode):** If the laser head is rotated relative to the camera,
       this is detected and corrected so the scanline is mathematically flat.
    3. **Virtual Reference Calculation:** Averages the aligned dot patterns across all
       facets to create a "perfect" target pattern.
    4. **Error Measurement:** Calculates the deviation—both parallel (scan) and orthogonal—of
       each facet's dots against the virtual reference.
    5. **Slicer Verification (2D Mode):** To verify the slicer's correction, a 2D grid
       pattern (multiple stacked scanlines) is generated. Here, `compute_rotation` is disabled,
       and the function measures pure translational errors to confirm that distortion has decreased.

    Args:
        image_dir (str | Path, optional): Directory containing the images. Defaults to config path.
        filename_pattern (str): String format to locate facet images. Defaults to "facet{}.jpg".
        debug (bool): If True, pauses execution to display visual debugging windows.
        store_log (bool): If True, appends the payload to 'calibration_history.json'.
        min_diameter_mm (float): Minimum physical diameter of a valid dot in millimeters.
        max_diameter_mm (float): Maximum physical diameter of a valid dot in millimeters.
        compute_rotation (bool): Toggle for calibration mode. True for 1D single lines
            (computes rotation); False for 2D stacked grids (pure translation).

    Returns:
        dict: A master report mapping each facet index to its calculated metrics:
            `Scan_shift_um`, `Orth_shift_um`, `rotation_angle_deg`, `mean_spot_size_um`,
            and `mean_eccentricity`.
    """
    cfg = PlatformConfig(test=False)
    num_facets = cfg.laser_timing["facets"]
    if image_dir is None:
        image_dir = cfg.paths["images"]
    else:
        image_dir = Path(image_dir)

    mode_str = "1D Line" if compute_rotation else "2D Grid"
    logger.debug(f"Starting {mode_str} Calibration Analysis on: {image_dir}")

    min_diameter_um = min_diameter_mm * 1000.0
    max_diameter_um = max_diameter_mm * 1000.0

    image_paths = [image_dir / filename_pattern.format(i) for i in range(num_facets)]

    all_raw_dots = []
    all_spot_stats = []
    all_angles = []

    # 1. Process each image to find dots
    for i, path in enumerate(image_paths):
        logger.debug(f"Processing Facet {i}...")
        path = path.resolve()
        img = cv.imread(str(path))

        dots_facet, stat = get_dots(
            img,
            debug=debug,
            max_diameter_um=max_diameter_um,
            min_diameter_um=min_diameter_um,
        )

        all_raw_dots.append(dots_facet)
        all_spot_stats.append(stat)

        # Compute angle only if requested
        if compute_rotation:
            angle = compute_angle(dots_facet)
            all_angles.append(angle)

    # 2. Handle Rotation Logic
    if compute_rotation:
        global_angle = np.median(all_angles)
        logger.debug(f"Global median rotation angle calculated: {global_angle:.4f}")
        all_processed_dots = [rotate_dots(dots, global_angle) for dots in all_raw_dots]
    else:
        global_angle = 0.0
        all_processed_dots = all_raw_dots  # Use raw 2D grid dots directly

    # 3. Calculate Shifts against the virtual mean
    shift_data, virtual_ref = calculate_facet_shifts(all_processed_dots)

    if shift_data is None:
        logger.error("Shift calculation failed.")
        return {}

    # 4. Create Master Report
    master_report = {}
    for i in range(num_facets):
        s_data = shift_data[str(i)]
        spot_data = all_spot_stats[i]

        # If we computed rotation, report the individual angle, else 0.0
        ind_angle = all_angles[i] if compute_rotation else 0.0
        angle_data = {"rotation_angle_deg": round(ind_angle, 4)}

        master_report[str(i)] = {**s_data, **spot_data, **angle_data}

    # 6. Verify Visuals
    verify_calibration(
        image_paths,
        master_report,
        global_angle,
        all_processed_dots,
        virtual_ref,
        visual_debug=debug,
    )

    return master_report


if __name__ == "__main__":
    from ..log_setup import configure_logging

    configure_logging(logging.INFO)

    # Run the analysis, passing the size thresholds in mm
    results = calibration(
        image_dir=None,  # Use default from config
        filename_pattern="facet{}.jpg",
        debug=False,
        min_diameter_mm=0.01,  # Equivalent to 10 um
        max_diameter_mm=0.20,  # Equivalent to 200 um
    )

    # Convert dictionary to a pretty-printed string
    logger.info(f"Calibration Results:\n{json.dumps(results, indent=4)}")
