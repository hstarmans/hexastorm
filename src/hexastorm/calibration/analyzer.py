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


def update_calibration_history(master_report):
    """
    Appends the new calibration results to 'calibration_history.json'
    ONLY if the data is different from the previous run.
    """
    # 1. Prepare the new data payload (without timestamp yet)
    new_facets_data = {}

    # Iterate through facets in the report
    for facet_key, data in master_report.items():
        if "Scan_shift_um" in data:
            new_facets_data[facet_key] = {
                "scan": round(data["Scan_shift_um"] / 1000.0, 6),
                "orth": round(data["Orth_shift_um"] / 1000.0, 6),
                "rotation_angle": data.get("rotation_angle_deg", 0.0),
                "spot_size_um": data.get("mean_spot_size_um", 0),
                "eccentricity": data.get("mean_eccentricity", 0),
            }

    payload = {"facets": new_facets_data}
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
    Calculates shifts against a 'virtual reference' which is the spatial mean
    of matching dots across all facets.
    """
    num_facets = len(rect_dots_list)
    if num_facets == 0 or any(len(f) < 3 for f in rect_dots_list):
        logger.error("Not enough valid dots across facets to calculate shifts.")
        return None, None

    # Physical constraints
    max_shift_um = 200.0
    max_shift_px = max_shift_um / abs(Camera.DEFAULT_PIXEL_SIZE_UM)

    # 1. Build the Virtual Reference Facet
    # Use the first facet as an anchor just to identify matching dot groupings
    anchor_facet = rect_dots_list[0]
    virtual_reference = []

    # Keep track of the aligned dots for each facet so we can compute math easily
    facet_matched_dots = {i: [] for i in range(num_facets)}

    for anchor_dot in anchor_facet:
        group = [anchor_dot]
        matched_for_this_dot = {0: anchor_dot}
        is_valid_group = True

        # Find the corresponding dot in the other facets
        for i in range(1, num_facets):
            facet = rect_dots_list[i]
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
            # Calculate the MEAN spatial position for this dot across all facets
            mean_dot = np.mean(group, axis=0)
            virtual_reference.append(mean_dot)
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

        # Deviations from the ideal mean
        x_diffs = target_dots[:, 0] - virtual_reference[:, 0]
        y_diffs = target_dots[:, 1] - virtual_reference[:, 1]

        facet_data[str(i)] = {
            "Scan_shift_um": round(np.mean(x_diffs) * Camera.DEFAULT_PIXEL_SIZE_UM, 3),
            "Orth_shift_um": round(np.mean(y_diffs) * Camera.DEFAULT_PIXEL_SIZE_UM, 3),
            "std_scan_um": round(
                np.std(x_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
            "std_orth_um": round(
                np.std(y_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
            "dots_matched": len(virtual_reference),
        }

    return facet_data, virtual_reference


def calculate_2d_grid_shifts(ref_dots, target_dots, pixel_size_um):
    """
    Calculates the X and Y translation shifts between two 2D point clouds
    (e.g., a reference grid and a target grid) using nearest-neighbor matching.
    Returns the average shift in micrometers.
    """
    if len(ref_dots) == 0 or len(target_dots) == 0:
        logger.error("Empty dot array provided for 2D shift calculation.")
        return 0.0, 0.0

    x_diffs = []
    y_diffs = []

    # Physical constraint to avoid matching a dot to the WRONG grid neighbor
    # (Assuming spacing is 300um, a shift of > 150um is physically impossible
    # without aliasing onto the next dot).
    max_shift_px = 150.0 / abs(pixel_size_um)

    for target_pt in target_dots:
        distances = np.linalg.norm(ref_dots - target_pt, axis=1)
        closest_idx = np.argmin(distances)

        if distances[closest_idx] <= max_shift_px:
            dx_px = target_pt[0] - ref_dots[closest_idx][0]
            dy_px = target_pt[1] - ref_dots[closest_idx][1]
            x_diffs.append(dx_px)
            y_diffs.append(dy_px)

    if not x_diffs:
        logger.error("No valid matching dots found within the 150um physical limit.")
        return 0.0, 0.0

    mean_shift_x_um = np.mean(x_diffs) * abs(pixel_size_um)
    mean_shift_y_um = np.mean(y_diffs) * abs(pixel_size_um)

    return mean_shift_x_um, mean_shift_y_um


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
        img = cv.imread(str(path))
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
    num_facets=4,
    filename_pattern="facet{}.jpg",
    debug=False,
    store_log=True,
    min_diameter_mm=0.01,  # Default 0.01 mm (10 um)
    max_diameter_mm=0.20,  # Default 0.20 mm (200 um)
):
    """
    Orchestrates the loading, analysis, and verification of calibration images.
    """
    cfg = PlatformConfig(test=False)
    num_facets = cfg.laser_timing["facets"]
    if image_dir is None:
        image_dir = cfg.paths["images"]
    else:
        image_dir = Path(image_dir)

    logger.info(f"Starting Calibration Analysis on: {image_dir}")

    # --- Convert mm inputs to microns for internal processing ---
    min_diameter_um = min_diameter_mm * 1000.0
    max_diameter_um = max_diameter_mm * 1000.0

    image_paths = []
    for i in range(num_facets):
        p = image_dir / filename_pattern.format(i)
        image_paths.append(p)

    all_raw_dots = []
    all_spot_stats = []
    all_angles = []

    # 1. Process each image to find dots and individual angles
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

        angle = compute_angle(dots_facet)
        all_angles.append(angle)

    # 2. Find the Global Truth (Median Angle)
    global_angle = np.median(all_angles)
    logger.debug(f"Global median rotation angle calculated: {global_angle:.4f}")

    # 3. Apply Global Rotation
    all_rect_dots = [rotate_dots(dots, global_angle) for dots in all_raw_dots]

    # 4. Calculate Shifts against the virtual mean
    shift_data, virtual_ref = calculate_facet_shifts(all_rect_dots)

    # 5. Create Master Report
    master_report = {}
    for i in range(num_facets):
        s_data = shift_data[str(i)]
        spot_data = all_spot_stats[i]
        angle_data = {"rotation_angle_deg": round(all_angles[i], 4)}
        master_report[str(i)] = {**s_data, **spot_data, **angle_data}

    # Save History optionally
    if store_log:
        update_calibration_history(master_report)

    # Verify Visuals
    verify_calibration(
        image_paths,
        master_report,
        global_angle,
        all_rect_dots,
        virtual_ref,
        visual_debug=debug,
    )

    return master_report


if __name__ == "__main__":
    from ..log_setup import configure_logging

    configure_logging(logging.DEBUG)

    # Run the analysis, passing the size thresholds in mm
    results = calibration(
        image_dir=None,  # Use default from config
        filename_pattern="facet{}.jpg",
        debug=False,
        store_log=False,  # Set to True to save to calibration_history.json
        min_diameter_mm=0.01,  # Equivalent to 10 um
        max_diameter_mm=0.20,  # Equivalent to 200 um
    )

    # Convert dictionary to a pretty-printed string
    logger.info(f"Calibration Results:\n{json.dumps(results, indent=4)}")
