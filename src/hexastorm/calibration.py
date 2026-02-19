import cv2 as cv
import numpy as np
from pathlib import Path
import json
import logging
from datetime import datetime

from .config import Camera, PlatformConfig

logger = logging.getLogger(__name__)


def update_calibration_history(master_report):
    """
    Appends the new calibration results to 'calibration_history.json'
    ONLY if the data is different from the previous run.
    """
    cfg = PlatformConfig(test=False)
    history_file = cfg.paths["calibration"] / "calibration_history.json"

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

    # 2. Load existing history
    history = []
    if history_file.exists():
        try:
            with open(history_file, "r") as f:
                content = f.read()
                if content:
                    history = json.loads(content)
        except Exception as e:
            logger.error(f"Could not read existing history: {e}")

    # 3. Check for Duplicate Content
    if history:
        last_record = history[-1]
        last_facets = last_record.get("facets", {})

        # Direct dictionary comparison checks if keys and values are identical
        if new_facets_data == last_facets:
            logger.warning(
                "Calibration result is identical to the last record. Skipping save."
            )
            return

    # 4. If different, add timestamp and save
    timestamp = datetime.now().isoformat()
    new_record = {"timestamp": timestamp, "facets": new_facets_data}

    history.append(new_record)

    if len(history) > 10000:
        history = history[-10000:]

    with open(history_file, "w") as f:
        json.dump(history, f, indent=2)
    logger.info(f"Calibration history updated: {history_file}")


def get_dots(img, pixelsize=Camera.DEFAULT_PIXEL_SIZE_UM, debug=False):
    """
    Analyzes an image to find laser dots.
    img: numpy array (loaded image).
    """
    if img is None:
        raise ValueError("Image data is None")

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # 1. Use an adaptive threshold or a higher fixed one
    _, thresh = cv.threshold(gray, 100, 255, cv.THRESH_BINARY)

    # 2. Find connected components
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Calculate Area Limits based on Physical Size ---
    # Expected limits: 10um to 200um diameter
    min_diameter_um = 10.0
    max_diameter_um = 200.0

    # Convert to pixel radius: (um_diameter / pixelsize) / 2
    min_radius_px = (min_diameter_um / pixelsize) / 2.0
    max_radius_px = (max_diameter_um / pixelsize) / 2.0

    # Calculate Area limits: A = pi * r^2
    min_area_px = np.pi * (min_radius_px**2)
    max_area_px = np.pi * (max_radius_px**2)
    # ---------------------------------------------------------

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
                # d = 2 * sqrt(area/pi) -> mapped to axes
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
        # Create a color visualization
        vis = img.copy()
        for i, (x, y) in enumerate(dots):
            # Draw dot center (sub-pixel)
            cv.circle(vis, (int(x), int(y)), 4, (0, 255, 0), -1)
            # Label the dot index
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

    # Calculate global metrics for the whole facet
    if dot_metrics:
        facet_avg_size = round(np.mean([m["major_axis"] for m in dot_metrics]), 3)
        facet_avg_ecc = round(np.mean([m["eccentricity"] for m in dot_metrics]), 3)
        return dots, {
            "mean_spot_size_um": facet_avg_size,
            "mean_eccentricity": facet_avg_ecc,
        }
    else:
        return dots, {}


def rectify_dots(dots):
    """
    dots: np.array of shape (n, 2) -> [[x, y], ...]
    Returns: rectified_dots, angle_deg
    """
    if len(dots) < 2:
        return dots, 0.0

    x = dots[:, 0]
    y = dots[:, 1]

    # 1. Fit line to find the angle of the scan path
    m, c = np.polyfit(x, y, 1)
    angle_rad = np.arctan(m)

    # 2. Create Rotation Matrix
    cos_a = np.cos(-angle_rad)
    sin_a = np.sin(-angle_rad)
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    # 3. Rotate all points so the line becomes horizontal
    rectified = dots @ rotation_matrix.T

    return rectified, np.degrees(angle_rad)


def calculate_facet_shifts(rect_dots_list):
    # Reference facet is index 0
    ref_facet = rect_dots_list[0]
    num_dots_ref = len(ref_facet)

    facet_data = {}

    for i, facet in enumerate(rect_dots_list):
        # 1. Sanity Check: Dot Count
        if len(facet) != num_dots_ref:
            logging.error(
                f"Facet {i} has {len(facet)} dots, but Facet 0 has {num_dots_ref}. Skipping."
            )
            facet_data[i] = {"error": "dot_count_mismatch"}
            continue

        # 2. Compute Timing Shift (X-axis)
        x_diffs = facet[:, 0] - ref_facet[:, 0]
        mean_x_shift = np.mean(x_diffs)

        # 3. Compute Orthogonal Shift (Y-axis)
        y_diffs = facet[:, 1] - ref_facet[:, 1]
        mean_y_shift = np.mean(y_diffs)

        facet_data[i] = {
            "Scan_shift_um": round(mean_x_shift * Camera.DEFAULT_PIXEL_SIZE_UM, 3),
            "Orth_shift_um": round(mean_y_shift * Camera.DEFAULT_PIXEL_SIZE_UM, 3),
            "std_scan_um": round(
                np.std(x_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
            "std_orth_um": round(
                np.std(y_diffs) * abs(Camera.DEFAULT_PIXEL_SIZE_UM), 3
            ),
        }

    return facet_data


def verify_calibration(
    image_paths, master_report, avg_angle_deg, rect_dots, visual_debug=False
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

    detected_colors = [(255, 255, 0), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
    predicted_color = (0, 0, 255)  # Red

    # --- Plot DETECTED dots ---
    for i in range(len(rect_dots)):
        if i >= len(detected_colors):
            break
        # De-rectify: Rotate back to original image space
        detected_dots_img_space = rect_dots[i] @ inv_rot_matrix.T

        for pt in detected_dots_img_space:
            cv.circle(vis, (int(pt[0]), int(pt[1])), 6, detected_colors[i], 1)

    # --- Plot PREDICTED dots ---
    ref_rect = rect_dots[0]
    for i in range(len(rect_dots)):
        report_key = str(i)
        if "Scan_shift_um" not in master_report[report_key]:
            continue

        shifted_rect = ref_rect.copy()
        px_size = Camera.DEFAULT_PIXEL_SIZE_UM
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
        # Show the result
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


def run_full_calibration_analysis(
    image_dir=None, num_facets=4, filename_pattern="facet{}.jpg", debug=False
):
    """
    Orchestrates the loading, analysis, and verification of calibration images.
    """
    cfg = PlatformConfig(test=False)

    # Safely default to the standard image pipeline folder
    if image_dir is None:
        image_dir = cfg.paths["images"]
    else:
        image_dir = Path(image_dir)

    logger.info(f"Starting Calibration Analysis on: {image_dir}")

    image_paths = []
    # Verify paths exist before starting heavy processing
    for i in range(num_facets):
        p = image_dir / filename_pattern.format(i)
        if not p.exists():
            logger.error(f"Missing expected file: {p}")
            return
        image_paths.append(p)

    all_rect_dots = []
    all_spot_stats = []
    all_angles = []

    # 1. Process each image
    for i, path in enumerate(image_paths):
        logger.debug(f"Processing Facet {i}...")

        # Resolve absolute path to ensure cv.imread works
        path = path.resolve()
        img = cv.imread(str(path))

        if img is None:
            logger.error(f"Could not load image at {path}")
            # Append empty placeholders to maintain index alignment
            all_rect_dots.append(np.array([]))
            all_spot_stats.append({})
            all_angles.append(0)
            continue

        dots_facet, stat = get_dots(img, debug=debug)

        if len(dots_facet) == 0:
            logger.warning(f"No dots found in {path.name}")
            all_rect_dots.append(np.array([]))
            all_spot_stats.append({})
            all_angles.append(0)
            continue

        all_spot_stats.append(stat)
        rect_dot, angle = rectify_dots(dots_facet)
        all_angles.append(angle)
        all_rect_dots.append(rect_dot)

    # 2. Calculate Shifts
    if all_rect_dots and len(all_rect_dots[0]) > 0:
        shift_data = calculate_facet_shifts(all_rect_dots)

        # 3. Create Master Report
        master_report = {}
        for i in range(num_facets):
            s_data = shift_data.get(i, {})
            spot_data = all_spot_stats[i] if i < len(all_spot_stats) else {}
            angle_data = (
                {"rotation_angle_deg": round(all_angles[i], 4)}
                if i < len(all_angles)
                else {}
            )
            master_report[str(i)] = {**s_data, **spot_data, **angle_data}

        # 4. Save History (No more markdown/json reports in local folder)
        update_calibration_history(master_report)

        # 5. Verify Visuals
        valid_angles = [a for a in all_angles if a != 0]
        avg_angle = np.mean(valid_angles) if valid_angles else 0
        verify_calibration(
            image_paths, master_report, avg_angle, all_rect_dots, visual_debug=debug
        )
    else:
        logger.error("Insufficient data to calculate shifts (Facet 0 empty?).")

    return master_report


if __name__ == "__main__":
    from .log_setup import configure_logging

    configure_logging(logging.DEBUG)

    run_full_calibration_analysis(
        image_dir=None,  # Use default from config
        num_facets=4,
        filename_pattern="facet{}.jpg",
        debug=False,
    )
