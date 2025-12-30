import cv2 as cv
import numpy as np
from pathlib import Path

# OV2311 is a 2-Megapixel (1600 x 1300)
# pixelsize is 3.0 μm x 3.0 μm
pixelsize = 3

def get_dots(img_path, pixelsize=3, debug=False):
    base_path = Path(__file__).parent
    img_path = base_path / img_path
    # Load image
    img = cv.imread(str(img_path))
    if img is None:
        raise ValueError(f"Could not load image at {img_path}")
    
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # 1. Use an adaptive threshold or a higher fixed one 
    # Laser dots often have a 'halo'; we want the core.
    _, thresh = cv.threshold(gray, 100, 255, cv.THRESH_BINARY)
    
    # 2. Find connected components
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    # 3. Filter by size (area) to remove single-pixel noise
    # stats[i, cv.CC_STAT_AREA] gives the pixel count
    valid_dots = []
    dot_metrics = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if 5 < area < 1000:
            # 1. Get the Centroid (Moment-based)
            M = cv.moments(cnt)
            if M["m00"] == 0: continue
            center = np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])
            
            # 2. Fit Ellipse (requires at least 5 points)
            if len(cnt) >= 5:
                # el = ((center_x, center_y), (width, height), angle)
                el = cv.fitEllipse(cnt)
                axes = np.array(el[1]) * pixelsize # Convert to micrometers
            else:
                # Fallback for very tiny dots
                axes = np.array([np.sqrt(area), np.sqrt(area)]) * pixelsize

            valid_dots.append(center)
            dot_metrics.append({
                "axes": axes,
                "major_axis": max(axes),
                "minor_axis": min(axes),
                "eccentricity": max(axes) / min(axes)
            })
    
    dots = np.array(valid_dots)
    
    if len(dots) == 0:
        return np.array([])

    # 4. Sort by X-coordinate (Time-axis)
    dots = dots[dots[:, 0].argsort()]

    if debug:
        # Create a color visualization
        vis = img.copy()
        for i, (x, y) in enumerate(dots):
            # Draw dot center (sub-pixel)
            cv.circle(vis, (int(x), int(y)), 4, (0, 255, 0), -1)
            # Label the dot index
            cv.putText(vis, str(i), (int(x), int(y) - 10), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        window_name = "Debug Dots (Press any key to close)"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        h, w = img.shape[:2]
        cv.resizeWindow(window_name, int(w * 0.6), int(h * 0.6))
        cv.imshow(window_name, vis)
        print("Image displayed. Click the image window and press any key (like 'Esc' or 'Space') to continue...")
        cv.waitKey(0)  # 0 means wait indefinitely for a keypress
        cv.destroyAllWindows()

    # Calculate global metrics for the whole facet
    facet_avg_size = round(np.mean([m["major_axis"] for m in dot_metrics]), 3)
    facet_avg_ecc = round(np.mean([m["eccentricity"] for m in dot_metrics]), 3)
    
    return dots, {"mean_spot_size_um": facet_avg_size, "mean_eccentricity": facet_avg_ecc}


def rectify_dots(dots):
    """
    dots: np.array of shape (n, 2) -> [[x, y], ...]
    Returns: rectified_dots, angle_deg
    """
    x = dots[:, 0]
    y = dots[:, 1]
    
    # 1. Fit line to find the angle of the scan path
    m, c = np.polyfit(x, y, 1)
    angle_rad = np.arctan(m)
    
    # 2. Create Rotation Matrix
    cos_a = np.cos(-angle_rad)
    sin_a = np.sin(-angle_rad)
    rotation_matrix = np.array([[cos_a, -sin_a], 
                                [sin_a,  cos_a]])
    
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
            print(f"Error: Facet {i} has {len(facet)} dots, but Facet 0 has {num_dots_ref}")
            continue
            
        # 2. Compute Timing Shift (X-axis)
        # Vector subtraction: [x0_f1 - x0_f0, x1_f1 - x1_f0, ...]
        x_diffs = facet[:, 0] - ref_facet[:, 0]
        mean_x_shift = np.mean(x_diffs)
        
        # 3. Compute Orthogonal Shift (Y-axis)
        y_diffs = facet[:, 1] - ref_facet[:, 1]
        mean_y_shift = np.mean(y_diffs)
        
        facet_data[i] = {
            "x_shift_px": round(mean_x_shift, 3),
            "y_shift_px": round(mean_y_shift, 3),
            "std_x": round(np.std(x_diffs), 3) # Check if shift is consistent
        }
        
    return facet_data


def verify_calibration(image_paths, master_report, avg_angle_deg, rect_dots):
    # 1. Create a Stacked Image (Average of all facets)
    base_img = None
    for path in image_paths:
        img = cv.imread(str(path)).astype(np.float32)
        if base_img is None:
            base_img = img
        else:
            base_img += img
    base_img = (base_img / len(image_paths)).astype(np.uint8)

    # 2. Setup Inverse Rotation (De-rectification)
    angle_rad = np.radians(avg_angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    inv_rot_matrix = np.array([[cos_a, -sin_a], 
                               [sin_a,  cos_a]])

    # 3. Create Visualization
    vis = cv.cvtColor(base_img, cv.COLOR_GRAY2BGR) if len(base_img.shape)==2 else base_img.copy()
    
    # Define colors for facets 0-3 (Cyan, Green, Blue, Yellow to match your image)
    detected_colors = [(255, 255, 0), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
    predicted_color = (0, 0, 255) # Red

    # --- Plot DETECTED dots as small hollow circles ---
    for i in range(4):
        # De-rectify: Rotate back to original image space
        detected_dots_img_space = rect_dots[i] @ inv_rot_matrix.T
        
        for pt in detected_dots_img_space:
            # cv.circle(img, center, radius, color, thickness)
            # A thickness of 1 makes it a thin hollow circle.
            cv.circle(vis, (int(pt[0]), int(pt[1])), 6, detected_colors[i], 1)

    # --- Plot PREDICTED dots as small thin 'x' marks ---
    ref_rect = rect_dots[0]
    for i in range(4):
        # Apply the computed shifts to the reference coordinates
        shifted_rect = ref_rect.copy()
        shifted_rect[:, 0] += master_report[i]['x_shift_px']
        shifted_rect[:, 1] += master_report[i]['y_shift_px']
        
        # De-rectify: Rotate back to original image space
        predicted_dots = shifted_rect @ inv_rot_matrix.T
        
        # Plot predicted dots
        for pt in predicted_dots:
            # markerSize=8, thickness=1 makes a small, thin 'x'
            cv.drawMarker(vis, (int(pt[0]), int(pt[1])), predicted_color, 
                          cv.MARKER_TILTED_CROSS, markerSize=8, thickness=1)
            
    # Show the result
    window_name = "Calibration Verification"
    cv.namedWindow(window_name, cv.WINDOW_NORMAL)
    h, w = vis.shape[:2]
    cv.resizeWindow(window_name, int(w * 0.7), int(h * 0.7))
    cv.imshow(window_name, vis)
    print("Verification image displayed. Press any key in the window to close.")
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.imwrite("calibration_verification.jpg", vis)

# --- Usage in your Test ---

rect_dots = []
spot_stats = []
angles = []

for facet in range(4):
    dots_facet, stat = get_dots(f"images/facet{facet}.jpg", debug=False)
    spot_stats.append(stat)
    rect_dot, angle = rectify_dots(dots_facet)
    angles.append(angle)
    rect_dots.append(rect_dot)

shift_data = calculate_facet_shifts(rect_dots)

# 3. Merge them into a single Master Report
master_report = {}
for facet in range(4):
    # Combine the shift dict and the spot stats dict
    master_report[facet] = {**shift_data[facet], **spot_stats[facet]}

# --- View the Final Results ---
import json
print(json.dumps(master_report, indent=4))

base_path = Path(__file__).parent
img_paths = [base_path / f"images/facet{facet}.jpg" for facet in range(4)]

verify_calibration(img_paths, master_report, np.mean(angles), rect_dots)
