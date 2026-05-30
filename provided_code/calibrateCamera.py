import cv2
import numpy as np
import argparse
import os

# ─────────────────────────────────────────────
# BOARD CONFIGURATION (unchanged - uses your existing printed GridBoard)
# ─────────────────────────────────────────────
BOARD_COLS = 4          # number of marker columns
BOARD_ROWS = 4          # number of marker rows
MARKER_LENGTH = 0.0706  # ArUco marker side in metres
MARKER_SEP = 0.0072     # separation between markers in metres
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_250
CAMERA_INDEX = 1        # webcam index (0 = default)
MIN_VALID_FRAMES = 40   # increased from 30 - more frames = better calibration
OUTPUT_FILE = "camera_params.npz"
MIN_MARKERS = 5         # slightly stricter - reduces noisy frames early
MIN_POINTS_PER_FRAME = 20  # new: require decent number of points per capture

# ─────────────────────────────────────────────
def build_board():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    board = cv2.aruco.GridBoard(
        (BOARD_COLS, BOARD_ROWS), MARKER_LENGTH, MARKER_SEP, aruco_dict
    )
    return board, aruco_dict

def calibrate(camera_index: int = CAMERA_INDEX):
    board, aruco_dict = build_board()

    # ── IMPROVED DETECTOR: sub-pixel refinement (big accuracy boost) ──
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector_params.cornerRefinementWinSize = 5
    detector_params.cornerRefinementMaxIterations = 30
    detector_params.cornerRefinementMinAccuracy = 0.001
    # extra robustness for varying lighting/angles
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 23
    detector_params.adaptiveThreshWinSizeStep = 10
    detector_params.minMarkerPerimeterRate = 0.03

    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    all_obj_points = []
    all_img_points = []
    image_size = None
    captured = 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_index}")

    print("\n──────────────────────────────────────────────")
    print(" CALIBRATION MODE (ArUco GridBoard 4×4) - IMPROVED")
    print(" • Hold board steady when pressing SPACE (no blur!)")
    print(" • Capture 50+ frames from many angles/distances/tilts")
    print(" • Press [SPACE] to capture (green = good)")
    print(" • Press [C] when ready (auto outlier removal runs)")
    print(" • Press [Q] to quit")
    print("──────────────────────────────────────────────\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        # Detect markers
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
        display = frame.copy()

        detected = marker_ids is not None and len(marker_ids) >= MIN_MARKERS
        if detected:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
            n = len(marker_ids)
            status_color = (0, 200, 0)
            status_text = f"Detected {n} markers | Captured: {captured} | SPACE=capture C=calibrate Q=quit"
        else:
            n = 0 if marker_ids is None else len(marker_ids)
            status_color = (0, 0, 220)
            status_text = f"Need {MIN_MARKERS}+ markers (saw {n}) | Captured: {captured} | Q=quit"

        cv2.putText(display, status_text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quit — no calibration saved.")
            break

        elif key == ord(' ') and detected:
            obj_pts, img_pts = board.matchImagePoints(marker_corners, marker_ids)

            if (obj_pts is not None and
                len(obj_pts) >= MIN_POINTS_PER_FRAME and
                len(img_pts) == len(obj_pts)):

                all_obj_points.append(obj_pts)
                all_img_points.append(img_pts)
                captured += 1
                print(f" Frame {captured:3d} captured ({len(marker_ids)} markers, {len(obj_pts)} points)")
            else:
                print(" ⚠ Not enough valid points — try again (hold steadier).")

        elif key == ord('c'):
            if captured < MIN_VALID_FRAMES:
                print(f" ⚠ Need at least {MIN_VALID_FRAMES} frames (have {captured}). Keep capturing.")
            else:
                print(f"\nRunning calibration + outlier removal on {captured} frames …")
                cap.release()
                cv2.destroyAllWindows()
                _run_calibration(all_obj_points, all_img_points, image_size)
                return

    cap.release()
    cv2.destroyAllWindows()

def _run_calibration(all_obj_points, all_img_points, image_size):
    flags = 0

    # Initial calibration
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_points, all_img_points, image_size, None, None, flags=flags
    )
    print(f"Initial RMS: {rms:.4f}")

    # ── ITERATIVE OUTLIER REMOVAL (this is the main improvement) ──
    MAX_ITERS = 5
    for iteration in range(MAX_ITERS):
        # Compute per-frame mean reprojection error
        reproj_errors = []
        for i in range(len(all_obj_points)):
            obj_pts = all_obj_points[i]
            img_pts = all_img_points[i]
            rvec = rvecs[i]
            tvec = tvecs[i]

            proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
            proj_pts = proj_pts.reshape(-1, 2)
            err = np.linalg.norm(img_pts - proj_pts, axis=1)
            mean_err = np.mean(err) if len(err) > 0 else 999
            reproj_errors.append(mean_err)

        mean_err = np.mean(reproj_errors)
        std_err = np.std(reproj_errors)
        median_err = np.median(reproj_errors)

        # Adaptive threshold: removes only extreme outliers
        threshold = max(1.0, mean_err + 2.0 * std_err)

        keep = [i for i, e in enumerate(reproj_errors) if e <= threshold]
        num_removed = len(all_obj_points) - len(keep)

        if num_removed == 0:
            print("No more outliers detected.")
            break

        print(f"Iteration {iteration+1}: Removed {num_removed} outlier frames "
              f"(threshold={threshold:.3f}px, median={median_err:.3f})")

        all_obj_points = [all_obj_points[i] for i in keep]
        all_img_points = [all_img_points[i] for i in keep]

        if len(all_obj_points) < MIN_VALID_FRAMES // 2:
            print("⚠ Too many frames removed - stopping.")
            break

        # Re-calibrate with cleaned data
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            all_obj_points, all_img_points, image_size, None, None, flags=flags
        )
        print(f"   → New RMS: {rms:.4f} ({len(all_obj_points)} frames kept)")

    print(f"\nFINAL Reprojection error (RMS): {rms:.4f}  ← target < 0.5")
    print(f" Camera matrix:\n{camera_matrix}")
    print(f" Distortion coefficients:\n{dist_coeffs.ravel()}")

    np.savez(
        OUTPUT_FILE,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=np.array(image_size),
        rms_error=np.array([rms])
    )
    print(f"\n Saved to -> {OUTPUT_FILE}\n")

# preview_undistort is unchanged (still works with the new .npz)
def preview_undistort(camera_index: int = CAMERA_INDEX):
    if not os.path.exists(OUTPUT_FILE):
        print(f"No calibration file found: {OUTPUT_FILE}\nRun with --calibrate first.")
        return

    data = np.load(OUTPUT_FILE)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    rms = float(data["rms_error"])

    print(f"\n Loaded calibration | FINAL RMS error: {rms:.4f}")

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_index}")

    print("\n UNDISTORT PREVIEW | Press [Q] to quit\n")

    ret, frame = cap.read()
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=1)
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        x, y, rw, rh = roi
        undistorted_crop = undistorted[y:y+rh, x:x+rw]
        undistorted_crop = cv2.resize(undistorted_crop, (w, h))

        combined = np.hstack([frame, undistorted_crop])
        cv2.putText(combined, "Original", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 255), 2)
        cv2.putText(combined, f"Undistorted (RMS={rms:.3f})", (w + 10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
        cv2.imshow("Original | Undistorted", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ──────────────────────────────────────────────────────────────────────────────
# HOW TO USE
# python camera_calibration.py --calibrate
# python camera_calibration.py --preview
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Improved Camera Intrinsic Calibration - ArUco GridBoard")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--calibrate", action="store_true", help="Run calibration + auto outlier removal")
    group.add_argument("--preview", action="store_true", help="Live undistort preview")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="Webcam index")
    args = parser.parse_args()

    if args.calibrate:
        calibrate(args.camera)
    elif args.preview:
        preview_undistort(args.camera)