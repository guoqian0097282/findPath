import numpy as np
import cv2

# ================== Camera parameters ==================
IMAGE_WIDTH = 1392
IMAGE_HEIGHT = 976

CENTER_U = 696.0
CENTER_V = 488.0

FOCAL_U = 357.143
FOCAL_V = 357.143

# Distortion parameters [k1, k2, k3, k4]
K1 = 0.127628
K2 = -0.0425526
K3 = 0.00876759
K4 = -0.00137963

# ================== Extrinsic (world -> camera) ==================
# Camera frame definition (NEW):
#   Xc: forward (optical axis)
#   Yc: left
#   Zc: up
#
# Below R_ACT / t_ACT is the converted version from your previous R_old/t_old
# to keep the SAME physical pose but expressed in the NEW camera axes.

R_ACT = np.array([
    [0.71933980, 0.0, -0.69465837],
    [0.0,        1.0,  0.0       ],
    [0.69465837, 0.0,  0.71933980]
], dtype=float)

t_ACT = np.array([0.69465837, 0.0, -0.71933980], dtype=float)


# ================== Geometry functions ==================
def undistort_fisheye_pixel(u: float,
                            v: float,
                            max_iter: int = 10,
                            eps: float = 1e-12) -> np.ndarray:
    """
    Fisheye inverse mapping: pixel (u, v) -> undistorted normalized image coords (x, y).

    Here (x, y) means:
        x = right / forward
        y = down  / forward
    under the NEW camera frame where forward axis is Xc.

    Model (OpenCV fisheye):
        theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)

    Return:
        np.array([x, y])
    """
    x_d = (u - CENTER_U) / FOCAL_U
    y_d = (v - CENTER_V) / FOCAL_V

    r_d = np.hypot(x_d, y_d)
    if r_d < 1e-8:
        return np.array([0.0, 0.0], dtype=float)

    theta_d = r_d
    theta = theta_d

    for _ in range(max_iter):
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta4 * theta4

        f = theta * (1.0 + K1 * theta2 + K2 * theta4 + K3 * theta6 + K4 * theta8) - theta_d
        if abs(f) < eps:
            break

        df = 1.0 + 3.0 * K1 * theta2 + 5.0 * K2 * theta4 + 7.0 * K3 * theta6 + 9.0 * K4 * theta8
        theta -= f / df

    r = np.tan(theta)
    scale = r / r_d

    x = x_d * scale
    y = y_d * scale
    return np.array([x, y], dtype=float)


def pixel_to_world(u: float,
                   v: float,
                   z_world: float,
                   check_front: bool = True) -> np.ndarray:
    """
    Given pixel (u, v) and world plane Z = z_world, compute world point (Xw, Yw, Zw).

    Extrinsic:
        Xc = R_ACT @ Xw + t_ACT    (world -> camera, NEW camera axes)

    Ray construction in NEW camera frame:
        undistort gives x = right/forward, y = down/forward
        right  = -Yc (since +Yc is left)
        down   = -Zc (since +Zc is up)
        forward=  Xc

    So direction in camera frame is proportional to:
        [Xc, Yc, Zc] = [1, -x, -y]
    """
    x, y = undistort_fisheye_pixel(u, v)

    d_cam = np.array([1.0, -x, -y], dtype=float)
    d_cam = d_cam / np.linalg.norm(d_cam)

    R = R_ACT
    t = t_ACT.reshape(3)

    # camera center in world: C_w = -R^T t
    C_w = -R.T @ t

    # direction in world
    d_w = R.T @ d_cam

    dz = d_w[2]
    if abs(dz) < 1e-8:
        raise ValueError(f"Ray is almost parallel to Z={z_world} plane, dz={dz}")

    lam = (z_world - C_w[2]) / dz
    if check_front and lam <= 0.0:
        raise ValueError(f"Intersection is behind the camera (lambda={lam})")

    return C_w + lam * d_w


def world_to_pixel(world_point: np.ndarray):
    """
    Project world point -> fisheye pixel under NEW camera frame.

    NEW camera axes:
        Xc forward, Yc left, Zc up

    Image normalized coords:
        x = right/forward = (-Yc) / Xc
        y = down /forward = (-Zc) / Xc

    Then apply fisheye distortion and K.
    """
    X_w = np.asarray(world_point, dtype=float).reshape(3)

    X_c = R_ACT @ X_w + t_ACT
    Xc, Yc, Zc = X_c

    # forward axis is Xc now
    if Xc <= 0.0:
        return None

    x = (-Yc) / Xc
    y = (-Zc) / Xc

    r = np.hypot(x, y)

    if r < 1e-8:
        x_d = 0.0
        y_d = 0.0
    else:
        theta = np.arctan(r)
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta4 * theta4

        theta_d = theta * (1.0 + K1 * theta2 + K2 * theta4 + K3 * theta6 + K4 * theta8)
        scale = theta_d / r

        x_d = x * scale
        y_d = y * scale

    u = FOCAL_U * x_d + CENTER_U
    v = FOCAL_V * y_d + CENTER_V
    return float(u), float(v)


# ================== Visualization ==================
def draw_point(image: np.ndarray,
               u: float,
               v: float,
               world_point: np.ndarray) -> np.ndarray:
    if image is None:
        raise ValueError("Input image is None.")

    img_out = image.copy()
    u_i = int(round(u))
    v_i = int(round(v))

    h, w = img_out.shape[:2]
    if not (0 <= u_i < w and 0 <= v_i < h):
        print(f"Warning: pixel ({u}, {v}) is outside image bounds ({w}x{h}).")

    cv2.circle(img_out, (u_i, v_i), 6, (0, 0, 255), thickness=-1)

    x_w, y_w, z_w = world_point
    text = f"X={x_w:.3f}, Y={y_w:.3f}, Z={z_w:.3f}"
    org = (u_i + 10, max(0, v_i - 10))

    cv2.putText(img_out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img_out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img_out


def draw_ground_grid(image: np.ndarray,
                     z_plane: float,
                     grid_spacing: float = 2.0,
                     grid_extent: float = 20.0,
                     line_color: tuple = (255, 255, 0),
                     line_thickness: int = 1) -> np.ndarray:
    if image is None:
        raise ValueError("Input image is None.")

    img_out = image.copy()
    h, w = img_out.shape[:2]
    num_samples = 50

    xs = np.arange(-grid_extent, grid_extent + 1e-6, grid_spacing)
    ys = np.linspace(-grid_extent, grid_extent, num_samples)

    for x in xs:
        pts_img = []
        for y in ys:
            world_pt = np.array([x, y, z_plane], dtype=float)
            uv = world_to_pixel(world_pt)
            if uv is None:
                if len(pts_img) >= 2:
                    pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)
                    pts_img = []
                continue

            u, v = uv
            if 0 <= u < w and 0 <= v < h:
                pts_img.append((int(round(u)), int(round(v))))
            else:
                if len(pts_img) >= 2:
                    pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)
                    pts_img = []

        if len(pts_img) >= 2:
            pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)

    ys2 = np.arange(-grid_extent, grid_extent + 1e-6, grid_spacing)
    xs2 = np.linspace(-grid_extent, grid_extent, num_samples)

    for y in ys2:
        pts_img = []
        for x in xs2:
            world_pt = np.array([x, y, z_plane], dtype=float)
            uv = world_to_pixel(world_pt)
            if uv is None:
                if len(pts_img) >= 2:
                    pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)
                    pts_img = []
                continue

            u, v = uv
            if 0 <= u < w and 0 <= v < h:
                pts_img.append((int(round(u)), int(round(v))))
            else:
                if len(pts_img) >= 2:
                    pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)
                    pts_img = []

        if len(pts_img) >= 2:
            pts_np = np.array(pts_img, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(img_out, [pts_np], False, line_color, line_thickness)

    # label x distances on y=0 line (first 3 on +x)
    for i in range(1, 4):
        x = i * grid_spacing
        world_pt = np.array([x, 0.0, z_plane], dtype=float)
        uv = world_to_pixel(world_pt)
        if uv is None:
            continue

        u, v = uv
        if not (0 <= u < w and 0 <= v < h):
            continue

        u_i = int(round(u))
        v_i = int(round(v))

        label = f"{x:.1f}m"
        org = (u_i + 5, v_i - 5)
        cv2.putText(img_out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img_out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return img_out


# ================== Example / test ==================
if __name__ == "__main__":
    IMAGE_PATH = "./test_data/hz_2.jpg"
    OUTPUT_PATH = "./test_data/fisheye_grid.jpg"

    u_test = 800.0
    v_test = 500.0
    z_plane = 0.0

    world_pt = pixel_to_world(u_test, v_test, z_plane, True)

    print(f"pixel = ({u_test}, {v_test}), Z_world = {z_plane}")
    print(f"world point = {world_pt}")

    img = cv2.imread(IMAGE_PATH, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Failed to read image: {IMAGE_PATH}")

    h, w = img.shape[:2]
    if (w, h) != (IMAGE_WIDTH, IMAGE_HEIGHT):
        print(f"Warning: image size ({w}x{h}) does not match calibration ({IMAGE_WIDTH}x{IMAGE_HEIGHT}).")

    img_grid = draw_ground_grid(img, z_plane, 2.0, 20.0)
    img_vis = draw_point(img_grid, u_test, v_test, world_pt)

    cv2.imwrite(OUTPUT_PATH, img_vis)
    print(f"saved visualization to: {OUTPUT_PATH}")
