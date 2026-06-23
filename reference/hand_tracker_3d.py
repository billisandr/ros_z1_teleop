#!/usr/bin/env python3
"""3D hand, finger, and full-arm tracking across video frames.

Pipeline per frame:
    1. (optional) An open-vocabulary localiser (Grounding DINO or
       LocateAnything-3B) finds a "hand" region and crops to it.
    2. MediaPipe Hands returns 21 landmarks per hand with stable indexing.
    3. (optional) MediaPipe Pose tracks the arm (shoulder, elbow, wrist) and the
       chain is decomposed into 6 joint angles, like a 6-DOF robotic arm.
    4. (optional) Depth Anything V2 supplies per-pixel depth at each landmark.
    5. Landmarks are lifted to 3D and smoothed across frames.
    6. The hand and arm are drawn as a sticks-and-beads skeleton moving in 3D,
       either live during processing or as an animation afterwards.

Everything is driven by config.yaml. Run with:
    python hand_tracker_3d.py --config config.yaml
    python hand_tracker_3d.py --config config.yaml --source clip.mp4

Core dependencies:
    pip install mediapipe opencv-python numpy matplotlib pyyaml
Optional (only if you enable a toggle):
    pip install torch transformers pillow accelerate
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings

import numpy as np
import yaml

try:
    import cv2
except Exception as exc:  # pragma: no cover
    sys.exit("OpenCV is required. Install it with: pip install opencv-python\n%s" % exc)

try:
    import mediapipe as mp
except Exception as exc:  # pragma: no cover
    sys.exit("MediaPipe is required. Install it with: pip install mediapipe\n%s" % exc)

import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)


# ---------------------------------------------------------------------------
# Landmark topology
# ---------------------------------------------------------------------------
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles
HAND_CONNECTIONS = list(mp_hands.HAND_CONNECTIONS)

FINGERS = {
    "thumb": [1, 2, 3, 4],
    "index": [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring": [13, 14, 15, 16],
    "pinky": [17, 18, 19, 20],
}
TIPS = [4, 8, 12, 16, 20]
WRIST = 0

FINGER_COLOR = {
    "thumb": "#e6194B",
    "index": "#3cb44b",
    "middle": "#4363d8",
    "ring": "#f58231",
    "pinky": "#911eb4",
    "palm": "#9aa0a6",
}
LM_FINGER = {}
for _name, _ids in FINGERS.items():
    for _i in _ids:
        LM_FINGER[_i] = _name
LM_FINGER[WRIST] = "palm"

ARM_BONE_COLOR = "#444444"
ARM_JOINT_COLOR = "#ff7f0e"
ARM_LINK_COLOR = "#1f77b4"  # connector from arm wrist to hand wrist

# Pose landmark indices per body side
POSE_IDX = {
    "left": {"shoulder": 11, "elbow": 13, "wrist": 15, "index": 19, "pinky": 17, "hip": 23},
    "right": {"shoulder": 12, "elbow": 14, "wrist": 16, "index": 20, "pinky": 18, "hip": 24},
}


# ---------------------------------------------------------------------------
# Config handling: deep-merge user yaml over defaults, expose as attributes
# ---------------------------------------------------------------------------
DEFAULTS = {
    "input": {"source": 0, "max_frames": 0, "frame_stride": 1, "flip_horizontal": True},
    "camera": {"fx": None, "fy": None, "cx": None, "cy": None},
    "hands": {
        "max_num_hands": 2,
        "model_complexity": 1,
        "min_detection_confidence": 0.6,
        "min_tracking_confidence": 0.6,
    },
    "arm": {
        "enabled": False,
        "side": "right",          # left | right | both
        "model_complexity": 1,
        "min_detection_confidence": 0.5,
        "min_tracking_confidence": 0.5,
        "min_visibility": 0.5,
        "compute_joint_angles": True,
        "draw_arm": True,
        "stitch_to_hand": True,
        "show_angles": True,
    },
    "depth_anything": {
        "enabled": False,
        "model": "depth-anything/Depth-Anything-V2-Small-hf",
        "metric": False,
        "device": "cuda",
        "infer_every": 1,
        "invert": True,
        "scale": 0.5,
        "shift": 0.2,
        "sample_window": 2,
    },
    "localize": {
        "enabled": False,
        "backend": "grounding_dino",
        "prompt": "hand",
        "device": "cuda",
        "detect_every": 5,
        "pad": 0.25,
        "min_box": 24,
        "grounding_dino": {
            "model": "IDEA-Research/grounding-dino-tiny",
            "box_threshold": 0.30,
            "text_threshold": 0.25,
        },
        "locateanything": {"model": "nvidia/LocateAnything-3B"},
    },
    "smoothing": {
        "method": "oneeuro",
        "ema_alpha": 0.5,
        "oneeuro_min_cutoff": 1.2,
        "oneeuro_beta": 0.03,
    },
    "fallback": {"z_nominal": 0.6},
    "view": {
        "live": False,
        "mode": "animate",
        "elev": 18,
        "azim": -70,
        "fps": 20,
        "trail": 12,
        "fixed_limits": None,
        "point_size": 40,
        "line_width": 2.5,
    },
    "output": {"video_path": None, "save_landmarks": None, "overlay_window": True},
}


class NS(dict):
    """Dictionary that also supports attribute access (cfg.view.elev)."""

    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError as exc:
            raise AttributeError(key) from exc


def _deep_merge(base, override):
    out = dict(base)
    for key, val in (override or {}).items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], val)
        else:
            out[key] = val
    return out


def _wrap(obj):
    if isinstance(obj, dict):
        return NS({k: _wrap(v) for k, v in obj.items()})
    return obj


def load_config(path):
    user = {}
    if path and os.path.exists(path):
        with open(path, "r", encoding="utf-8") as handle:
            user = yaml.safe_load(handle) or {}
    return _wrap(_deep_merge(DEFAULTS, user))


# ---------------------------------------------------------------------------
# One Euro filter (per-coordinate, low lag landmark smoothing)
# ---------------------------------------------------------------------------
class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq = float(freq)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = None
        self.dx_prev = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * np.pi * np.asarray(cutoff, dtype=np.float64))
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x):
        x = np.asarray(x, dtype=np.float64)
        if self.x_prev is None:
            self.x_prev = x.copy()
            self.dx_prev = np.zeros_like(x)
            return x.copy()
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = self._alpha(cutoff)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


class EmaFilter:
    def __init__(self, alpha=0.5):
        self.alpha = float(alpha)
        self.prev = None

    def __call__(self, x):
        x = np.asarray(x, dtype=np.float64)
        if self.prev is None:
            self.prev = x.copy()
            return x.copy()
        self.prev = self.alpha * x + (1.0 - self.alpha) * self.prev
        return self.prev.copy()


def make_smoother(cfg, freq):
    method = (cfg.method or "none").lower()
    if method == "oneeuro":
        return OneEuroFilter(freq, cfg.oneeuro_min_cutoff, cfg.oneeuro_beta)
    if method == "ema":
        return EmaFilter(cfg.ema_alpha)
    return None


# ---------------------------------------------------------------------------
# Depth Anything V2 wrapper (lazy import so the base script runs without torch)
# ---------------------------------------------------------------------------
class DepthAnythingEstimator:
    def __init__(self, cfg):
        from transformers import pipeline
        import torch

        self.cfg = cfg
        use_cuda = cfg.device == "cuda" and torch.cuda.is_available()
        device = 0 if use_cuda else -1
        print("[depth-anything] loading %s on %s" % (cfg.model, "cuda" if use_cuda else "cpu"))
        self.pipe = pipeline(task="depth-estimation", model=cfg.model, device=device)
        self._last = None

    def infer(self, frame_bgr):
        from PIL import Image
        import torch

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        out = self.pipe(Image.fromarray(rgb))
        pred = out.get("predicted_depth", None)
        if pred is None:
            depth = np.asarray(out["depth"], dtype=np.float32)
        else:
            if isinstance(pred, torch.Tensor):
                pred = pred.squeeze().detach().cpu().numpy()
            depth = np.asarray(pred, dtype=np.float32)
        h, w = frame_bgr.shape[:2]
        if depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

        if self.cfg.metric:
            depth = depth.astype(np.float32)
        else:
            dmin, dmax = float(depth.min()), float(depth.max())
            if dmax - dmin < 1e-6:
                depth = np.zeros_like(depth)
            else:
                depth = (depth - dmin) / (dmax - dmin)
            if self.cfg.invert:
                depth = 1.0 - depth
            depth = self.cfg.shift + self.cfg.scale * depth
        self._last = depth
        return depth

    def sample(self, depth_map, px, py):
        h, w = depth_map.shape[:2]
        win = int(self.cfg.sample_window)
        x = int(round(px))
        y = int(round(py))
        x0, x1 = max(0, x - win), min(w, x + win + 1)
        y0, y1 = max(0, y - win), min(h, y + win + 1)
        if x1 <= x0 or y1 <= y0:
            return float(self.cfg.shift)
        patch = depth_map[y0:y1, x0:x1]
        return float(np.median(patch))


# ---------------------------------------------------------------------------
# Localiser backends. Each exposes locate(frame_bgr) -> list of [x1,y1,x2,y2].
# Any failure returns an empty list so the pipeline falls back to full frame.
# ---------------------------------------------------------------------------
class GroundingDinoDetector:
    """Turnkey open-vocabulary detector via transformers (recommended)."""

    def __init__(self, cfg):
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

        self.cfg = cfg
        self.torch = torch
        sub = cfg.grounding_dino
        use_cuda = cfg.device == "cuda" and torch.cuda.is_available()
        self.device = "cuda" if use_cuda else "cpu"
        print("[grounding-dino] loading %s on %s" % (sub.model, self.device))
        self.processor = AutoProcessor.from_pretrained(sub.model)
        self.model = (
            AutoModelForZeroShotObjectDetection.from_pretrained(sub.model).to(self.device).eval()
        )
        self.box_threshold = float(sub.box_threshold)
        self.text_threshold = float(sub.text_threshold)
        phrase = cfg.prompt.strip().lower()
        if not phrase.endswith("."):
            phrase = phrase + "."
        self.phrase = phrase
        self._warned = False

    def _post(self, outputs, inputs, size):
        post = self.processor.post_process_grounded_object_detection
        try:
            return post(outputs, inputs.input_ids, threshold=self.box_threshold,
                        text_threshold=self.text_threshold, target_sizes=[size])
        except TypeError:
            return post(outputs, inputs.input_ids, box_threshold=self.box_threshold,
                        text_threshold=self.text_threshold, target_sizes=[size])

    def locate(self, frame_bgr):
        from PIL import Image

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        h, w = frame_bgr.shape[:2]
        try:
            try:
                inputs = self.processor(
                    images=image, text=[[self.cfg.prompt.strip().lower()]], return_tensors="pt"
                ).to(self.device)
            except Exception:
                inputs = self.processor(images=image, text=self.phrase, return_tensors="pt").to(self.device)
            with self.torch.no_grad():
                outputs = self.model(**inputs)
            results = self._post(outputs, inputs, (h, w))
        except Exception as exc:
            if not self._warned:
                warnings.warn("Grounding DINO failed, using full frame. Reason: %s" % exc)
                self._warned = True
            return []
        boxes = []
        for box in results[0].get("boxes", []):
            vals = box.tolist() if hasattr(box, "tolist") else list(box)
            x1, y1, x2, y2 = (float(v) for v in vals)
            if (x2 - x1) >= self.cfg.min_box and (y2 - y1) >= self.cfg.min_box:
                boxes.append([x1, y1, x2, y2])
        return boxes


class LocateAnythingDetector:
    """NVIDIA LocateAnything-3B open-vocabulary grounding VLM.

    LocateAnything-3B decodes boxes with a custom Parallel Box Decoding head,
    so the exact generate and post-processing call lives in the model's remote
    code rather than a stable public API. _run_grounding below is the single
    place to adapt to the current model card. On any failure the detector
    returns no boxes and the pipeline reverts to full-frame MediaPipe.
    """

    def __init__(self, cfg):
        import torch
        from transformers import AutoModel, AutoProcessor

        self.cfg = cfg
        self.torch = torch
        sub = cfg.locateanything
        use_cuda = cfg.device == "cuda" and torch.cuda.is_available()
        self.device = "cuda" if use_cuda else "cpu"
        dtype = torch.float16 if use_cuda else torch.float32
        print("[locate-anything] loading %s on %s" % (sub.model, self.device))
        self.processor = AutoProcessor.from_pretrained(sub.model, trust_remote_code=True)
        self.model = (
            AutoModel.from_pretrained(sub.model, trust_remote_code=True, torch_dtype=dtype)
            .to(self.device)
            .eval()
        )
        self._warned = False

    def _run_grounding(self, frame_rgb, prompt):
        from PIL import Image

        image = Image.fromarray(frame_rgb)
        text = "Locate all the instances that matches the following description: %s." % prompt
        inputs = self.processor(images=image, text=text, return_tensors="pt").to(self.device)
        with self.torch.no_grad():
            outputs = self.model.generate(**inputs)
        post = getattr(self.processor, "post_process_grounding", None)
        if post is None:
            post = getattr(self.processor, "post_process_object_detection", None)
        h, w = frame_rgb.shape[:2]
        results = post(outputs, target_sizes=[(h, w)])
        boxes = []
        for res in results:
            for box in res.get("boxes", []):
                boxes.append([float(v) for v in box])
        return boxes

    def locate(self, frame_bgr):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        try:
            boxes = self._run_grounding(rgb, self.cfg.prompt)
        except Exception as exc:
            if not self._warned:
                warnings.warn(
                    "LocateAnything inference failed, using full frame. "
                    "Adapt _run_grounding to the model card. Reason: %s" % exc
                )
                self._warned = True
            return []
        keep = []
        for x1, y1, x2, y2 in boxes:
            if (x2 - x1) >= self.cfg.min_box and (y2 - y1) >= self.cfg.min_box:
                keep.append([x1, y1, x2, y2])
        return keep


def build_localizer(cfg):
    if not cfg.enabled:
        return None
    backend = (cfg.backend or "grounding_dino").lower()
    if backend in ("grounding_dino", "groundingdino", "gdino"):
        return GroundingDinoDetector(cfg)
    if backend in ("locateanything", "locate_anything", "la"):
        return LocateAnythingDetector(cfg)
    warnings.warn("Unknown localize.backend %r, localisation disabled." % cfg.backend)
    return None


# ---------------------------------------------------------------------------
# Arm tracking (MediaPipe Pose) and 6-DOF joint-angle decomposition
# ---------------------------------------------------------------------------
def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.zeros_like(v)


def _angle(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(_unit(a), _unit(b)), -1.0, 1.0))))


def _signed_angle(a, b, axis):
    a, b, axis = _unit(a), _unit(b), _unit(axis)
    s = float(np.dot(np.cross(a, b), axis))
    c = float(np.dot(a, b))
    return float(np.degrees(np.arctan2(s, c)))


def arm_joint_angles(s, e, w, s_other, hip, hip_other, idx_pt, pky_pt):
    """Approximate 6-DOF decomposition of the shoulder-elbow-wrist-hand chain.

    Returns degrees [q1..q6]:
        q1 shoulder azimuth   (rotation of upper arm about torso vertical)
        q2 shoulder elevation (lift of upper arm above the horizontal)
        q3 humeral rotation   (forearm roll about the upper-arm axis)
        q4 elbow flexion      (0 = straight arm)
        q5 wrist flexion      (forearm-to-hand bend)
        q6 wrist deviation    (hand roll/deviation about the forearm axis)

    This is a pragmatic kinematic mapping for visualisation and relative motion,
    not a clinical biomechanics model.
    """
    # torso frame from shoulders and hips
    shoulder_mid = (s + s_other) / 2.0
    hip_mid = (hip + hip_other) / 2.0
    up = _unit(shoulder_mid - hip_mid)
    lateral = _unit(s - s_other)
    forward = _unit(np.cross(up, lateral))
    lateral = _unit(np.cross(forward, up))  # re-orthogonalise

    u = e - s          # upper arm
    f = w - e          # forearm
    hvec = (idx_pt + pky_pt) / 2.0 - w  # hand direction

    ux, uy, uz = np.dot(u, lateral), np.dot(u, up), np.dot(u, forward)
    q1 = float(np.degrees(np.arctan2(uz, ux)))
    q2 = float(np.degrees(np.arctan2(uy, np.hypot(ux, uz))))

    n = _unit(u)
    ref = up - np.dot(up, n) * n
    fp = f - np.dot(f, n) * n
    q3 = _signed_angle(ref, fp, n)

    q4 = _angle(u, f)

    lat_w = _unit(idx_pt - pky_pt)
    q5 = _signed_angle(f, hvec, lat_w)

    n2 = _unit(f)
    hand_normal = _unit(np.cross(idx_pt - w, pky_pt - w))
    ref2 = up - np.dot(up, n2) * n2
    q6 = _signed_angle(ref2, hand_normal, n2)

    return np.array([q1, q2, q3, q4, q5, q6], dtype=np.float64)


class ArmTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.pose = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=cfg.model_complexity,
            smooth_landmarks=True,
            min_detection_confidence=cfg.min_detection_confidence,
            min_tracking_confidence=cfg.min_tracking_confidence,
        )

    def sides(self):
        s = (self.cfg.side or "right").lower()
        return ["left", "right"] if s == "both" else [s]

    def process(self, full_rgb):
        full_rgb.flags.writeable = False
        return self.pose.process(full_rgb)

    def close(self):
        self.pose.close()

    def arms_3d(self, res, width, height, depth_map, depth_est, intr, z_nominal):
        out = {}
        if res is None or not res.pose_landmarks:
            return out
        lm_img = res.pose_landmarks.landmark
        lm_world = res.pose_world_landmarks.landmark if res.pose_world_landmarks else None
        fx, fy, cx, cy = intr

        def lift(i):
            if depth_map is not None and depth_est is not None:
                px, py = lm_img[i].x * width, lm_img[i].y * height
                d = depth_est.sample(depth_map, px, py)
                return np.array([(px - cx) / fx * d, -((py - cy) / fy * d), d])
            if lm_world is not None:
                lw = lm_world[i]
                return np.array([lw.x, -lw.y, -lw.z])
            px, py = lm_img[i].x * width, lm_img[i].y * height
            d = float(z_nominal)
            return np.array([(px - cx) / fx * d, -((py - cy) / fy * d), d])

        for side in self.sides():
            idx = POSE_IDX[side]
            vis = min(
                lm_img[idx["shoulder"]].visibility,
                lm_img[idx["elbow"]].visibility,
                lm_img[idx["wrist"]].visibility,
            )
            if vis < self.cfg.min_visibility:
                continue
            s = lift(idx["shoulder"])
            e = lift(idx["elbow"])
            w = lift(idx["wrist"])
            entry = {"shoulder": s, "elbow": e, "wrist": w}
            if self.cfg.compute_joint_angles:
                other = "right" if side == "left" else "left"
                entry["angles"] = arm_joint_angles(
                    s, e, w,
                    lift(POSE_IDX[other]["shoulder"]),
                    lift(idx["hip"]),
                    lift(POSE_IDX[other]["hip"]),
                    lift(idx["index"]),
                    lift(idx["pinky"]),
                )
            out[side] = entry
        return out


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def resolve_intrinsics(cfg_cam, width, height):
    fx = cfg_cam.fx if cfg_cam.fx is not None else float(width)
    fy = cfg_cam.fy if cfg_cam.fy is not None else float(width)
    cx = cfg_cam.cx if cfg_cam.cx is not None else width / 2.0
    cy = cfg_cam.cy if cfg_cam.cy is not None else height / 2.0
    return fx, fy, cx, cy


def union_box(boxes, width, height, pad):
    xs1 = min(b[0] for b in boxes)
    ys1 = min(b[1] for b in boxes)
    xs2 = max(b[2] for b in boxes)
    ys2 = max(b[3] for b in boxes)
    bw, bh = xs2 - xs1, ys2 - ys1
    xs1 -= pad * bw
    xs2 += pad * bw
    ys1 -= pad * bh
    ys2 += pad * bh
    return (int(max(0, xs1)), int(max(0, ys1)), int(min(width, xs2)), int(min(height, ys2)))


def hand_world_points(world_lms):
    return np.array([[lm.x, -lm.y, -lm.z] for lm in world_lms.landmark], dtype=np.float64)


def lift_hand(norm_xy, world_lms, depth_map, depth_est, intr, z_nominal):
    """Standalone hand placement when no arm anchor is used."""
    fx, fy, cx, cy = intr
    if depth_map is not None and depth_est is not None:
        h, w = depth_map.shape[:2]
        pts = np.zeros((21, 3), dtype=np.float64)
        for i in range(21):
            px, py = norm_xy[i, 0] * w, norm_xy[i, 1] * h
            d = depth_est.sample(depth_map, px, py)
            pts[i] = [(px - cx) / fx * d, -((py - cy) / fy * d), d]
        return pts
    world = hand_world_points(world_lms)
    d0 = float(z_nominal)
    u0, v0 = norm_xy[WRIST]
    world[:, 0] += (u0 - 0.5) * d0
    world[:, 1] += -(v0 - 0.5) * d0
    world[:, 2] += d0
    return world


# ---------------------------------------------------------------------------
# Visualisation helpers (shared by the live view and the post animation)
# ---------------------------------------------------------------------------
def iter_points(frame):
    for arr in frame["hands"].values():
        yield arr
    for entry in frame["arms"].values():
        yield np.stack([entry["shoulder"], entry["elbow"], entry["wrist"]])


def limits_box(mins, maxs):
    mins = np.asarray(mins, dtype=np.float64)
    maxs = np.asarray(maxs, dtype=np.float64)
    span = float((maxs - mins).max())
    if span < 1e-6:
        span = 1.0
    centre = (maxs + mins) / 2.0
    half = span * 0.6
    return [[float(centre[i] - half), float(centre[i] + half)] for i in range(3)]


def compute_limits(trajectory, fixed):
    if fixed is not None:
        return fixed
    pts = [arr for frame in trajectory for arr in iter_points(frame)]
    if not pts:
        return [[-1, 1], [-1, 1], [0, 2]]
    stacked = np.concatenate(pts, axis=0)
    return limits_box(stacked.min(axis=0), stacked.max(axis=0))


def update_running(running, frame):
    pts = list(iter_points(frame))
    if not pts:
        return running
    stacked = np.concatenate(pts, axis=0)
    fmin, fmax = stacked.min(axis=0), stacked.max(axis=0)
    if running is None:
        return [fmin, fmax]
    return [np.minimum(running[0], fmin), np.maximum(running[1], fmax)]


def connection_color(a, b):
    name = LM_FINGER.get(max(a, b), "palm")
    return FINGER_COLOR.get(name, FINGER_COLOR["palm"])


def _fmt_angles(side, q):
    body = ", ".join("%4.0f" % v if np.isfinite(v) else " nan" for v in q)
    return "%s arm q1..q6 = [%s] deg" % (side[:1].upper(), body)


def draw_skeleton(ax, trajectory, f, cfg, limits, title=None):
    view = cfg.view
    ax.cla()
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_zlim(limits[2])
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z (depth)")
    ax.view_init(elev=view.elev, azim=view.azim)
    if title:
        ax.set_title(title)

    frame = trajectory[f]

    # hands
    for label, pts in frame["hands"].items():
        for a, b in HAND_CONNECTIONS:
            ax.plot(
                [pts[a, 0], pts[b, 0]], [pts[a, 1], pts[b, 1]], [pts[a, 2], pts[b, 2]],
                color=connection_color(a, b), linewidth=view.line_width,
            )
        colors = [FINGER_COLOR[LM_FINGER.get(i, "palm")] for i in range(21)]
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors, s=view.point_size,
                   depthshade=True, edgecolors="k", linewidths=0.4)
        if view.trail and f > 0:
            start = max(0, f - view.trail)
            for tip in TIPS:
                xs, ys, zs = [], [], []
                for g in range(start, f + 1):
                    if label in trajectory[g]["hands"]:
                        p = trajectory[g]["hands"][label][tip]
                        xs.append(p[0])
                        ys.append(p[1])
                        zs.append(p[2])
                if len(xs) > 1:
                    ax.plot(xs, ys, zs, color=FINGER_COLOR[LM_FINGER[tip]], alpha=0.35, linewidth=1.2)

    # arms
    for side, entry in frame["arms"].items():
        s, e, w = entry["shoulder"], entry["elbow"], entry["wrist"]
        for p, q in ((s, e), (e, w)):
            ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]],
                    color=ARM_BONE_COLOR, linewidth=view.line_width + 1.0)
        joints = np.stack([s, e, w])
        ax.scatter(joints[:, 0], joints[:, 1], joints[:, 2], c=ARM_JOINT_COLOR,
                   s=view.point_size + 30, depthshade=True, edgecolors="k", linewidths=0.6)
        if cfg.arm.stitch_to_hand:
            hand = frame["hands"].get(side.capitalize())
            if hand is not None:
                hw = hand[WRIST]
                ax.plot([w[0], hw[0]], [w[1], hw[1]], [w[2], hw[2]],
                        color=ARM_LINK_COLOR, linewidth=view.line_width, linestyle="--")

    # joint-angle readout
    if cfg.arm.show_angles:
        lines = [_fmt_angles(side, entry["angles"])
                 for side, entry in frame["arms"].items() if "angles" in entry]
        if lines:
            ax.text2D(0.02, 0.98, "\n".join(lines), transform=ax.transAxes,
                      va="top", ha="left", fontsize=8, family="monospace",
                      bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.8))


# ---------------------------------------------------------------------------
# Frame processing (with optional live 3D preview)
# ---------------------------------------------------------------------------
def process_video(cfg):
    src = cfg.input.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit("Could not open input source: %r" % src)

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    proc_fps = max(1.0, src_fps / max(1, cfg.input.frame_stride))

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=cfg.hands.max_num_hands,
        model_complexity=cfg.hands.model_complexity,
        min_detection_confidence=cfg.hands.min_detection_confidence,
        min_tracking_confidence=cfg.hands.min_tracking_confidence,
    )

    depth_est = DepthAnythingEstimator(cfg.depth_anything) if cfg.depth_anything.enabled else None
    locator = build_localizer(cfg.localize)
    arm_tracker = ArmTracker(cfg.arm) if cfg.arm.enabled else None

    smoothers = {}
    trajectory = []
    intr = None

    live_ax = None
    running_mm = None
    if cfg.view.live:
        plt.ion()
        live_fig = plt.figure(figsize=(8, 7))
        live_ax = live_fig.add_subplot(111, projection="3d")
        try:
            live_fig.canvas.manager.set_window_title("live 3D hand and arm")
        except Exception:
            pass
        live_fig.show()

    raw_idx = -1
    used = 0
    depth_map = None
    roi = None

    print("[run] reading frames. press q in the overlay window to stop early.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        raw_idx += 1
        if raw_idx % cfg.input.frame_stride != 0:
            continue
        if cfg.input.max_frames and used >= cfg.input.max_frames:
            break

        if cfg.input.flip_horizontal:
            frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]
        if intr is None:
            intr = resolve_intrinsics(cfg.camera, width, height)

        full_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Stage 1: optional open-vocabulary localisation -> crop ROI
        if locator is not None and used % cfg.localize.detect_every == 0:
            boxes = locator.locate(frame)
            roi = union_box(boxes, width, height, cfg.localize.pad) if boxes else None

        if roi is not None:
            rx1, ry1, rx2, ry2 = roi
            crop_rgb = cv2.cvtColor(frame[ry1:ry2, rx1:rx2], cv2.COLOR_BGR2RGB)
        else:
            rx1, ry1, rx2, ry2 = 0, 0, width, height
            crop_rgb = full_rgb.copy()

        # Stage 2: MediaPipe hands on the (cropped) frame
        crop_rgb.flags.writeable = False
        hand_res = hands.process(crop_rgb)

        # Stage 3: optional depth map
        if depth_est is not None and used % cfg.depth_anything.infer_every == 0:
            depth_map = depth_est.infer(frame)

        # Stage 4: optional arm (pose) on the full frame
        arms = {}
        pose_res = None
        if arm_tracker is not None:
            pose_res = arm_tracker.process(full_rgb)
            arms = arm_tracker.arms_3d(pose_res, width, height, depth_map, depth_est,
                                       intr, cfg.fallback.z_nominal)

        # Stage 5: assemble hands, stitching to the arm wrist where possible
        frame_hands = {}
        overlay = frame.copy() if cfg.output.overlay_window else None
        depth_on = depth_map is not None and depth_est is not None

        if hand_res.multi_hand_landmarks and hand_res.multi_hand_world_landmarks:
            cw, ch = (rx2 - rx1), (ry2 - ry1)
            zipped = zip(
                hand_res.multi_hand_landmarks,
                hand_res.multi_hand_world_landmarks,
                hand_res.multi_handedness,
            )
            for h_idx, (lms, world_lms, handed) in enumerate(zipped):
                label = handed.classification[0].label
                if label in frame_hands:
                    label = "%s_%d" % (label, h_idx)

                norm_xy = np.zeros((21, 2), dtype=np.float64)
                for i, lm in enumerate(lms.landmark):
                    px = rx1 + lm.x * cw
                    py = ry1 + lm.y * ch
                    norm_xy[i] = [px / width, py / height]

                arm_match = arms.get(label.lower()) if cfg.arm.stitch_to_hand else None
                if arm_match is not None and not depth_on:
                    # anchor the hand wrist onto the arm wrist in world coords
                    world = hand_world_points(world_lms)
                    pts3d = world - world[WRIST] + arm_match["wrist"]
                else:
                    pts3d = lift_hand(norm_xy, world_lms, depth_map, depth_est, intr,
                                      cfg.fallback.z_nominal)

                sm = smoothers.get(label)
                if sm is None:
                    sm = make_smoother(cfg.smoothing, proc_fps)
                    smoothers[label] = sm
                if sm is not None:
                    pts3d = sm(pts3d.reshape(-1)).reshape(21, 3)

                frame_hands[label] = pts3d

                if overlay is not None:
                    mp_draw.draw_landmarks(
                        overlay[ry1:ry2, rx1:rx2], lms, mp_hands.HAND_CONNECTIONS,
                        mp_styles.get_default_hand_landmarks_style(),
                        mp_styles.get_default_hand_connections_style(),
                    )

        trajectory.append({"hands": frame_hands, "arms": arms})
        used += 1

        if overlay is not None and pose_res is not None and pose_res.pose_landmarks:
            mp_draw.draw_landmarks(overlay, pose_res.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        if live_ax is not None:
            running_mm = update_running(running_mm, trajectory[-1])
            if cfg.view.fixed_limits is not None:
                limits = cfg.view.fixed_limits
            elif running_mm is not None:
                limits = limits_box(running_mm[0], running_mm[1])
            else:
                limits = [[-1, 1], [-1, 1], [0, 2]]
            draw_skeleton(live_ax, trajectory, len(trajectory) - 1, cfg, limits,
                          title="live frame %d  hands %d  arms %d" % (used, len(frame_hands), len(arms)))
            plt.pause(0.001)

        if overlay is not None:
            if roi is not None:
                cv2.rectangle(overlay, (rx1, ry1), (rx2, ry2), (0, 200, 255), 2)
            cv2.putText(overlay, "frame %d  hands %d  arms %d" % (used, len(frame_hands), len(arms)),
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 50), 2)
            cv2.imshow("hand tracking (press q to stop)", overlay)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    hands.close()
    if arm_tracker is not None:
        arm_tracker.close()
    if cfg.output.overlay_window:
        cv2.destroyAllWindows()
    if live_ax is not None:
        plt.ioff()
    n_hands = sum(1 for fr in trajectory if fr["hands"])
    n_arms = sum(1 for fr in trajectory if fr["arms"])
    print("[run] processed %d frames. hands in %d, arms in %d." % (used, n_hands, n_arms))
    return trajectory


# ---------------------------------------------------------------------------
# Post-processing animation / static render
# ---------------------------------------------------------------------------
def render(trajectory, cfg):
    view = cfg.view
    n = len(trajectory)
    if n == 0:
        print("[viz] nothing to draw, no frames captured.")
        return
    if (view.mode or "animate").lower() == "none":
        return

    limits = compute_limits(trajectory, view.fixed_limits)
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    def has_content(i):
        return bool(trajectory[i]["hands"] or trajectory[i]["arms"])

    if view.mode == "show_last":
        last = max((i for i in range(n) if has_content(i)), default=n - 1)
        draw_skeleton(ax, trajectory, last, cfg, limits, title="frame %d" % (last + 1))
        plt.tight_layout()
        plt.show()
        return

    def update(f):
        draw_skeleton(ax, trajectory, f, cfg, limits, title="frame %d / %d" % (f + 1, n))
        return []

    anim = animation.FuncAnimation(fig, update, frames=n, interval=1000.0 / max(1, view.fps), blit=False)

    out_path = cfg.output.video_path
    if view.mode == "save" or out_path:
        if not out_path:
            out_path = "hand_3d.mp4"
        print("[viz] saving animation to %s" % out_path)
        if out_path.lower().endswith(".gif"):
            writer = animation.PillowWriter(fps=view.fps)
        else:
            writer = animation.FFMpegWriter(fps=view.fps, bitrate=3000)
        anim.save(out_path, writer=writer)
        print("[viz] saved.")
    else:
        plt.tight_layout()
        plt.show()


# ---------------------------------------------------------------------------
# Persist the raw 3D trajectory if requested
# ---------------------------------------------------------------------------
def save_landmarks(trajectory, path):
    if not path:
        return
    n = len(trajectory)
    data = {}

    hand_labels = sorted({lbl for fr in trajectory for lbl in fr["hands"]})
    for lbl in hand_labels:
        arr = np.full((n, 21, 3), np.nan, dtype=np.float32)
        for f, fr in enumerate(trajectory):
            if lbl in fr["hands"]:
                arr[f] = fr["hands"][lbl]
        data["hand_%s" % lbl] = arr

    arm_sides = sorted({side for fr in trajectory for side in fr["arms"]})
    for side in arm_sides:
        joints = np.full((n, 3, 3), np.nan, dtype=np.float32)
        angles = np.full((n, 6), np.nan, dtype=np.float32)
        for f, fr in enumerate(trajectory):
            if side in fr["arms"]:
                entry = fr["arms"][side]
                joints[f] = np.stack([entry["shoulder"], entry["elbow"], entry["wrist"]])
                if "angles" in entry:
                    angles[f] = entry["angles"]
        data["arm_%s_joints" % side] = joints
        data["arm_%s_angles" % side] = angles

    np.savez_compressed(path, **data)
    print("[out] wrote 3D trajectory to %s" % path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="3D hand, finger, and arm tracking from video.")
    parser.add_argument("--config", default="config.yaml", help="path to the YAML config")
    parser.add_argument("--source", default=None, help="override input source (webcam index or video path)")
    parser.add_argument("--save-video", default=None, help="override output video path")
    parser.add_argument("--live", action="store_true", help="force the live 3D preview on")
    parser.add_argument("--arm", action="store_true", help="force arm tracking on")
    args = parser.parse_args()

    cfg = load_config(args.config)

    if args.source is not None:
        cfg.input["source"] = int(args.source) if args.source.isdigit() else args.source
    if args.save_video is not None:
        cfg.output["video_path"] = args.save_video
        cfg.view["mode"] = "save"
    if args.live:
        cfg.view["live"] = True
    if args.arm:
        cfg.arm["enabled"] = True

    trajectory = process_video(cfg)
    save_landmarks(trajectory, cfg.output.save_landmarks)

    if (cfg.view.mode or "animate").lower() == "none":
        if cfg.view.live:
            plt.show()
    else:
        render(trajectory, cfg)


if __name__ == "__main__":
    main()
