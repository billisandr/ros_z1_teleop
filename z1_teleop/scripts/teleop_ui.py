#!/usr/bin/env python3
"""
Z1 Hand-Teleop — Streamlit control panel (live rosparam knobs + status).

Browser UI at http://localhost:8501 for tuning the station WITHOUT editing YAML or
restarting nodes. It sets ROS params live; the detector and arm tracker re-read the
"soft" knobs every loop, so sliders take effect immediately.

Run inside the container:  z1_ui     (alias; needs the container started with -p 8501:8501)
Successor to the source repo's workshop_ui.py, retargeted for hand-teleop.

A few knobs need a relaunch to apply (they are set up at node init): control/mode,
MediaPipe model complexities/confidences, flip_horizontal, image_source, the
gripper enable. These are flagged in the UI.
"""
import os
os.environ.setdefault("MPLBACKEND", "Agg")

import yaml
import streamlit as st
import rospy
from std_msgs.msg import Bool, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

YAML_PATH = os.path.expanduser("~/catkin_ws/src/z1_teleop/config/teleop.yaml")

ANGLE_NAMES = [
    "shoulder_azimuth", "shoulder_elevation", "humeral_rotation",
    "elbow_flexion", "wrist_flexion", "wrist_deviation",
]
JOINT_LABELS = [
    "joint1 base yaw", "joint2 shoulder", "joint3 elbow",
    "joint4 wrist roll", "joint5 wrist pitch", "joint6 wrist yaw",
]


@st.cache_resource
def ros_init():
    rospy.init_node("teleop_ui", anonymous=True, disable_signals=True)
    return True


def gp(key, default):
    try:
        return rospy.get_param(key, default)
    except Exception:
        return default


def sp(key, value):
    try:
        rospy.set_param(key, value)
    except Exception:
        pass


def snap(topic, typ, timeout=0.25):
    try:
        return rospy.wait_for_message(topic, typ, timeout=timeout)
    except Exception:
        return None


def load_yaml():
    try:
        with open(YAML_PATH, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


# set_page_config MUST be the first Streamlit command — even invoking a
# @st.cache_resource function (ros_init) counts as a command and trips it.
st.set_page_config(page_title="Z1 Hand-Teleop", layout="wide")
ros_init()
st.title("🤖 Z1 Hand-Teleop — Control Panel")

ros_ok = True
try:
    rospy.get_param("/rosdistro", "")
except Exception:
    ros_ok = False
if not ros_ok:
    st.error("ROS master not reachable. Start the station first (z1_teleop / "
             "start_teleop.sh), then reload this page.")

mode = gp("control/mode", "cartesian")

# --------------------------------------------------------------------------- status
with st.container():
    c1, c2, c3, c4 = st.columns(4)
    act = snap("/hand/tracking_active", Bool)
    active = bool(act.data) if act is not None else None
    c1.metric("mode", mode)
    c2.metric("tracking_active",
              "—" if active is None else ("DRIVE ✅" if active else "FROZEN 🛑"))
    grip = snap("/hand/gripper_cmd", Float64)
    c3.metric("gripper", "—" if grip is None else "%.2f" % grip.data)
    js = snap("/z1_gazebo/joint_states", JointState)
    c4.metric("joint_states", "live" if js is not None else "—")

    if mode == "joint_mirror":
        jt = snap("/hand/joint_targets", Float64MultiArray)
        if jt is not None and len(jt.data) >= 6:
            st.caption("human q[1-6] (deg): " + "  ".join("%+.0f" % v for v in jt.data[:6]))
    else:
        tp = snap("/hand/target_pose", PoseStamped)
        if tp is not None:
            p = tp.pose.position
            st.caption("target (world): X=%.2f  Y=%+.3f  Z=%+.3f" % (p.x, p.y, p.z))
    if js is not None:
        st.caption("Z1 joints (rad): " + "  ".join(
            "%+.2f" % js.position[js.name.index(n)]
            for n in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            if n in js.name))
    st.button("🔄 Refresh status (snapshot)")

st.divider()

tab_mode, tab_cart, tab_jm, tab_gest, tab_io = st.tabs(
    ["Mode", "Cartesian", "Joint-mirror", "Gestures", "Presets / Save"])

# --------------------------------------------------------------------------- mode
with tab_mode:
    st.subheader("Control mode")
    new_mode = st.radio("mode", ["cartesian", "joint_mirror"],
                        index=0 if mode == "cartesian" else 1, horizontal=True)
    sp("control/mode", new_mode)
    if new_mode != mode:
        st.warning("Mode change takes effect after a **relaunch** (nodes set up "
                   "perception/subscriptions at startup). The param is saved; "
                   "restart the station to switch.")
    st.caption("cartesian = hand position → IK.  joint_mirror = arm pose → joints "
               "directly (see docs/JOINT_MIRROR.md).")

# --------------------------------------------------------------------------- cartesian
with tab_cart:
    st.subheader("Cartesian hand-mirror")
    a, b = st.columns(2)
    with a:
        sp("arm_tracker/smoothing_alpha",
           st.slider("Calmness (smoothing_alpha)", 0.01, 1.0,
                     float(gp("arm_tracker/smoothing_alpha", 0.10)), 0.01))
        sp("arm_tracker/fixed_x",
           st.slider("Forward reach fixed_x (m)", 0.15, 0.60,
                     float(gp("arm_tracker/fixed_x", 0.25)), 0.01))
        sp("arm_tracker/joint_kp",
           st.slider("Stiffness (joint_kp)", 20.0, 400.0,
                     float(gp("arm_tracker/joint_kp", 150.0)), 5.0))
        sp("arm_tracker/joint_kd",
           st.slider("Damping (joint_kd)", 0.0, 20.0,
                     float(gp("arm_tracker/joint_kd", 3.0)), 0.5))
    with b:
        yr = gp("mapping/y_range", [-0.35, 0.35])
        zr = gp("mapping/z_range", [0.10, 0.75])
        y = st.slider("Image→world Y range (m)", -0.6, 0.6,
                      (float(yr[0]), float(yr[1])), 0.01)
        sp("mapping/y_range", [float(y[0]), float(y[1])])
        z = st.slider("Image→world Z range (m)", 0.0, 1.0,
                      (float(zr[0]), float(zr[1])), 0.01)
        sp("mapping/z_range", [float(z[0]), float(z[1])])
        sp("mapping/deadzone",
           st.slider("Deadzone (still-hand jitter kill)", 0.0, 0.3,
                     float(gp("mapping/deadzone", 0.0)), 0.01))
        sp("mapping/tracked_point",
           st.selectbox("Tracked point", ["wrist", "palm_centroid"],
                        index=["wrist", "palm_centroid"].index(
                            gp("mapping/tracked_point", "wrist"))))
    st.caption("workspace clamp (safety) — applied on top of the ranges above")
    ws = gp("arm_tracker/workspace", {})
    wy = ws.get("y", [-0.40, 0.40])
    wz = ws.get("z", [0.10, 0.75])
    cy, cz = st.columns(2)
    with cy:
        wyv = st.slider("workspace Y (m)", -0.6, 0.6, (float(wy[0]), float(wy[1])), 0.01)
    with cz:
        wzv = st.slider("workspace Z (m)", 0.0, 1.0, (float(wz[0]), float(wz[1])), 0.01)
    ws["y"], ws["z"] = [float(wyv[0]), float(wyv[1])], [float(wzv[0]), float(wzv[1])]
    sp("arm_tracker/workspace", ws)

# --------------------------------------------------------------------------- joint-mirror
with tab_jm:
    st.subheader("Joint-mirror (arm pose → joints)")
    if mode != "joint_mirror":
        st.info("Not active — set Mode to joint_mirror and relaunch. You can still "
                "pre-tune the map below; it is read live once joint_mirror runs.")
    a, b = st.columns(2)
    with a:
        sp("joint_mirror/pose_side",
           st.selectbox("Tracked arm (pose_side)", ["right", "left"],
                        index=0 if gp("joint_mirror/pose_side", "right") == "right" else 1))
    with b:
        sp("joint_mirror/smoothing_alpha",
           st.slider("Calmness (jm smoothing_alpha)", 0.02, 1.0,
                     float(gp("joint_mirror/smoothing_alpha", 0.12)), 0.01))
    st.caption("Per-joint map: each Z1 joint is driven by one human angle. "
               "command = clamp(sign·scale·angle[src] + offset), clamped to joint_limits.")
    hdr = st.columns([2.4, 2.6, 1.0, 1.2, 1.4, 1.0])
    for col, name in zip(hdr, ["Z1 joint", "source angle", "sign", "scale", "offset°", "on"]):
        col.markdown("**%s**" % name)
    for j in range(6):
        m = gp("joint_mirror/map_joint%d" % (j + 1), [j, 1.0, 1.0, 0.0, j < 3])
        cols = st.columns([2.4, 2.6, 1.0, 1.2, 1.4, 1.0])
        cols[0].markdown(JOINT_LABELS[j])
        src = cols[1].selectbox("src%d" % j, ANGLE_NAMES, index=int(m[0]),
                                label_visibility="collapsed")
        sign = cols[2].selectbox("sg%d" % j, [1.0, -1.0],
                                 index=0 if float(m[1]) >= 0 else 1,
                                 label_visibility="collapsed")
        scale = cols[3].number_input("sc%d" % j, 0.1, 3.0, float(m[2]), 0.1,
                                     label_visibility="collapsed")
        offset = cols[4].number_input("of%d" % j, -180.0, 180.0, float(m[3]), 5.0,
                                      label_visibility="collapsed")
        enabled = cols[5].checkbox("en%d" % j, value=bool(m[4]),
                                   label_visibility="collapsed")
        sp("joint_mirror/map_joint%d" % (j + 1),
           [ANGLE_NAMES.index(src), float(sign), float(scale), float(offset), bool(enabled)])

# --------------------------------------------------------------------------- gestures
with tab_gest:
    st.subheader("Operator conventions")
    a, b = st.columns(2)
    with a:
        sp("gesture/hand",
           st.selectbox("Handedness", ["right", "left", "either"],
                        index=["right", "left", "either"].index(gp("gesture/hand", "right"))))
        sp("gesture/clutch",
           st.selectbox("Clutch", ["palm_fist", "always_on", "pinch_hold"],
                        index=["palm_fist", "always_on", "pinch_hold"].index(
                            gp("gesture/clutch", "palm_fist"))))
        sp("gesture/gripper",
           st.selectbox("Gripper", ["pinch", "none"],
                        index=0 if gp("gesture/gripper", "pinch") == "pinch" else 1))
    with b:
        sp("gesture/open_fingers",
           st.slider("Open-palm threshold (fingers)", 2, 5,
                     int(gp("gesture/open_fingers", 4))))
        sp("gesture/fist_fingers",
           st.slider("Fist threshold (fingers)", 0, 3,
                     int(gp("gesture/fist_fingers", 1))))
        sp("gesture/hysteresis_frames",
           st.slider("Clutch hysteresis (frames)", 1, 10,
                     int(gp("gesture/hysteresis_frames", 3))))
        sp("gesture/lost_frames",
           st.slider("Freeze after lost (frames)", 2, 30,
                     int(gp("gesture/lost_frames", 10))))
    pc, po = st.columns(2)
    with pc:
        sp("gesture/pinch_closed_dist",
           st.slider("Pinch = closed (dist)", 0.01, 0.10,
                     float(gp("gesture/pinch_closed_dist", 0.03)), 0.005))
    with po:
        sp("gesture/pinch_open_dist",
           st.slider("Pinch = open (dist)", 0.10, 0.30,
                     float(gp("gesture/pinch_open_dist", 0.18)), 0.005))

# --------------------------------------------------------------------------- presets / save
with tab_io:
    st.subheader("Presets")
    p1, p2, p3 = st.columns(3)
    if p1.button("🐢 Smooth / calm", use_container_width=True):
        sp("arm_tracker/smoothing_alpha", 0.05)
        sp("joint_mirror/smoothing_alpha", 0.08)
        st.success("Applied smooth preset")
    if p2.button("⚡ Responsive", use_container_width=True):
        sp("arm_tracker/smoothing_alpha", 0.25)
        sp("joint_mirror/smoothing_alpha", 0.25)
        st.success("Applied responsive preset")
    if p3.button("↩️ Reset to YAML defaults", use_container_width=True):
        cfg = load_yaml()
        for top in ("control", "hand", "mapping", "gesture", "smoothing",
                    "joint_mirror", "arm_tracker"):
            if top in cfg:
                sp(top, cfg[top])
        st.success("Reloaded defaults from teleop.yaml")

    st.divider()
    st.subheader("Save current values → teleop.yaml")
    st.caption("Writes the live params back into %s (so a relaunch keeps them)." % YAML_PATH)
    if st.button("💾 Save to YAML"):
        cfg = load_yaml()
        for top in ("control", "mapping", "gesture", "smoothing",
                    "joint_mirror", "arm_tracker"):
            cur = gp(top, None)
            if isinstance(cur, dict):
                cfg.setdefault(top, {})
                cfg[top].update(cur)
        try:
            with open(YAML_PATH, "w") as f:
                yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
            st.success("Saved to %s" % YAML_PATH)
        except Exception as e:
            st.error("Save failed: %s" % e)

st.caption("Sliders set ROS params live; the nodes re-read the soft knobs each loop. "
           "Mode / MediaPipe model / camera-source changes need a relaunch.")
