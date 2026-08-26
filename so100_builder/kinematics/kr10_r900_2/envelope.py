"""Reachability queries built on chain.ik() -- mirrors
``so_arm_100_kinematics.envelope``'s role and caveats (fast, conservative
pre-filter; does NOT model self-collision, the mount platform, the table, or
already-placed sticks; MoveIt has the last word regardless).

Unlike SO-100 (1 free orientation DOF), this arm's ``ik()`` needs a full
target orientation and has 4 branches (``elbow_up`` x ``wrist_flip`` -- see
``chain.py``), so both functions below try all 4 before declaring a target
unreachable, and ``target_rot`` must be supplied explicitly -- there is no
single canonical "default orientation" the way SO-100's ``tool_elevation=0``
was.
"""

from .chain import Unreachable, ik

_BRANCHES = ((True, False), (True, True), (False, False), (False, True))


def is_reachable(target_xyz_m, target_rot):
    """Returns (reachable: bool, reason: str or None). Tries all 4
    elbow_up/wrist_flip branches."""
    last_reason = None
    for elbow_up, wrist_flip in _BRANCHES:
        try:
            ik(target_xyz_m, target_rot, elbow_up=elbow_up, wrist_flip=wrist_flip)
            return True, None
        except Unreachable as exc:
            last_reason = str(exc)
    return False, last_reason


def sweep_envelope(z_values_m, target_rot, radius_step_m=0.005, max_radius_m=0.9):
    """For each height in z_values_m, find the min/max horizontal radius (in
    ``base``, from ``joint1``'s own vertical axis) reachable with the given
    fixed target orientation. Mirrors
    ``so_arm_100_kinematics.envelope.sweep_envelope``'s role -- use this
    instead of re-deriving a build-volume sweep by hand.

    Returns {z_m: (min_radius_m, max_radius_m) or None if unreachable at
    every radius}.
    """
    result = {}
    for z in z_values_m:
        reachable_radii = []
        r = 0.0
        while r <= max_radius_m:
            ok, _reason = is_reachable((r, 0.0, z), target_rot)
            if ok:
                reachable_radii.append(r)
            r += radius_step_m
        if reachable_radii:
            result[z] = (min(reachable_radii), max(reachable_radii))
        else:
            result[z] = None
    return result
