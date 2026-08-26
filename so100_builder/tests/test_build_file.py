"""io/build_file.py -- BRIDGE_PROTOCOL.md Part A, Phase D.

This file is the entire integration surface with ROS2, so its contract is
tested harder than anything else in the addon: key order, density of
``order``, the ``||tip - base|| == length_m`` invariant, and the protocol
document's own worked example reproduced end to end.
"""

import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import order as core_order  # noqa: E402
from so100_builder.core import sticks as core_sticks  # noqa: E402
from so100_builder.core import validate as core_validate  # noqa: E402
from so100_builder.io import build_file as BF  # noqa: E402

Y = -0.370
L = 0.110
GAP = 0.0065


def stack(count=3, y=Y, length=L):
    """A vertical stack -- the protocol's own worked example shape.

    Vertices are spaced by the STICK length, not length+gap: Sec 5.2.1's
    whole point is that the drawn edge length *is* the physical stick, and
    the expansion solve then pushes the vertices apart to make room for the
    glue joints. Spacing them pre-expanded would silently produce 116.5 mm
    sticks and miss the protocol's documented numbers.
    """
    points = [(0.020, y, i * length) for i in range(count + 1)]
    edges = [(i, i + 1) for i in range(count)]
    return core_sticks.extract_sticks(
        points, edges, ground_mode=core_sticks.GROUND_SLIDE)


def exported(design=None, **kwargs):
    design = design if design is not None else stack()
    order_result = core_order.compute_order(design.sticks)
    verdicts = core_validate.validate_sticks(design.sticks)
    kwargs.setdefault("robot", "so_arm_100")
    kwargs.setdefault("kinematics_version", "1.0.0")
    kwargs.setdefault("source", "test.blend")
    return BF.build_document(order_result.ordered, verdicts, **kwargs)


class TestDocumentShape(unittest.TestCase):
    def test_top_level_keys_match_the_protocol_in_order(self):
        # A.1 rule 4: "pretty-printed, stable key order". Emitted in the
        # protocol's documented order, NOT sorted -- sort_keys would put
        # `base` before `id` and make the file read nothing like its spec.
        self.assertEqual(
            list(exported().keys()),
            ["format", "version", "generated", "source", "frame", "units",
             "robot", "kinematics_version", "stock", "build_volume", "sticks"],
        )

    def test_robot_id_is_written(self):
        # A.1.1 (added 2026-08-21): the executor refuses to run a file for
        # the wrong robot, before even checking kinematics_version.
        self.assertEqual(exported(robot="kr10_r900_2")["robot"], "kr10_r900_2")

    def test_frame_is_the_target_robots_own_root_link(self):
        # Regression for a real bug found 2026-08-23: a kr10_r900_2 export
        # carried the flat FRAME constant's value ("base_link", so_arm_100's
        # own root link) instead of kr10_r900_2's real root link ("base") --
        # the executor correctly refused the file. frame must now come from
        # core_robots.get_robot(robot).frame, per-robot, not a single
        # constant -- so_arm_100 and kr10_r900_2 must disagree here.
        self.assertEqual(exported(robot="so_arm_100")["frame"], "base_link")
        self.assertEqual(exported(robot="kr10_r900_2")["frame"], "base")

    def test_stick_keys_match_the_protocol_in_order(self):
        self.assertEqual(
            list(exported()["sticks"][0].keys()),
            ["id", "order", "base", "tip", "roll_deg", "length_m",
             "shared_ends", "supports", "validation", "warnings"],
        )

    def test_format_and_version_identify_the_file(self):
        document = exported()
        self.assertEqual(document["format"], "so100_build")
        self.assertEqual(document["version"], 1)

    def test_frame_and_units_are_fixed(self):
        # A.1 rule 2: already in robot coordinates; ROS2 does no conversion.
        document = exported()
        self.assertEqual(document["frame"], "base_link")
        self.assertEqual(document["units"], "meters")

    def test_generated_is_utc_iso8601(self):
        generated = exported()["generated"]
        self.assertTrue(generated.endswith("Z"), generated)
        self.assertEqual(len(generated), 20, generated)

    def test_kinematics_version_is_written(self):
        # A.2: ROS2 refuses to execute on mismatch, because the two sides
        # sharing a kinematics module is the whole basis for trusting
        # Blender's validation.
        self.assertEqual(exported(kinematics_version="9.9.9")["kinematics_version"],
                         "9.9.9")

    def test_it_is_valid_json_and_newline_terminated(self):
        text = BF.dumps(exported())
        self.assertTrue(text.endswith("\n"))
        json.loads(text)


class TestOrderDensity(unittest.TestCase):
    """A.2: "`order` must be dense and match array position"."""

    def test_order_matches_array_position(self):
        sticks = exported()["sticks"]
        self.assertEqual([s["order"] for s in sticks], list(range(len(sticks))))

    def test_the_array_is_in_build_order(self):
        design = stack()
        order_result = core_order.compute_order(design.sticks)
        document = BF.build_document(order_result.ordered, {})
        self.assertEqual([s["id"] for s in document["sticks"]],
                         [entry.id for entry in order_result.ordered])

    def test_a_desynchronised_order_is_rewritten_not_trusted(self):
        # A mismatch would silently desynchronise ROS2's progress reporting
        # from the array it iterates, so build_document renumbers rather
        # than trusting its input.
        design = stack()
        order_result = core_order.compute_order(design.sticks)
        order_result.ordered[0].order = 99
        document = BF.build_document(order_result.ordered, {})
        self.assertEqual(document["sticks"][0]["order"], 0)


class TestGeometryInvariants(unittest.TestCase):
    def test_tip_minus_base_equals_length_after_rounding(self):
        # A.2: "||tip - base|| == length_m always", and Sec 6 makes ROS2
        # reject a >1 mm inconsistency with bad_request. Rounding to the
        # micrometre must stay far inside that.
        for entry in exported()["sticks"]:
            span = math.dist(entry["base"], entry["tip"])
            self.assertAlmostEqual(span, entry["length_m"], delta=1e-5,
                                   msg="stick %s" % entry["id"])

    def test_the_invariant_holds_for_tilted_sticks_too(self):
        # Axis-aligned sticks round trivially; a tilted one exercises all
        # three coordinates rounding at once.
        points = [(0.0, Y, 0.0), (0.0, Y, L), (0.045, Y - 0.045, L + 0.08)]
        design = core_sticks.extract_sticks(
            points, [(0, 1), (1, 2)], ground_mode=core_sticks.GROUND_SLIDE)
        for entry in exported(design)["sticks"]:
            span = math.dist(entry["base"], entry["tip"])
            self.assertAlmostEqual(span, entry["length_m"], delta=1e-5)

    def test_coordinates_are_rounded_for_diffability(self):
        for entry in exported()["sticks"]:
            for value in entry["base"] + entry["tip"]:
                self.assertEqual(value, round(value, 6))


class TestProtocolWorkedExample(unittest.TestCase):
    """A.2's own example, reproduced end to end. These are the numbers the
    protocol document was corrected to on 2026-07-28, so this test is what
    keeps the addon and the spec agreeing."""

    def test_a_vertical_stack_reproduces_the_documented_numbers(self):
        sticks = exported(stack(3))["sticks"]
        first, second = sticks[0], sticks[1]

        self.assertAlmostEqual(first["base"][2], 0.0, places=6)
        self.assertAlmostEqual(first["tip"][2], 0.110, places=6)
        self.assertAlmostEqual(second["base"][2], 0.1165, places=6)
        self.assertAlmostEqual(second["tip"][2], 0.2265, places=6)

    def test_the_glue_gap_between_them_is_two_allowances(self):
        sticks = exported(stack(3))["sticks"]
        gap = sticks[1]["base"][2] - sticks[0]["tip"][2]
        self.assertAlmostEqual(gap * 1000.0, 6.5, places=3)

    def test_shared_ends_is_reported(self):
        sticks = exported(stack(3))["sticks"]
        self.assertEqual(sticks[0]["shared_ends"], 1)
        self.assertEqual(sticks[1]["shared_ends"], 2)


class TestSupports(unittest.TestCase):
    """A.2: "ids of already-placed sticks this one's `base` end attaches to.
    Empty => it seats on the base plate"."""

    def test_the_first_stick_seats_on_the_plate(self):
        self.assertEqual(exported(stack(3))["sticks"][0]["supports"], [])

    def test_later_sticks_name_what_they_stand_on(self):
        sticks = exported(stack(3))["sticks"]
        self.assertEqual(sticks[1]["supports"], [sticks[0]["id"]])
        self.assertEqual(sticks[2]["supports"], [sticks[1]["id"]])

    def test_supports_only_ever_reference_earlier_sticks(self):
        # ROS2 uses this to sanity-check the order, so a forward reference
        # would be worse than useless.
        sticks = exported(stack(3))["sticks"]
        seen = set()
        for entry in sticks:
            for support in entry["supports"]:
                self.assertIn(support, seen,
                              "%s references %s which is not placed yet"
                              % (entry["id"], support))
            seen.add(entry["id"])


class TestValidationAndWarnings(unittest.TestCase):
    def test_buildable_sticks_report_no_reason(self):
        for entry in exported()["sticks"]:
            if entry["validation"]["buildable"]:
                self.assertIsNone(entry["validation"]["reason"])

    def test_unbuildable_sticks_are_still_present_and_in_order(self):
        # A.2: they "must still appear in the file, in order, so the operator
        # sees the complete picture and can decide to build the rest".
        design = stack(5)   # tall enough that the top is out of reach
        document = exported(design)
        self.assertEqual(len(document["sticks"]), len(design.sticks))
        unbuildable = [s for s in document["sticks"]
                       if not s["validation"]["buildable"]]
        self.assertTrue(unbuildable, "fixture no longer has an unreachable stick")
        for entry in unbuildable:
            self.assertTrue(entry["validation"]["reason"],
                            "%s is unbuildable but gives no reason" % entry["id"])

    def test_warnings_are_a_list_of_strings(self):
        for entry in exported()["sticks"]:
            self.assertIsInstance(entry["warnings"], list)
            for code in entry["warnings"]:
                self.assertIsInstance(code, str)

    def test_warnings_are_deduplicated(self):
        for entry in exported()["sticks"]:
            self.assertEqual(len(entry["warnings"]), len(set(entry["warnings"])))

    def test_roll_is_always_zero(self):
        # Deliberate: a wireframe edge carries no spin information, and with
        # position fixed this 5-DOF arm has no spare orientation DOF for it
        # anyway. See io/build_file.py's note.
        for entry in exported()["sticks"]:
            self.assertEqual(entry["roll_deg"], 0.0)


class TestLoadBuildFile(unittest.TestCase):
    def test_round_trip(self):
        document = exported()
        reloaded = BF.load_build_file(BF.dumps(document))
        self.assertEqual(reloaded, document)

    def test_rejects_a_foreign_format(self):
        with self.assertRaises(BF.ProtocolError) as caught:
            BF.load_build_file(json.dumps({"format": "something_else"}))
        self.assertIn("not a build file", str(caught.exception))

    def test_rejects_a_future_version(self):
        document = exported()
        document["version"] = 99
        with self.assertRaises(BF.ProtocolError):
            BF.load_build_file(BF.dumps(document))

    def test_rejects_a_non_dense_order(self):
        document = exported()
        document["sticks"][1]["order"] = 7
        with self.assertRaises(BF.ProtocolError) as caught:
            BF.load_build_file(BF.dumps(document))
        self.assertIn("dense", str(caught.exception))

    def test_rejects_malformed_json(self):
        with self.assertRaises(BF.ProtocolError):
            BF.load_build_file("{not json")


# --- A.3 status sidecar -------------------------------------------------------


class TestStatusPath(unittest.TestCase):
    def test_json_extension_is_replaced_not_appended(self):
        self.assertEqual(BF.status_path_for("/tmp/tower_v3.build.json"),
                         "/tmp/tower_v3.build.status.json")

    def test_a_pathless_name_still_works(self):
        self.assertEqual(BF.status_path_for("tower.json"), "tower.status.json")

    def test_a_non_json_path_gets_the_suffix_appended(self):
        self.assertEqual(BF.status_path_for("/tmp/tower"), "/tmp/tower.status.json")


class TestStatusDocument(unittest.TestCase):
    def test_shape_matches_the_protocol(self):
        document = BF.status_document("tower.build.json", {}, current_index=3)
        self.assertEqual(
            list(document.keys()),
            ["format", "version", "build_file", "updated", "current_index", "sticks"])
        self.assertEqual(document["format"], "so100_build_status")
        self.assertEqual(document["current_index"], 3)

    def test_pending_entries_are_omitted(self):
        # A.3: "anything absent is pending" -- writing them would be noise in
        # a file whose whole point is to stay small and diffable.
        document = BF.status_document("b.json", {
            "s_001": {"status": "placed", "at": "2026-07-27T14:31:02Z"},
            "s_002": {"status": "pending"},
        })
        self.assertEqual(list(document["sticks"]), ["s_001"])

    def test_a_reason_is_kept_for_failures(self):
        document = BF.status_document("b.json", {
            "s_008": {"status": "failed", "at": "t", "reason": "grasp failed twice"},
        })
        self.assertEqual(document["sticks"]["s_008"]["reason"], "grasp failed twice")

    def test_round_trip(self):
        original = {
            "s_001": {"status": "placed", "at": "2026-07-27T14:31:02Z", "reason": ""},
            "s_008": {"status": "failed", "at": "2026-07-27T15:03:11Z",
                      "reason": "grasp verification failed twice"},
        }
        document = BF.status_document("b.json", original, current_index=7)
        index, entries = BF.load_status_file(BF.dumps(document))
        self.assertEqual(index, 7)
        self.assertEqual(entries["s_001"]["status"], "placed")
        self.assertEqual(entries["s_008"]["reason"],
                         "grasp verification failed twice")

    def test_an_unknown_status_degrades_to_pending(self):
        # The protocol evolves additively: a newer ROS2 may write a state
        # this addon has not heard of, and refusing to open the file would be
        # a worse failure than showing that stick as not-yet-done.
        text = json.dumps({
            "format": "so100_build_status", "version": 1,
            "sticks": {"s_001": {"status": "teleported"}},
        })
        _index, entries = BF.load_status_file(text)
        self.assertEqual(entries["s_001"]["status"], "pending")

    def test_rejects_a_foreign_format(self):
        with self.assertRaises(BF.ProtocolError):
            BF.load_status_file(json.dumps({"format": "so100_build"}))

    def test_a_missing_sticks_block_is_tolerated(self):
        text = json.dumps({"format": "so100_build_status", "version": 1})
        index, entries = BF.load_status_file(text)
        self.assertEqual((index, entries), (0, {}))


class TestNextStickToLoad(unittest.TestCase):
    """A.5 / Sec 10.3 -- what the Build panel shows the operator."""

    def test_the_first_pending_stick(self):
        order = ["a", "b", "c"]
        statuses = {"a": {"status": "placed"}}
        self.assertEqual(BF.next_stick_to_load(order, statuses), "b")

    def test_an_empty_status_file_starts_at_the_beginning(self):
        self.assertEqual(BF.next_stick_to_load(["a", "b"], {}), "a")

    def test_a_finished_build_returns_none(self):
        statuses = {"a": {"status": "placed"}, "b": {"status": "placed"}}
        self.assertIsNone(BF.next_stick_to_load(["a", "b"], statuses))

    def test_a_failed_stick_is_not_silently_skipped(self):
        # The operator decides what to do about a failure; stepping past it
        # would hide exactly the thing they need to see.
        statuses = {"a": {"status": "failed"}}
        self.assertEqual(BF.next_stick_to_load(["a", "b"], statuses), "a")

    def test_a_skipped_stick_is_also_surfaced(self):
        statuses = {"a": {"status": "skipped"}}
        self.assertEqual(BF.next_stick_to_load(["a", "b"], statuses), "a")


if __name__ == "__main__":
    unittest.main()
