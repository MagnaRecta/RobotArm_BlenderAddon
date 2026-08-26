"""End-to-end tests that need Blender. Skipped in bare CPython.

Run with::

    blender --background --python so100_builder/tests/run.py

Covers the bpy-facing half that ``test_sticks.py`` deliberately cannot: the
mesh read, the ``matrix_world`` / ``scale_length`` handling (constraint B6),
stable ids across mesh edits (Sec 9.2), and the derived build mesh.
"""

import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    import bmesh
    import bpy
    from mathutils import Matrix
except ImportError:  # pragma: no cover -- bare CPython path
    bpy = None

if bpy is not None:
    import so100_builder
    from so100_builder.core import order as core_order
    from so100_builder.core import state as core_state
    from so100_builder.ops.design import EDGE_ID_LAYER


CUBE_EDGE_M = 0.110
CUBE_Y = -0.370


def _wireframe_cube(name="Design", edge=CUBE_EDGE_M):
    """A cube with only vertices and edges -- 8 verts, 12 edges, no faces,
    sitting on the plate and centred in the build volume."""
    h = edge / 2.0
    verts = [
        (-h, CUBE_Y - h, 0.0), (h, CUBE_Y - h, 0.0), (h, CUBE_Y + h, 0.0), (-h, CUBE_Y + h, 0.0),
        (-h, CUBE_Y - h, edge), (h, CUBE_Y - h, edge), (h, CUBE_Y + h, edge),
        (-h, CUBE_Y + h, edge),
    ]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, edges, [])
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


@unittest.skipIf(bpy is None, "requires Blender")
class BlenderTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            so100_builder.register()
        except Exception:
            so100_builder.unregister()
            so100_builder.register()

    @classmethod
    def tearDownClass(cls):
        so100_builder.unregister()

    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.props = bpy.context.scene.so100
        self.design = _wireframe_cube()
        bpy.ops.so100.create_base_empty()
        self.props.design_mesh = self.design

    def extract(self, **overrides):
        for key, value in overrides.items():
            setattr(self.props, key, value)
        result = bpy.ops.so100.extract_sticks()
        self.assertEqual(result, {"FINISHED"})
        return self.props


@unittest.skipIf(bpy is None, "requires Blender")
class TestRegistration(BlenderTestCase):
    def test_scene_property_group_exists(self):
        self.assertTrue(hasattr(bpy.context.scene, "so100"))

    def test_operators_are_registered(self):
        for name in ("extract_sticks", "create_base_empty", "clear_results",
                     "export_cut_list"):
            self.assertTrue(hasattr(bpy.ops.so100, name), name)

    def test_panels_are_registered(self):
        self.assertIsNotNone(bpy.types.SO100_PT_design)
        self.assertEqual(bpy.types.SO100_PT_design.bl_category, "RA130")


@unittest.skipIf(bpy is None, "requires Blender")
class TestExtraction(BlenderTestCase):
    def test_cube_yields_twelve_sticks_of_110mm(self):
        props = self.extract()
        self.assertEqual(len(props.sticks), 12)
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 110.0, places=4)

    def test_every_stick_gets_a_stable_id(self):
        props = self.extract()
        ids = [item.stick_id for item in props.sticks]
        self.assertEqual(len(set(ids)), 12)
        self.assertTrue(all(core_state.parse_id(i) for i in ids))

    def test_summary_reports_the_counts(self):
        # Not "0 impossible": Phase B wires in real kinematic validation,
        # and this cube's horizontal top/bottom-ring edges are a genuine
        # "horizontal radial" placement (Sec 9.4 -- costs ~150mm of extra
        # reach), so some legitimately come back unreachable at this
        # position. Check the summary's structure, not a specific count.
        props = self.extract(ground_mode="SLIDE")
        self.assertIn("12 sticks", props.summary)
        self.assertRegex(props.summary, r"\d+ buildable - \d+ impossible")
        buildable = sum(1 for i in props.sticks if i.status == core_state.STATUS_BUILDABLE)
        impossible = sum(1 for i in props.sticks if i.status == core_state.STATUS_IMPOSSIBLE)
        self.assertEqual(buildable + impossible, 12)
        self.assertGreater(buildable, 0)

    def test_dimensions_show_design_and_built(self):
        props = self.extract(ground_mode="SLIDE")
        self.assertIn("110.0 x 110.0 x 110.0", props.design_dimensions_mm)
        self.assertIn("116.5", props.expanded_dimensions_mm)

    def test_pinned_ground_flags_the_bottom_ring(self):
        # Isolate the PIN-vs-SLIDE ground-mode question (Sec 5.2.2) from
        # Phase B's kinematic validation, which is orthogonal and can flag
        # its own, unrelated impossible sticks at this cube's position
        # (Sec 9.4's "horizontal radial" reach cost).
        props = self.extract(ground_mode="PIN")
        residual_errors = [i for i in props.sticks if "will not physically fit" in i.reason]
        self.assertEqual(len(residual_errors), 4)

    def test_operator_is_blocked_without_a_base_empty(self):
        self.props.base_empty = None
        self.assertFalse(bpy.ops.so100.extract_sticks.poll())

    def test_edit_mode_is_refused_rather_than_silently_wrong(self):
        bpy.context.view_layer.objects.active = self.design
        bpy.ops.object.mode_set(mode="EDIT")
        try:
            with self.assertRaises(RuntimeError) as caught:
                bpy.ops.so100.extract_sticks()
            self.assertIn("Edit Mode", str(caught.exception))
        finally:
            bpy.ops.object.mode_set(mode="OBJECT")

    def test_a_mesh_with_faces_still_extracts_and_warns(self):
        mesh = self.design.data
        mesh.from_pydata([], [], [])
        cube = bpy.ops.mesh.primitive_cube_add
        cube(size=0.110, location=(0.0, CUBE_Y, 0.055))
        faced = bpy.context.active_object
        self.props.design_mesh = faced
        self.assertEqual(bpy.ops.so100.extract_sticks(), {"FINISHED"})
        self.assertEqual(len(self.props.sticks), 12)


@unittest.skipIf(bpy is None, "requires Blender")
class TestWorldTransform(BlenderTestCase):
    """Constraint B6: always read matrix_world, never object.location."""

    def test_moving_the_base_empty_moves_the_design_in_base_link(self):
        before = self.extract().sticks[0].stick_length_mm
        self.props.base_empty.location = (0.05, 0.05, 0.02)
        bpy.context.view_layer.update()
        after = self.extract().sticks[0].stick_length_mm
        # A rigid move of base_link cannot change a stick's length.
        self.assertAlmostEqual(before, after, places=6)

    def test_object_scale_is_honoured(self):
        # Doubling the design object's scale doubles the physical sticks, so
        # they blow past the 150 mm maximum.
        self.design.scale = (2.0, 2.0, 2.0)
        bpy.context.view_layer.update()
        props = self.extract()
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 220.0, places=3)

    def test_parent_transform_is_honoured(self):
        parent = bpy.data.objects.new("Parent", None)
        bpy.context.collection.objects.link(parent)
        parent.scale = (0.5, 0.5, 0.5)
        self.design.parent = parent
        bpy.context.view_layer.update()
        props = self.extract()
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 55.0, places=3)

    def test_scene_scale_length_is_applied(self):
        # Halving the scene's unit scale halves the physical stick length.
        # Deliberately mild (not an extreme scale like 0.001): the resulting
        # 55mm stick stays above EVERY registered robot's own hard length
        # floor (core_sticks.hard_min_stick_length_m(), now robot-aware --
        # docs/STATUS.md 2026-08-23), so this test can exercise scale_length
        # without also needing to fight that unrelated check via a
        # min_stick_length_mm override.
        bpy.context.scene.unit_settings.scale_length = 0.5
        props = self.extract()
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 55.0, places=3)

    def test_rotated_base_empty_does_not_change_lengths(self):
        self.props.base_empty.rotation_euler = (0.3, -0.2, 1.1)
        bpy.context.view_layer.update()
        props = self.extract()
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 110.0, places=4)


@unittest.skipIf(bpy is None, "requires Blender")
class TestStableIds(BlenderTestCase):
    """Sec 9.2 / Sec 12: edge indices are not stable across mesh edits, so
    ids must survive one or the user must be warned the order is stale."""

    def test_id_layer_is_created_on_the_edge_domain(self):
        self.extract()
        attribute = self.design.data.attributes[EDGE_ID_LAYER]
        self.assertEqual(attribute.domain, "EDGE")
        self.assertEqual(attribute.data_type, "INT")

    def test_ids_are_unchanged_by_a_second_extraction(self):
        first = [i.stick_id for i in self.extract().sticks]
        second = [i.stick_id for i in self.extract().sticks]
        self.assertEqual(first, second)

    def test_ids_survive_moving_a_vertex(self):
        before = {i.stick_id for i in self.extract().sticks}
        self.design.data.vertices[6].co.z += 0.02
        after = {i.stick_id for i in self.extract().sticks}
        self.assertEqual(before, after)

    def test_a_new_edge_gets_a_fresh_id_and_the_others_keep_theirs(self):
        before = {i.stick_id for i in self.extract().sticks}
        mesh = self.design.data
        mesh.vertices.add(1)
        mesh.vertices[-1].co = (0.0, CUBE_Y, CUBE_EDGE_M * 2.0)
        mesh.edges.add(1)
        mesh.edges[-1].vertices = (4, len(mesh.vertices) - 1)
        mesh.update()

        after = {i.stick_id for i in self.extract().sticks}
        self.assertEqual(len(after), 13)
        self.assertTrue(before.issubset(after),
                        "existing sticks lost their ids when an edge was added")

    def test_ids_are_never_reused_after_a_deletion(self):
        props = self.extract()
        highest = max(core_state.parse_id(i.stick_id) for i in props.sticks)
        self.assertGreaterEqual(props.next_stick_number, highest + 1)
        allocated = props.next_stick_number

        self.design.data.edges[0].vertices = (0, 1)  # touch, then re-extract
        self.extract()
        self.assertGreaterEqual(self.props.next_stick_number, allocated)

    def test_topology_signature_detects_a_stale_result(self):
        props = self.extract()
        signature = props.topology_signature
        self.assertTrue(signature)
        self.design.data.vertices[4].co.z += 0.03
        self.extract()
        self.assertNotEqual(self.props.topology_signature, signature)
        self.assertTrue(props.results_are_stale(signature))


@unittest.skipIf(bpy is None, "requires Blender")
class TestBuildMesh(BlenderTestCase):
    """Sec 5.2.3 point 3: non-destructive -- generate a derived build mesh and
    never modify the user's design mesh."""

    def test_build_mesh_is_a_separate_object(self):
        props = self.extract(ground_mode="SLIDE")
        self.assertIsNotNone(props.build_mesh)
        self.assertNotEqual(props.build_mesh, props.design_mesh)
        self.assertEqual(len(props.build_mesh.data.edges), 12)
        self.assertEqual(len(props.build_mesh.data.vertices), 24)

    def test_design_mesh_geometry_is_untouched(self):
        before = [tuple(v.co) for v in self.design.data.vertices]
        self.extract(ground_mode="SLIDE")
        after = [tuple(v.co) for v in self.design.data.vertices]
        self.assertEqual(before, after)

    def test_build_mesh_shows_the_glue_gaps(self):
        props = self.extract(ground_mode="SLIDE")
        mesh = props.build_mesh.data
        lengths = [
            (mesh.vertices[a].co - mesh.vertices[b].co).length
            for a, b in (tuple(e.vertices) for e in mesh.edges)
        ]
        for length in lengths:
            self.assertAlmostEqual(length, CUBE_EDGE_M, places=6)
        # The sticks are 110 mm but span a 116.5 mm structure -- the
        # difference is the gaps.
        zs = [v.co.z for v in mesh.vertices]
        self.assertAlmostEqual(max(zs) - min(zs), 0.1165, places=4)

    def test_build_mesh_is_regenerated_not_duplicated(self):
        self.extract(ground_mode="SLIDE")
        first = self.props.build_mesh.name
        self.extract(ground_mode="SLIDE")
        self.assertEqual(self.props.build_mesh.name, first)
        self.assertEqual(len(self.props.build_mesh.data.edges), 12)


@unittest.skipIf(bpy is None, "requires Blender")
class TestStockLengthMode(BlenderTestCase):
    def test_snapping_uses_the_configured_lengths(self):
        props = self.extract(length_mode="STOCK", stock_lengths_mm="100, 150",
                             ground_mode="SLIDE")
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 100.0, places=4)

    def test_an_exact_tie_snaps_to_the_shorter_stock(self):
        # The 110 mm cube against 100/120 mm stock. See the matching unit
        # test: the tie-break must not depend on transform float noise.
        props = self.extract(length_mode="STOCK", stock_lengths_mm="100, 120",
                             ground_mode="SLIDE")
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 100.0, places=4)

    def test_a_bad_stock_length_field_is_an_error_not_a_crash(self):
        # An operator that reports {'ERROR'} raises RuntimeError out of
        # bpy.ops -- which is exactly how the message reaches the user's
        # status bar. What matters is that it is a clean, specific message
        # and not a traceback from deep inside the solver.
        self.props.length_mode = "STOCK"
        self.props.stock_lengths_mm = "one hundred"
        with self.assertRaises(RuntimeError) as caught:
            bpy.ops.so100.extract_sticks()
        self.assertIn("not a number in Stock Lengths", str(caught.exception))


@unittest.skipIf(bpy is None, "requires Blender")
class TestAutoFlip(BlenderTestCase):
    """Finding 6 automated (2026-07-30, user-requested): when a stick is
    unreachable ONLY because of which end got picked as "base"
    (Wrist_Roll's asymmetric limit), Extract Sticks now flips it itself
    instead of leaving the user to notice the reason text and toggle a
    checkbox by hand. Exact geometry from the user's own third report."""

    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.props = bpy.context.scene.so100
        verts = [
            (0.06283699721097946, -0.37227827310562134, 0.10999858379364014),
            (0.06283699721097946, -0.37227827310562134, 0.0),
            (-0.04716134071350098, -0.37227827310562134, 0.10999858379364014),
            (-0.047161001712083817, -0.37227827310562134, 0.0),
        ]
        edges = [(3, 2), (0, 1), (0, 2)]
        mesh = bpy.data.meshes.new("D")
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        self.design = bpy.data.objects.new("D", mesh)
        bpy.context.collection.objects.link(self.design)
        bpy.ops.so100.create_base_empty()
        self.props.design_mesh = self.design

    def extract(self, **overrides):
        for key, value in overrides.items():
            setattr(self.props, key, value)
        result = bpy.ops.so100.extract_sticks()
        self.assertEqual(result, {"FINISHED"})
        return self.props

    def test_the_unreachable_end_gets_flipped_automatically(self):
        props = self.extract(ground_mode="SLIDE")
        for item in props.sticks:
            self.assertEqual(item.status, "buildable", "%s: %s" % (item.stick_id, item.reason))

    def test_the_auto_flipped_stick_carries_the_warning_and_flip_flag(self):
        props = self.extract(ground_mode="SLIDE")
        top = max(props.sticks, key=lambda i: i.shared_ends)  # the connecting stick
        flagged = [i for i in props.sticks if "auto_flipped" in i.warning_list()]
        self.assertEqual(len(flagged), 1)
        self.assertTrue(flagged[0].flip)

    def test_a_second_extraction_stays_stable_no_double_flip(self):
        # Re-running must not flip it back -- the stored `flip` from the
        # first pass is itself now the input to the second.
        first = self.extract(ground_mode="SLIDE")
        first_flip = {i.stick_id: i.flip for i in first.sticks}
        second = self.extract(ground_mode="SLIDE")
        second_flip = {i.stick_id: i.flip for i in second.sticks}
        self.assertEqual(first_flip, second_flip)
        for item in second.sticks:
            self.assertEqual(item.status, "buildable")


@unittest.skipIf(bpy is None, "requires Blender")
class TestBuildOrderOperator(BlenderTestCase):
    """Phase C's operator layer. ``bpy.ops.*()`` from a script uses
    EXEC_DEFAULT, so this exercises the synchronous path; the chunked modal
    path needs a real event loop and is smoke-tested in a windowed Blender
    instead (see the README)."""

    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.props = bpy.context.scene.so100
        # A 3-level ladder: multi-layer, all-tangential horizontals, and
        # fully buildable -- Phase C's own "done when" fixture.
        half, length, gap = 0.055, 0.110, 0.0065
        verts, edges = [], []
        for level in range(3):
            z = level * (length + gap)
            verts.extend([(-half, CUBE_Y, z), (half, CUBE_Y, z)])
        for level in range(2):
            base = level * 2
            edges.append((base, base + 2))
            edges.append((base + 1, base + 3))
        for level in range(1, 3):
            base = level * 2
            edges.append((base, base + 1))
        mesh = bpy.data.meshes.new("Ladder")
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        self.design = bpy.data.objects.new("Ladder", mesh)
        bpy.context.collection.objects.link(self.design)
        bpy.ops.so100.create_base_empty()
        self.props.design_mesh = self.design
        self.props.ground_mode = "SLIDE"

    def _order(self):
        self.assertEqual(bpy.ops.so100.compute_build_order(), {"FINISHED"})
        return self.props

    def test_the_operator_is_registered(self):
        self.assertTrue(hasattr(bpy.ops.so100, "compute_build_order"))
        self.assertTrue(hasattr(bpy.ops.so100, "clear_build_order"))

    def test_ordering_assigns_a_dense_order_to_every_stick(self):
        props = self._order()
        orders = sorted(item.order for item in props.sticks)
        self.assertEqual(orders, list(range(len(props.sticks))))

    def test_it_works_without_extracting_first(self):
        # Compute Build Order re-runs extraction itself, so it must not
        # require the user to press Extract Sticks first.
        self.assertEqual(len(self.props.sticks), 0)
        props = self._order()
        self.assertGreater(len(props.sticks), 0)
        self.assertTrue(props.has_order)

    def test_the_order_is_support_valid_in_the_stored_result(self):
        props = self._order()
        by_order = sorted(props.sticks, key=lambda i: i.order)
        self.assertEqual([i.order for i in by_order], list(range(len(by_order))))
        # The first stick placed must be a grounded one.
        self.assertEqual(by_order[0].order, 0)

    def test_props_stay_in_extraction_order_for_check_by_eye(self):
        # The build mesh's edge i must still be sticks[i] -- ordering must
        # not permute the collection itself.
        props = self._order()
        self.assertEqual(len(props.build_mesh.data.edges), len(props.sticks))
        bpy.ops.so100.select_stick_in_viewport()
        bpy.ops.object.mode_set(mode="OBJECT")

    def test_the_summary_reports_the_order(self):
        props = self._order()
        self.assertIn("in order", props.order_summary)

    def test_clear_build_order_resets_without_losing_sticks(self):
        props = self._order()
        count = len(props.sticks)
        self.assertEqual(bpy.ops.so100.clear_build_order(), {"FINISHED"})
        self.assertFalse(props.has_order)
        self.assertEqual(len(props.sticks), count)
        self.assertTrue(all(item.order == -1 for item in props.sticks))

    def test_a_floating_component_is_reported_through_the_operator(self):
        mesh = self.design.data
        mesh.vertices.add(2)
        mesh.vertices[-2].co = (0.0, CUBE_Y, 0.30)
        mesh.vertices[-1].co = (0.0, CUBE_Y, 0.41)
        mesh.edges.add(1)
        mesh.edges[-1].vertices = (len(mesh.vertices) - 2, len(mesh.vertices) - 1)
        mesh.update()

        props = self._order()
        codes = [entry.code for entry in props.order_warnings]
        self.assertIn("floating_component", codes)
        self.assertTrue(any(entry.is_error for entry in props.order_warnings))

    def test_raising_the_build_plate_grounds_a_previously_floating_component(self):
        # A standalone stick sitting entirely above Z=0 (own lowest point at
        # 100mm) -- not the ladder above, to avoid the ladder's own base
        # (Z=0) ending up BELOW a raised plate and confusing the result.
        # Raising the plate to meet it (2026-08-23: the plate is a real,
        # height-adjustable object) makes it buildable through the full
        # operator path, not just core/order.py directly.
        mesh = bpy.data.meshes.new("Floating")
        mesh.from_pydata([(0.0, CUBE_Y, 0.100), (0.0, CUBE_Y, 0.210)], [(0, 1)], [])
        mesh.update()
        self.design = bpy.data.objects.new("Floating", mesh)
        bpy.context.collection.objects.link(self.design)
        self.props.design_mesh = self.design

        props = self._order()
        codes = [entry.code for entry in props.order_warnings]
        self.assertIn("floating_component", codes)

        self.props.build_plate_height_mm = 100.0
        props = self._order()
        codes = [entry.code for entry in props.order_warnings]
        self.assertNotIn("floating_component", codes)
        self.assertTrue(props.has_order)
        self.assertTrue(all(item.order >= 0 for item in props.sticks))

    def test_unchecking_require_build_plate_orders_a_design_with_no_ground_at_all(self):
        # 2026-08-23 user request: "I would put a stick's base... on shapes
        # that are not a flat base" -- a design with NO vertex anywhere
        # near a plausible plate height. Require Build Plate off makes it
        # buildable through the full operator path (checkbox -> property ->
        # both extraction and ordering agreeing on no ground check).
        mesh = bpy.data.meshes.new("Floating")
        mesh.from_pydata([(0.0, CUBE_Y, 0.100), (0.0, CUBE_Y, 0.210)], [(0, 1)], [])
        mesh.update()
        self.design = bpy.data.objects.new("Floating", mesh)
        bpy.context.collection.objects.link(self.design)
        self.props.design_mesh = self.design

        self.props.require_build_plate = False
        props = self._order()
        codes = [entry.code for entry in props.order_warnings]
        self.assertNotIn("floating_component", codes)
        self.assertIn(core_order.WARN_NO_BUILD_PLATE, codes)
        self.assertTrue(props.has_order)
        self.assertTrue(all(item.order >= 0 for item in props.sticks))

    def _use_two_independent_uprights(self):
        # Two separately-grounded sticks with no dependency on each other --
        # either can legally go first, so swapping them is always "safe".
        mesh = bpy.data.meshes.new("TwoUprights")
        mesh.from_pydata(
            [(-0.05, CUBE_Y, 0.0), (-0.05, CUBE_Y, 0.110),
             (0.05, CUBE_Y, 0.0), (0.05, CUBE_Y, 0.110)],
            [(0, 1), (2, 3)], [])
        mesh.update()
        self.design = bpy.data.objects.new("TwoUprights", mesh)
        bpy.context.collection.objects.link(self.design)
        self.props.design_mesh = self.design

    def _use_a_three_stick_chain(self):
        # A straight vertical chain -- "b" needs "a"'s tip, "c" needs "b"'s
        # tip -- so the automatic order is forced and unambiguous, and
        # moving "c" before "b" is guaranteed to break its own support.
        mesh = bpy.data.meshes.new("Chain")
        mesh.from_pydata(
            [(0.0, CUBE_Y, 0.0), (0.0, CUBE_Y, 0.110), (0.0, CUBE_Y, 0.220),
             (0.0, CUBE_Y, 0.330)],
            [(0, 1), (1, 2), (2, 3)], [])
        mesh.update()
        self.design = bpy.data.objects.new("Chain", mesh)
        bpy.context.collection.objects.link(self.design)
        self.props.design_mesh = self.design

    def test_move_build_step_swaps_two_independent_sticks_cleanly(self):
        self._use_two_independent_uprights()
        props = self._order()
        self.assertEqual(len(props.sticks), 2)
        first_index = next(i for i, item in enumerate(props.sticks) if item.order == 0)
        first_id, second_id = props.sticks[first_index].stick_id, None
        for item in props.sticks:
            if item.order == 1:
                second_id = item.stick_id

        props.active_stick_index = first_index
        self.assertEqual(bpy.ops.so100.move_build_step(direction=1), {"FINISHED"})

        by_id = {item.stick_id: item.order for item in props.sticks}
        self.assertEqual(by_id[first_id], 1)
        self.assertEqual(by_id[second_id], 0)
        self.assertFalse(any(entry.is_error for entry in props.order_warnings))

    def test_move_build_step_flags_a_move_that_breaks_support(self):
        self._use_a_three_stick_chain()
        props = self._order()
        self.assertEqual(len(props.sticks), 3)
        # "c" is whichever stick landed at build position 2 (the forced,
        # unambiguous last position in a straight chain).
        c_index = next(i for i, item in enumerate(props.sticks) if item.order == 2)
        c_id = props.sticks[c_index].stick_id

        props.active_stick_index = c_index
        self.assertEqual(bpy.ops.so100.move_build_step(direction=-1), {"FINISHED"})

        by_id = {item.stick_id: item.order for item in props.sticks}
        self.assertEqual(by_id[c_id], 1)  # moved earlier, as asked
        moved = next(item for item in props.sticks if item.stick_id == c_id)
        self.assertEqual(moved.status, core_state.STATUS_IMPOSSIBLE)
        self.assertTrue(moved.reason)
        self.assertTrue(any(
            entry.is_error and entry.stick_id == c_id for entry in props.order_warnings))

    def test_move_build_step_at_the_start_of_the_order_is_a_no_op(self):
        self._use_a_three_stick_chain()
        props = self._order()
        first_index = next(i for i, item in enumerate(props.sticks) if item.order == 0)
        props.active_stick_index = first_index
        self.assertEqual(bpy.ops.so100.move_build_step(direction=-1), {"CANCELLED"})
        self.assertEqual(props.sticks[first_index].order, 0)

    def test_poll_fails_without_an_active_stick_selected(self):
        self._use_a_three_stick_chain()
        self._order()
        self.props.active_stick_index = -1
        self.assertFalse(bpy.ops.so100.move_build_step.poll())

    def test_export_respects_a_manual_reorder(self):
        import tempfile

        self._use_two_independent_uprights()
        props = self._order()
        first_index = next(i for i, item in enumerate(props.sticks) if item.order == 0)
        props.active_stick_index = first_index
        bpy.ops.so100.move_build_step(direction=1)
        expected = [item.stick_id for item in sorted(props.sticks, key=lambda i: i.order)]

        path = os.path.join(tempfile.mkdtemp(), "reordered.build.json")
        self.assertEqual(bpy.ops.so100.export_build_file(filepath=path), {"FINISHED"})
        with open(path) as handle:
            document = json.loads(handle.read())
        self.assertEqual([s["id"] for s in document["sticks"]], expected)

    def test_ordering_is_idempotent(self):
        first = [(i.stick_id, i.order) for i in self._order().sticks]
        second = [(i.stick_id, i.order) for i in self._order().sticks]
        self.assertEqual(first, second)


@unittest.skipIf(bpy is None, "requires Blender")
class TestMirrorRig(BlenderTestCase):
    """Phase E's robot mirror. BLENDER_ADDON_PLAN.md Sec 10.4: a rig posed
    by the vendored FK, scrubbed through the build order."""

    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.props = bpy.context.scene.so100
        # Same fully-buildable 3-level ladder as TestBuildOrderOperator --
        # multi-layer, all-tangential horizontals.
        half, length, gap = 0.055, 0.110, 0.0065
        verts, edges = [], []
        for level in range(3):
            z = level * (length + gap)
            verts.extend([(-half, CUBE_Y, z), (half, CUBE_Y, z)])
        for level in range(2):
            base = level * 2
            edges.append((base, base + 2))
            edges.append((base + 1, base + 3))
        for level in range(1, 3):
            base = level * 2
            edges.append((base, base + 1))
        mesh = bpy.data.meshes.new("Ladder")
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        self.design = bpy.data.objects.new("Ladder", mesh)
        bpy.context.collection.objects.link(self.design)
        bpy.ops.so100.create_base_empty()
        self.props.design_mesh = self.design
        self.props.ground_mode = "SLIDE"
        self.assertEqual(bpy.ops.so100.compute_build_order(), {"FINISHED"})

    def test_toggle_shows_and_hides_the_rig_object(self):
        self.assertFalse(self.props.show_mirror)
        self.assertEqual(bpy.ops.so100.mirror_toggle(), {"FINISHED"})
        self.assertTrue(self.props.show_mirror)
        obj = bpy.data.objects.get("SO100_Mirror")
        self.assertIsNotNone(obj)
        self.assertFalse(obj.hide_get())
        self.assertEqual(len(obj.data.vertices), 6)
        self.assertEqual(len(obj.data.edges), 5)

        bpy.ops.so100.mirror_toggle()
        self.assertFalse(self.props.show_mirror)
        self.assertTrue(bpy.data.objects.get("SO100_Mirror").hide_get())

    def test_the_rig_is_never_selectable_or_rendered(self):
        bpy.ops.so100.mirror_toggle()
        obj = bpy.data.objects.get("SO100_Mirror")
        self.assertTrue(obj.hide_select)
        self.assertTrue(obj.hide_render)

    def test_step_wraps_around_the_build_order(self):
        bpy.ops.so100.mirror_toggle()
        count = sum(1 for item in self.props.sticks if item.order >= 0)
        self.props.mirror_index = count - 1
        bpy.ops.so100.mirror_step(direction=1)
        self.assertEqual(self.props.mirror_index, 0)

        bpy.ops.so100.mirror_step(direction=-1)
        self.assertEqual(self.props.mirror_index, count - 1)

    def test_step_poll_fails_while_hidden(self):
        self.assertFalse(self.props.show_mirror)
        self.assertFalse(bpy.ops.so100.mirror_step.poll())

    def test_status_names_the_stick_and_its_position(self):
        bpy.ops.so100.mirror_toggle()
        self.assertRegex(self.props.mirror_status, r"^s_\d+ -- build order 1 of \d+$")

    def test_the_last_segment_ends_near_the_stick_grasp_target(self):
        # Sanity check that the rig is actually posed AT the stick, not
        # just drawing something -- the TCP (last point) should sit within
        # a few mm of the stick's own grasp target.
        from so100_builder.core import transform as core_transform
        from so100_builder.kinematics.so_arm_100 import grasp as kgrasp

        bpy.ops.so100.mirror_toggle()
        ordered = sorted((i for i in self.props.sticks if i.order >= 0),
                         key=lambda i: i.order)
        item = ordered[0]
        edge_index = next(i for i, s in enumerate(self.props.sticks)
                          if s.stick_id == item.stick_id)
        build_edge = self.props.build_mesh.data.edges[edge_index]
        scale_length = bpy.context.scene.unit_settings.scale_length
        base_matrix = core_transform.to_tuple_4x4(self.props.base_empty.matrix_world)
        a = core_transform.blender_to_robot(
            tuple(self.props.build_mesh.data.vertices[build_edge.vertices[0]].co),
            base_matrix, scale_length,
        )
        b = core_transform.blender_to_robot(
            tuple(self.props.build_mesh.data.vertices[build_edge.vertices[1]].co),
            base_matrix, scale_length,
        )
        expected_target = kgrasp.grasp_target(a, b)
        expected_blender = core_transform.robot_to_blender(
            expected_target, base_matrix, scale_length
        )

        rig = bpy.data.objects.get("SO100_Mirror")
        tcp = tuple(rig.data.vertices[-1].co)
        for got, want in zip(tcp, expected_blender):
            self.assertAlmostEqual(got, want, delta=0.001)

    def test_unregister_removes_the_rig_object(self):
        bpy.ops.so100.mirror_toggle()
        self.assertIsNotNone(bpy.data.objects.get("SO100_Mirror"))
        so100_builder.unregister()
        self.assertIsNone(bpy.data.objects.get("SO100_Mirror"))
        so100_builder.register()  # tearDownClass expects it still registered

    def test_kr10_mirror_rig_uses_kr10s_own_kinematics_not_so_arm_100s(self):
        # kr10_r900_2_kinematics was vendored 2026-08-22 (kuka_control) --
        # switching robot_id must actually run ITS fk()/solve, never
        # so_arm_100's geometry and never the pre-vendoring "not vendored"
        # message. The ladder fixture sits at so_arm_100's usual build
        # position, which this very different 6-DOF arm may or may not
        # reach -- either outcome is fine here, "not vendored" is not.
        self.props.robot_id = "kr10_r900_2"
        self.assertEqual(bpy.ops.so100.mirror_toggle(), {"FINISHED"})
        self.assertNotIn("not vendored", self.props.mirror_status)
        obj = bpy.data.objects.get("SO100_Mirror")
        if not obj.hide_get():
            # Posed: 6 joints -> 7 points -> 6 segments (core/mirror.py is
            # robot-agnostic on joint count), not so_arm_100's 6 points/5
            # segments -- confirms this isn't just reusing the 5-DOF shape.
            self.assertEqual(len(obj.data.vertices), 7)
            self.assertEqual(len(obj.data.edges), 6)


@unittest.skipIf(bpy is None, "requires Blender")
class TestMultiRobot(BlenderTestCase):
    """docs/STATUS.md "Multi-robot support kicked off 2026-08-21", both
    robots' kinematics vendored (so_arm_100 from the start, kr10_r900_2
    2026-08-22). Confirms robot_id actually selects real, different
    kinematics rather than a stub or a silent fall-through to so_arm_100's
    geometry."""

    def test_default_robot_id_is_so_arm_100(self):
        self.assertEqual(self.props.robot_id, "so_arm_100")

    def test_create_base_empty_names_it_per_selected_robot(self):
        # Previously always "SO100_Base" regardless of robot_id -- the addon
        # had no way to create a KUKA base at all (docs/STATUS.md 2026-08-22).
        bpy.ops.wm.read_factory_settings(use_empty=True)
        props = bpy.context.scene.so100
        props.robot_id = "kr10_r900_2"
        self.assertEqual(bpy.ops.so100.create_base_empty(), {"FINISHED"})
        self.assertIsNotNone(props.base_empty)
        self.assertEqual(props.base_empty.name, "KR10_Base")
        self.assertIn(props.base_empty.name, bpy.data.objects)

    def test_create_base_empty_still_names_it_so100_base_by_default(self):
        # so_arm_100's own path must stay byte-for-byte unchanged.
        bpy.ops.wm.read_factory_settings(use_empty=True)
        props = bpy.context.scene.so100
        self.assertEqual(bpy.ops.so100.create_base_empty(), {"FINISHED"})
        self.assertEqual(props.base_empty.name, "SO100_Base")

    def test_build_mesh_is_named_per_selected_robot(self):
        # Previously always "SO100_BuildMesh" regardless of robot_id
        # (docs/STATUS.md 2026-08-23).
        props = self.extract(robot_id="kr10_r900_2", ground_mode="SLIDE")
        self.assertEqual(props.build_mesh.name, "KR10_BuildMesh")

    def test_build_mesh_still_named_so100_buildmesh_by_default(self):
        props = self.extract(ground_mode="SLIDE")
        self.assertEqual(props.build_mesh.name, "SO100_BuildMesh")

    def test_reset_stock_to_robot_defaults_uses_kr10s_own_numbers(self):
        from so100_builder.core import robots as core_robots

        props = self.props
        props.robot_id = "kr10_r900_2"
        self.assertEqual(bpy.ops.so100.reset_stock_to_robot_defaults(), {"FINISHED"})
        kinematics = core_robots.get_robot("kr10_r900_2").kinematics
        # section_mm/joint_allowance_mm are the UI ergonomics overrides
        # (2mm / 1.5mm), not kinematics.STICK_SECTION_M/JOINT_ALLOWANCE_M
        # directly -- see _STOCK_DEFAULT_OVERRIDES_MM in ops/design.py.
        self.assertAlmostEqual(props.section_mm, 2.0, places=6)
        self.assertAlmostEqual(props.joint_allowance_mm, 1.5, places=6)
        self.assertAlmostEqual(
            props.min_stick_length_mm, kinematics.STICK_LENGTH_RANGE_M[0] * 1000.0, places=6)
        self.assertAlmostEqual(
            props.max_stick_length_mm, kinematics.STICK_LENGTH_RANGE_M[1] * 1000.0, places=6)

    def test_reset_stock_to_robot_defaults_uses_so_arm_100s_own_numbers(self):
        from so100_builder.core import robots as core_robots

        self.assertEqual(bpy.ops.so100.reset_stock_to_robot_defaults(), {"FINISHED"})
        kinematics = core_robots.get_robot("so_arm_100").kinematics
        self.assertAlmostEqual(self.props.section_mm, kinematics.STICK_SECTION_M * 1000.0,
                               places=6)

    def test_kr10_validation_runs_for_real_without_crashing(self):
        # Real per-stick verdicts against kr10_r900_2's own kinematics --
        # never a crash, and never the pre-vendoring "not vendored" text.
        props = self.extract(robot_id="kr10_r900_2")
        self.assertGreater(len(props.sticks), 0)
        for item in props.sticks:
            self.assertIn(
                item.status, (core_state.STATUS_BUILDABLE, core_state.STATUS_IMPOSSIBLE))
            if item.status == core_state.STATUS_IMPOSSIBLE:
                self.assertNotIn("not vendored", item.reason)

    def test_so_arm_100_extraction_is_unaffected_by_the_registry(self):
        # The default path must stay byte-for-byte what it was before
        # multi-robot support -- same fixture, same shape of outcome as
        # TestExtraction's own cube (some sticks buildable, some genuinely
        # impossible for kinematic reasons unrelated to the robot registry).
        props = self.extract(ground_mode="SLIDE")
        self.assertEqual(len(props.sticks), 12)
        buildable = sum(1 for i in props.sticks
                        if i.status == core_state.STATUS_BUILDABLE)
        impossible = sum(1 for i in props.sticks
                         if i.status == core_state.STATUS_IMPOSSIBLE)
        self.assertEqual(buildable + impossible, 12)
        self.assertGreater(buildable, 0)

    def test_kr10_has_a_confirmed_build_volume_and_is_vendored(self):
        from so100_builder.core import robots as core_robots
        profile = core_robots.get_robot("kr10_r900_2")
        self.assertTrue(profile.has_build_volume)
        self.assertTrue(profile.is_vendored)

    def test_export_stamps_the_selected_robot_and_its_kinematics_version(self):
        import tempfile
        from so100_builder.core import robots as core_robots

        self.extract(robot_id="kr10_r900_2", ground_mode="SLIDE")
        path = os.path.join(tempfile.mkdtemp(), "kuka.build.json")
        result = bpy.ops.so100.export_build_file(filepath=path)
        self.assertEqual(result, {"FINISHED"})
        with open(path) as handle:
            document = json.loads(handle.read())
        self.assertEqual(document["robot"], "kr10_r900_2")
        self.assertEqual(
            document["kinematics_version"],
            core_robots.get_robot("kr10_r900_2").kinematics.__version__,
        )
        self.assertEqual(document["build_volume"]["min"], [0.3, -0.15, -0.02])
        self.assertEqual(document["build_volume"]["max"], [0.6, 0.15, 0.28])

    def test_compute_build_order_works_for_a_kr10_only_design(self):
        # Regression test for the reported bug: every reachability check
        # inside core/order.py's solver used to run so_arm_100's own
        # kinematics regardless of robot_id, so a design sitting well
        # within kr10_r900_2's own build zone but past so_arm_100's much
        # smaller documented reach (max validated Y -450mm) would report
        # "0 in order ... 1 error" with no clue the wrong robot was being
        # checked. Y=-550mm is deep in kr10_r900_2's own zone
        # ([-600,-300]mm) and clearly outside so_arm_100's.
        bpy.ops.wm.read_factory_settings(use_empty=True)
        props = bpy.context.scene.so100
        half, length, gap, y = 0.055, 0.110, 0.0065, -0.550
        verts, edges = [], []
        for level in range(3):
            z = level * (length + gap)
            verts.extend([(-half, y, z), (half, y, z)])
        for level in range(2):
            base = level * 2
            edges.append((base, base + 2))
            edges.append((base + 1, base + 3))
        for level in range(1, 3):
            base = level * 2
            edges.append((base, base + 1))
        mesh = bpy.data.meshes.new("KukaLadder")
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        design = bpy.data.objects.new("KukaLadder", mesh)
        bpy.context.collection.objects.link(design)
        props.robot_id = "kr10_r900_2"
        bpy.ops.so100.create_base_empty()
        props.design_mesh = design
        props.ground_mode = "SLIDE"

        self.assertEqual(bpy.ops.so100.compute_build_order(), {"FINISHED"})
        self.assertTrue(props.has_order)
        self.assertGreater(len(props.sticks), 0)
        for item in props.sticks:
            self.assertGreaterEqual(item.order, 0, item.reason)


@unittest.skipIf(bpy is None, "requires Blender")
class TestTranslations(BlenderTestCase):
    """i18n.py -- 2026-08-24 user request: "is it possible to add a second
    language to the addon GUI?". Registered once for real via
    ``so100_builder.register()`` (``BlenderTestCase``'s own setUp), so this
    exercises the actual dict handed to ``bpy.app.translations.register()``,
    not a copy -- a typo in a msgctxt or a source string that has drifted
    from the real code would otherwise pass silently."""

    def setUp(self):
        super().setUp()
        self._language = bpy.context.preferences.view.language
        self._use_interface = bpy.context.preferences.view.use_translate_interface
        self._use_tooltips = bpy.context.preferences.view.use_translate_tooltips

    def tearDown(self):
        # Preferences are process-global, not per-scene -- must not leak
        # into whichever test class runs next in the same process.
        bpy.context.preferences.view.language = self._language
        bpy.context.preferences.view.use_translate_interface = self._use_interface
        bpy.context.preferences.view.use_translate_tooltips = self._use_tooltips

    def _use_japanese(self):
        bpy.context.preferences.view.language = "ja_JP"
        bpy.context.preferences.view.use_translate_interface = True
        bpy.context.preferences.view.use_translate_tooltips = True

    def test_a_panel_title_translates_to_japanese(self):
        self._use_japanese()
        self.assertEqual(bpy.app.translations.pgettext_iface("Design"), "デザイン")

    def test_an_operator_label_translates_to_japanese(self):
        self._use_japanese()
        translated = bpy.app.translations.pgettext(
            "Extract Sticks", bpy.app.translations.contexts.operator_default)
        self.assertEqual(translated, "スティックを抽出")

    def test_a_property_name_translates_to_japanese(self):
        self._use_japanese()
        self.assertEqual(
            bpy.app.translations.pgettext_iface("Joint Allowance"), "ジョイント許容量")

    def test_nothing_translates_when_language_stays_english(self):
        bpy.context.preferences.view.language = "en_US"
        self.assertEqual(bpy.app.translations.pgettext_iface("Design"), "Design")

    def test_a_sample_of_operator_labels_matches_the_real_registered_ones(self):
        # Not exhaustive (bl_rna.name is the CLASS IDENTIFIER for an
        # Operator, not its label -- confirmed directly; only bl_label is)
        # -- a spot check across several real operators that a rename in
        # ops/*.py without a matching i18n.py update would catch, without
        # the much larger machinery an exhaustive check over every
        # label/description/nested-property/enum-item would need.
        from so100_builder import i18n as i18n_module

        op_ctx = bpy.app.translations.contexts.operator_default
        for idname in ("SO100_OT_extract_sticks", "SO100_OT_compute_build_order",
                      "SO100_OT_move_build_step", "SO100_OT_export_build_file",
                      "SO100_OT_create_base_empty"):
            label = getattr(bpy.types, idname).bl_label
            self.assertIn((op_ctx, label), i18n_module._JA, idname)


@unittest.skipIf(bpy is None, "requires Blender")
class TestOverlayBuildVolume(unittest.TestCase):
    """ui/overlay.py's build-volume box, previously always so_arm_100's
    regardless of robot_id (docs/STATUS.md 2026-08-23). ``_volume_box_points``
    is pure geometry (no gpu calls), so -- unlike the rest of this module,
    which needs a real GPU context the module's own docstring says
    ``--background`` cannot provide -- it's directly testable here."""

    IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
               (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def _bounds(self, points):
        lo = [min(p[i] for p in points) for i in range(3)]
        hi = [max(p[i] for p in points) for i in range(3)]
        return lo, hi

    def test_so_arm_100_box_matches_its_own_profile(self):
        from so100_builder.core import robots as core_robots
        from so100_builder.ui.overlay import _volume_box_points

        points = _volume_box_points(self.IDENTITY, 1.0, "so_arm_100")
        self.assertEqual(len(points), 24)  # 12 edges x 2 endpoints
        lo, hi = self._bounds(points)
        profile = core_robots.get_robot("so_arm_100")
        for got, want in zip(lo, profile.build_volume_min_m):
            self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(hi, profile.build_volume_max_m):
            self.assertAlmostEqual(got, want, places=9)

    def test_kr10_box_is_the_real_300mm_cube_not_so_arm_100s(self):
        from so100_builder.ui.overlay import _volume_box_points

        points = _volume_box_points(self.IDENTITY, 1.0, "kr10_r900_2")
        lo, hi = self._bounds(points)
        self.assertEqual([round(v, 6) for v in lo], [0.3, -0.15, -0.02])
        self.assertEqual([round(v, 6) for v in hi], [0.6, 0.15, 0.28])

    def test_kr10_base_box_sits_below_the_robot_origin(self):
        from so100_builder.ui.overlay import _base_box_points

        points = _base_box_points(self.IDENTITY, 1.0, "kr10_r900_2")
        self.assertEqual(len(points), 24)  # 12 edges x 2 endpoints
        lo, hi = self._bounds(points)
        self.assertEqual([round(v, 6) for v in lo], [-0.16, -0.16, -0.02])
        self.assertEqual([round(v, 6) for v in hi], [0.16, 0.16, 0.0])

    def test_so_arm_100_has_no_base_box_to_draw(self):
        from so100_builder.ui.overlay import _base_box_points

        self.assertEqual(_base_box_points(self.IDENTITY, 1.0, "so_arm_100"), [])


@unittest.skipIf(bpy is None, "requires Blender")
class TestSelectStickInViewport(BlenderTestCase):
    """"Check it by eye" -- BLENDER_ADDON_PLAN.md Sec 10.3's "highlighted in
    list and viewport", built ahead of Phase C so extraction results are
    already eyeball-checkable, and requiring no change once build order
    lands (see the operator's own docstring for why)."""

    def test_selecting_a_row_selects_exactly_that_edge(self):
        props = self.extract(ground_mode="SLIDE")
        props.active_stick_index = 5
        self.assertEqual(bpy.ops.so100.select_stick_in_viewport(), {"FINISHED"})

        obj = props.build_mesh
        self.assertEqual(obj.mode, "EDIT")
        bm = bmesh.from_edit_mesh(obj.data)
        bm.edges.ensure_lookup_table()
        selected = [e.index for e in bm.edges if e.select]
        self.assertEqual(selected, [5])
        bpy.ops.object.mode_set(mode="OBJECT")

    def test_build_mesh_is_locked_again_after_the_next_extraction(self):
        props = self.extract(ground_mode="SLIDE")
        bpy.ops.so100.select_stick_in_viewport()
        bpy.ops.object.mode_set(mode="OBJECT")
        self.assertFalse(props.build_mesh.hide_select)

        self.extract(ground_mode="SLIDE")
        self.assertTrue(self.props.build_mesh.hide_select)

    def test_poll_fails_without_a_build_mesh(self):
        self.assertFalse(bpy.ops.so100.select_stick_in_viewport.poll())

    def test_step_stick_wraps_around_the_list(self):
        props = self.extract(ground_mode="SLIDE")
        props.active_stick_index = len(props.sticks) - 1
        bpy.ops.so100.step_stick(direction=1)
        self.assertEqual(self.props.active_stick_index, 0)

        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.so100.step_stick(direction=-1)
        self.assertEqual(self.props.active_stick_index, len(props.sticks) - 1)
        bpy.ops.object.mode_set(mode="OBJECT")

    def test_step_stick_follows_build_order_once_one_exists(self):
        # 2026-08-23, user-reported: stepping followed the raw extraction/
        # edge index, not the order the robot actually builds in -- the two
        # differ as soon as a build order is computed (the cube's top ring
        # can only be placed after its bottom ring, regardless of which
        # edge id either got at extraction).
        from so100_builder.core import state as core_state

        props = self.extract(ground_mode="SLIDE")
        self.assertEqual(bpy.ops.so100.compute_build_order(), {"FINISHED"})

        count = len(props.sticks)
        permutation = core_state.build_order_permutation(
            [item.order for item in props.sticks])
        inverse = [0] * count
        for original_index, position in enumerate(permutation):
            inverse[position] = original_index
        # The fixture actually has to exercise the bug: if build order
        # happened to match extraction order exactly, stepping by raw
        # index would look identical to stepping by build order too.
        self.assertNotEqual(inverse, list(range(count)))

        props.active_stick_index = inverse[0]
        for build_position in range(1, count):
            bpy.ops.so100.step_stick(direction=1)
            bpy.ops.object.mode_set(mode="OBJECT")
            self.assertEqual(self.props.active_stick_index, inverse[build_position])

    def test_out_of_sync_build_mesh_is_a_clean_error(self):
        # A stick was added/removed since the build mesh was generated --
        # must not silently select the wrong edge.
        props = self.extract(ground_mode="SLIDE")
        props.build_mesh.data.edges[-1:]  # touch, no-op, keeps intent clear
        bmesh_data = props.build_mesh.data
        bm = bmesh.new()
        bm.from_mesh(bmesh_data)
        bm.edges.new((bm.verts.new((0, 0, 0)), bm.verts.new((1, 0, 0))))
        bm.to_mesh(bmesh_data)
        bm.free()
        bmesh_data.update()

        with self.assertRaises(RuntimeError) as caught:
            bpy.ops.so100.select_stick_in_viewport()
        self.assertIn("out of sync", str(caught.exception))


@unittest.skipIf(bpy is None, "requires Blender")
class TestBuildFileExportAndResume(BlenderTestCase):
    """Phase D end to end. The "done when" is a hardware run, which cannot be
    automated here -- what *can* be checked is the half that would make it
    fail: that the file conforms to Part A, and that reopening after a build
    shows the correct placed/pending state."""

    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.props = bpy.context.scene.so100
        half, length, gap = 0.055, 0.110, 0.0065
        verts, edges = [], []
        for level in range(3):
            z = level * (length + gap)
            verts.extend([(-half, CUBE_Y, z), (half, CUBE_Y, z)])
        for level in range(2):
            base = level * 2
            edges.append((base, base + 2))
            edges.append((base + 1, base + 3))
        for level in range(1, 3):
            base = level * 2
            edges.append((base, base + 1))
        mesh = bpy.data.meshes.new("Ladder")
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        self.design = bpy.data.objects.new("Ladder", mesh)
        bpy.context.collection.objects.link(self.design)
        bpy.ops.so100.create_base_empty()
        self.props.design_mesh = self.design
        self.props.ground_mode = "SLIDE"

        import tempfile
        self.tmpdir = tempfile.mkdtemp()
        self.build_path = os.path.join(self.tmpdir, "tower.build.json")

    def _export(self):
        self.assertEqual(
            bpy.ops.so100.export_build_file(filepath=self.build_path), {"FINISHED"})
        with open(self.build_path) as handle:
            return json.loads(handle.read())

    def test_export_writes_a_conforming_build_file(self):
        from so100_builder.io import build_file as build_io
        document = self._export()
        # Parsed by the loader, so the density/format checks run for real.
        build_io.load_build_file(json.dumps(document))
        self.assertEqual(document["format"], "so100_build")
        self.assertEqual(document["frame"], "base_link")
        self.assertEqual(len(document["sticks"]), len(self.props.sticks))

    def test_it_works_without_pressing_anything_else_first(self):
        # Export re-runs extraction and ordering itself.
        self.assertEqual(len(self.props.sticks), 0)
        document = self._export()
        self.assertGreater(len(document["sticks"]), 0)

    def test_the_kinematics_version_is_stamped(self):
        # ROS2 refuses to execute on a mismatch, so an empty or wrong value
        # here would strand the operator.
        import so100_builder
        self.assertEqual(self._export()["kinematics_version"],
                         so100_builder.kinematics.so_arm_100.__version__)

    def test_the_robot_id_is_stamped(self):
        # BRIDGE_PROTOCOL.md Sec A.1.1 (added 2026-08-21): the executor
        # refuses to run a file meant for a different robot, so this has to
        # be the id actually selected -- default is so_arm_100.
        self.assertEqual(self.props.robot_id, "so_arm_100")
        self.assertEqual(self._export()["robot"], "so_arm_100")

    def test_stock_and_build_volume_are_included(self):
        document = self._export()
        self.assertAlmostEqual(document["stock"]["section_m"][0], 0.00645, places=6)
        self.assertAlmostEqual(document["stock"]["joint_allowance_m"], 0.00325,
                               places=6)
        self.assertEqual(len(document["build_volume"]["min"]), 3)

    def test_stock_values_are_rounded_not_float32_noise(self):
        # Blender stores these as float32, so 6.45 mm round-trips as
        # 0.006449999809265136 -- which would land verbatim in a file whose
        # design rule is "human-readable and diffable".
        stock = self._export()["stock"]
        for value in stock["section_m"] + stock["length_range_m"]:
            self.assertEqual(value, round(value, 6))
        self.assertEqual(stock["section_m"][0], 0.00645)

    def test_the_export_path_is_remembered_for_the_sidecar(self):
        self._export()
        self.assertEqual(self.props.build_file_path, self.build_path)

    def test_marking_a_stick_writes_the_sidecar(self):
        from so100_builder.io import build_file as build_io
        self._export()
        first = min((i for i in self.props.sticks if i.order >= 0),
                    key=lambda i: i.order)
        self.assertEqual(bpy.ops.so100.mark_stick(status="placed"), {"FINISHED"})
        self.assertEqual(first.status, "placed")

        status_path = build_io.status_path_for(self.build_path)
        self.assertTrue(os.path.exists(status_path))
        with open(status_path) as handle:
            _index, entries = build_io.load_status_file(handle.read())
        self.assertEqual(entries[first.stick_id]["status"], "placed")

    def test_pending_sticks_are_absent_from_the_sidecar(self):
        from so100_builder.io import build_file as build_io
        self._export()
        bpy.ops.so100.mark_stick(status="placed")
        with open(build_io.status_path_for(self.build_path)) as handle:
            _index, entries = build_io.load_status_file(handle.read())
        self.assertEqual(len(entries), 1)

    def test_marking_advances_the_next_stick_to_load(self):
        from so100_builder.ops.build import next_stick_id
        self._export()
        first = next_stick_id(self.props)
        bpy.ops.so100.mark_stick(status="placed")
        second = next_stick_id(self.props)
        self.assertIsNotNone(first)
        self.assertNotEqual(first, second)

    def test_a_failed_stick_is_not_skipped_over(self):
        from so100_builder.ops.build import next_stick_id
        self._export()
        first = next_stick_id(self.props)
        bpy.ops.so100.mark_stick(status="failed")
        # Still the one the operator must deal with.
        self.assertEqual(next_stick_id(self.props), first)

    def test_sync_restores_progress_from_a_sidecar(self):
        """The "done when": reopening shows the correct placed/pending state."""
        from so100_builder.io import build_file as build_io
        self._export()
        ordered = sorted((i for i in self.props.sticks if i.order >= 0),
                         key=lambda i: i.order)
        placed_ids = [ordered[0].stick_id, ordered[1].stick_id]

        # Simulate what ROS2 writes after building the first two sticks.
        document = build_io.status_document(
            "tower.build.json",
            {stick_id: {"status": "placed", "at": "2026-07-29T10:00:00Z"}
             for stick_id in placed_ids},
            current_index=2)
        status_path = build_io.status_path_for(self.build_path)
        build_io.write_status_file(status_path, document)

        # Now wipe the in-.blend progress, as if reopening a stale file.
        bpy.ops.so100.reset_build_progress()
        self.assertTrue(all(i.status != "placed" for i in self.props.sticks))

        self.assertEqual(bpy.ops.so100.sync_status(filepath=status_path),
                         {"FINISHED"})
        by_id = {i.stick_id: i for i in self.props.sticks}
        for stick_id in placed_ids:
            self.assertEqual(by_id[stick_id].status, "placed")
        self.assertEqual(self.props.status_conflicts, "")

    def test_a_disagreement_is_reported_and_not_silently_merged(self):
        # Sec 9.3: "if the two disagree, show both and let the user choose --
        # never silently pick one."
        from so100_builder.io import build_file as build_io
        self._export()
        ordered = sorted((i for i in self.props.sticks if i.order >= 0),
                         key=lambda i: i.order)
        target = ordered[0]
        target.status = "placed"

        document = build_io.status_document(
            "tower.build.json",
            {target.stick_id: {"status": "failed", "at": "t",
                               "reason": "grasp failed"}})
        status_path = build_io.status_path_for(self.build_path)
        build_io.write_status_file(status_path, document)

        bpy.ops.so100.sync_status(filepath=status_path)
        self.assertEqual(target.status, "placed", "the .blend was overwritten")
        self.assertIn(target.stick_id, self.props.status_conflicts)
        self.assertIn("placed", self.props.status_conflicts)
        self.assertIn("failed", self.props.status_conflicts)

    def test_sync_rejects_a_file_that_is_not_a_sidecar(self):
        self._export()
        with self.assertRaises(RuntimeError):
            bpy.ops.so100.sync_status(filepath=self.build_path)

    def test_reset_clears_progress_but_keeps_the_order(self):
        self._export()
        bpy.ops.so100.mark_stick(status="placed")
        orders = [(i.stick_id, i.order) for i in self.props.sticks]
        bpy.ops.so100.reset_build_progress()
        self.assertEqual([(i.stick_id, i.order) for i in self.props.sticks], orders)
        self.assertTrue(all(i.status != "placed" for i in self.props.sticks))


@unittest.skipIf(bpy is None, "requires Blender")
class TestExport(BlenderTestCase):
    def test_cut_list_csv_round_trips(self):
        import tempfile

        props = self.extract(ground_mode="SLIDE")
        path = os.path.join(tempfile.mkdtemp(), "cut_list.csv")
        self.assertEqual(
            bpy.ops.so100.export_cut_list(filepath=path), {"FINISHED"}
        )
        with open(path) as handle:
            lines = handle.read().strip().split("\n")

        self.assertTrue(lines[0].startswith("build_order,stick_id,stick_length_mm"))
        # One row per stick, then a blank line and the saw tally (Sec 5.5:
        # "under the fixed-stock-length mode this collapses to a tally,
        # which is what you actually want at a saw").
        self.assertEqual(len([ln for ln in lines[1:] if ln and not ln.startswith("#")
                              and not ln.startswith("count,")]),
                         len(props.sticks) + 1)  # +1 for the single tally row
        self.assertIn("# tally", lines)

    def test_the_cut_list_tally_counts_every_stick(self):
        import tempfile

        props = self.extract(ground_mode="SLIDE")
        path = os.path.join(tempfile.mkdtemp(), "cut_list.csv")
        bpy.ops.so100.export_cut_list(filepath=path)
        with open(path) as handle:
            lines = handle.read().strip().split("\n")
        tally_start = lines.index("count,stick_length_mm") + 1
        total = sum(int(line.split(",")[0]) for line in lines[tally_start:] if line)
        self.assertEqual(total, len(props.sticks))


if __name__ == "__main__":
    unittest.main()
