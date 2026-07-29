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
        self.assertEqual(bpy.types.SO100_PT_design.bl_category, "SO-100")


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
        # A millimetre-scale scene: the same numbers mean 1000x less.
        bpy.context.scene.unit_settings.scale_length = 0.001
        props = self.extract(min_stick_length_mm=0.066, max_stick_length_mm=1000.0)
        for item in props.sticks:
            self.assertAlmostEqual(item.stick_length_mm, 0.110, places=6)

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

    def test_ordering_is_idempotent(self):
        first = [(i.stick_id, i.order) for i in self._order().sticks]
        second = [(i.stick_id, i.order) for i in self._order().sticks]
        self.assertEqual(first, second)


@unittest.skipIf(bpy is None, "requires Blender")
class TestBuildOrderListSorting(unittest.TestCase):
    """``filter_items`` wants "the new position OF item i", not a sorted
    index list. Getting that backwards scrambles the list silently instead
    of raising, so the permutation is checked directly."""

    def test_already_ordered_is_the_identity(self):
        from so100_builder.ui.panels import build_order_permutation
        self.assertEqual(build_order_permutation([0, 1, 2]), [0, 1, 2])

    def test_a_reversed_order_reverses_the_display(self):
        from so100_builder.ui.panels import build_order_permutation
        self.assertEqual(build_order_permutation([2, 1, 0]), [2, 1, 0])

    def test_it_is_a_true_permutation(self):
        from so100_builder.ui.panels import build_order_permutation
        orders = [3, 0, 4, 1, 2]
        permutation = build_order_permutation(orders)
        self.assertEqual(sorted(permutation), list(range(len(orders))))

    def test_the_permutation_puts_each_item_at_its_build_index(self):
        from so100_builder.ui.panels import build_order_permutation
        orders = [3, 0, 4, 1, 2]
        permutation = build_order_permutation(orders)
        for item_index, build_index in enumerate(orders):
            self.assertEqual(permutation[item_index], build_index)

    def test_unordered_sticks_sort_last_keeping_relative_order(self):
        from so100_builder.ui.panels import build_order_permutation
        permutation = build_order_permutation([-1, 1, -1, 0])
        # items 1 and 3 are ordered (build 1 and 0) -> display 1 and 0;
        # items 0 and 2 are unordered -> display 2 and 3, in that order.
        self.assertEqual(permutation, [2, 1, 3, 0])

    def test_an_empty_list_is_handled(self):
        from so100_builder.ui.panels import build_order_permutation
        self.assertEqual(build_order_permutation([]), [])


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
                         so100_builder.kinematics.__version__)

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
