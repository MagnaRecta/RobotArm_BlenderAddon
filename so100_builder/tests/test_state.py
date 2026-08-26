"""core/state.py -- pure-Python state helpers, no bpy required.

``build_order_permutation`` used to live in ``ui/panels.py`` (and could
only be tested under Blender as a result, since that module imports
``bpy.types`` at load time) until 2026-08-23, when ``ops/design.py``'s
Check By Eye stepping needed it too and importing ``ui.panels`` from
``ops.design`` would have been circular (``ui.panels`` already imports
from ``ops.design``). Moving it here fixed both problems at once.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from so100_builder.core import state as S  # noqa: E402


class TestBuildOrderPermutation(unittest.TestCase):
    """``filter_items`` wants "the new position OF item i", not a sorted
    index list. Getting that backwards scrambles the list silently instead
    of raising, so the permutation is checked directly."""

    def test_already_ordered_is_the_identity(self):
        self.assertEqual(S.build_order_permutation([0, 1, 2]), [0, 1, 2])

    def test_a_reversed_order_reverses_the_display(self):
        self.assertEqual(S.build_order_permutation([2, 1, 0]), [2, 1, 0])

    def test_it_is_a_true_permutation(self):
        orders = [3, 0, 4, 1, 2]
        permutation = S.build_order_permutation(orders)
        self.assertEqual(sorted(permutation), list(range(len(orders))))

    def test_the_permutation_puts_each_item_at_its_build_index(self):
        orders = [3, 0, 4, 1, 2]
        permutation = S.build_order_permutation(orders)
        for item_index, build_index in enumerate(orders):
            self.assertEqual(permutation[item_index], build_index)

    def test_unordered_sticks_sort_last_keeping_relative_order(self):
        permutation = S.build_order_permutation([-1, 1, -1, 0])
        # items 1 and 3 are ordered (build 1 and 0) -> display 1 and 0;
        # items 0 and 2 are unordered -> display 2 and 3, in that order.
        self.assertEqual(permutation, [2, 1, 3, 0])

    def test_an_empty_list_is_handled(self):
        self.assertEqual(S.build_order_permutation([]), [])


if __name__ == "__main__":
    unittest.main()
