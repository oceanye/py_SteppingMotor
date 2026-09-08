"""方向文案与软件位置符号约定（2026-09-07 用户指定）。

直线轴：向下(OUTWARD)=负、向上(INWARD)=正——最低点归零后向上为正，
顶死失步时读到负值即异常指示。旋转轴：逆时针(OUTWARD)=正、顺时针=负。
其余轴保持 OUTWARD=正 的旧约定。
"""

import unittest

from motor_control import (
    AXIS_LABEL,
    NUM_STEPPER_AXES,
    direction_label_parts,
    outward_position_sign,
)


class AxisDirectionConventionTests(unittest.TestCase):
    def test_linear_axes_count_upward_inward_as_positive(self):
        for axis in (0, 1):
            self.assertIn("直", AXIS_LABEL[axis])
            self.assertEqual(outward_position_sign(axis), -1)

    def test_rotary_axes_count_counterclockwise_outward_as_positive(self):
        for axis in (2, 3):
            self.assertIn("转", AXIS_LABEL[axis])
            self.assertEqual(outward_position_sign(axis), +1)

    def test_other_axes_keep_outward_positive(self):
        for axis in range(4, NUM_STEPPER_AXES):
            self.assertEqual(outward_position_sign(axis), +1)

    def test_labels_stay_consistent_with_sign_convention(self):
        for axis in range(NUM_STEPPER_AXES):
            out_txt, in_txt, _out_arrow, _in_arrow = direction_label_parts(axis)
            sign = outward_position_sign(axis)
            if "直" in AXIS_LABEL[axis]:
                self.assertEqual((out_txt, in_txt), ("向下", "向上"))
                self.assertEqual(sign, -1)
            elif "转" in AXIS_LABEL[axis]:
                self.assertEqual((out_txt, in_txt), ("逆时针", "顺时针"))
                self.assertEqual(sign, +1)
            else:
                self.assertEqual((out_txt, in_txt), ("正向", "反向"))
                self.assertEqual(sign, +1)


if __name__ == "__main__":
    unittest.main()
