import os
import sys
from pathlib import Path
import unittest


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from PyQt5.QtWidgets import QApplication

from gb5753_tool.app import MainWindow


class MotionMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = MainWindow()
        self.window.connected = True
        self.window.safety_check.setChecked(True)
        self.sent_frames = []
        self.window._send_frame = lambda frame, _note="": self.sent_frames.append(frame)  # type: ignore[method-assign]

    def tearDown(self) -> None:
        self.window.connected = False
        self.window.close()

    def test_mit_target_is_captured_only_when_sent(self) -> None:
        self.window.control_tabs.setCurrentIndex(0)
        self.window.mit_position.setValue(0.2)
        self.window.mit_velocity.setValue(1.0)
        self.window.mit_torque.setValue(0.3)

        self.assertIsNone(self.window.last_motion_target)
        self.window._send_current_control()

        self.assertEqual(len(self.sent_frames), 1)
        target = self.window.last_motion_target
        self.assertIsNotNone(target)
        assert target is not None
        self.assertEqual(target.mode, "MIT")
        self.assertEqual((target.position, target.velocity, target.torque), (0.2, 1.0, 0.3))

    def test_feedback_updates_summary_and_mode_specific_missing_targets(self) -> None:
        self.window.control_tabs.setCurrentIndex(2)
        self.window.velocity_value.setValue(0.5)
        self.window._send_current_control()
        self.window._record_motion(1.25, 0.4, -0.1)

        sample = self.window.motion_samples[-1]
        self.assertIsNone(sample.target_position)
        self.assertEqual(sample.target_velocity, 0.5)
        self.assertIsNone(sample.target_torque)
        self.assertEqual(self.window.motion_labels["position_target"].text(), "—")
        self.assertEqual(self.window.motion_labels["velocity_error"].text(), "+0.10000 rad/s")
        self.assertEqual(self.window.motion_labels["torque_actual"].text(), "-0.10000 Nm")

    def test_clear_history_keeps_active_target_but_resets_samples(self) -> None:
        self.window.control_tabs.setCurrentIndex(0)
        self.window._send_current_control()
        self.window._record_motion(0.1, 0.2, 0.3)

        self.window._clear_motion_history()

        self.assertEqual(self.window.motion_samples, [])
        self.assertIsNotNone(self.window.last_motion_target)
        self.assertEqual(self.window.motion_sample_count.text(), "0 个反馈采样点（最多保留 10000 个）")


if __name__ == "__main__":
    unittest.main()
