"""The look-at must bring the face to the image center, not settle short of it."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lumen.behavior.look_at import DEADBAND, LookAt   # noqa: E402

HALF_FOV_DEG = 22.0     # a face at the frame edge is this far off the camera axis


class LookAtTest(unittest.TestCase):
    def test_centers_a_face_below_center(self):
        look, face_deg, t = LookAt(), 15.0, 0.0      # face 15 deg below the camera axis
        for _ in range(15 * 10):                     # 10 s at 15 fps
            t += 1 / 15
            ey = (face_deg - look.head) / HALF_FOV_DEG   # turning the head moves the face in the image
            look.update((0.0, ey), t, 1 / 15)
        self.assertLess(abs(face_deg - look.head) / HALF_FOV_DEG, DEADBAND)
        self.assertTrue(look.centered(t))
        self.assertGreater(look.head, 0)                 # +head = look down

    def test_no_face_holds_and_is_not_centered(self):
        look = LookAt()
        look.update((0.0, 0.5), 0.0, 0.5)
        head = look.head
        look.update(None, 1.0, 0.5)
        self.assertEqual(look.head, head)
        self.assertFalse(look.centered(1.0))


if __name__ == "__main__":
    unittest.main()
