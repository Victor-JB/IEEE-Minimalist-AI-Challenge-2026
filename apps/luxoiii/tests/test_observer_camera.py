"""Direct-camera freshness at the observer boundary, without a device or HTTP server."""
import importlib.util
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[3] / "tools/spatial/observe.py"
spec = importlib.util.spec_from_file_location("observer_camera", SOURCE)
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class Stop:
    def __init__(self):
        self.stopped = False
        self.waits = []

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.waits.append(seconds)
        return self.stopped


class Feed:
    def __init__(self, events, stop):
        self.events = iter(events)
        self.stop = stop
        self.error = None
        self.entered = self.exited = False
        self.sequences = []

    def watch(self):
        return self

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, *_):
        self.exited = True

    def next_frame(self, previous, timeout):
        self.sequences.append(previous)
        try:
            sequence, jpeg, self.error = next(self.events)
        except StopIteration:
            self.stop.set()
            return previous, None
        if isinstance(jpeg, Exception):
            raise jpeg
        return sequence, jpeg


class ObserverCamera(unittest.TestCase):
    def test_real_capture_sequence_keeps_identical_scene_fresh(self):
        stop = Stop()
        feed = Feed([(1, b"same image", None), (2, b"same image", None)], stop)
        published = []
        ticks = iter([10., 11.])
        observer.capture_camera(feed, stop, lambda *p: published.append(p), lambda: next(ticks))
        self.assertEqual(published, [(b"same image", 10.), (b"same image", 11.), (None, 0.)])
        self.assertTrue(feed.entered and feed.exited)

    def test_cached_sequence_timeout_and_unplug_clear_freshness_then_recover(self):
        stop = Stop()
        feed = Feed([(1, b"first", None), (1, b"cached", None),
                     (1, None, None), (1, None, "USB camera unplugged"),
                     (2, b"recovered", None)], stop)
        published = []
        observer.capture_camera(feed, stop, lambda *p: published.append(p), lambda: 10.)
        self.assertEqual(published, [(b"first", 10.), (None, 0.), (None, 0.),
                                    (None, 0.), (b"recovered", 10.), (None, 0.)])
        self.assertEqual(stop.waits, [.1])
        self.assertTrue(feed.exited)

    def test_unexpected_capture_error_unsubscribes_and_clears_frame(self):
        stop = Stop()
        feed = Feed([(1, RuntimeError("capture failed"), None)], stop)
        published = []
        with self.assertRaisesRegex(RuntimeError, "capture failed"):
            observer.capture_camera(feed, stop, lambda *p: published.append(p))
        self.assertEqual(published, [(None, 0.)])
        self.assertTrue(feed.exited)


if __name__ == "__main__":
    unittest.main()
