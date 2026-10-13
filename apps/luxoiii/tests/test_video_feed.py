import unittest
from unittest.mock import patch
from luxoiii.desk.video import VideoFeed


class VideoRecoveryTests(unittest.TestCase):
    def test_timeout_does_not_repeat_cached_image(self):
        feed=VideoFeed();feed.seq=3;feed.jpeg=b'old'
        self.assertEqual(feed.next_frame(3,timeout=0),(3,None))
        self.assertEqual(feed.next_frame(2,timeout=0),(3,b'old'))

    def test_capture_error_hides_old_image(self):
        feed=VideoFeed();feed.seq=3;feed.jpeg=b'old';feed.error='USB disconnected'
        self.assertEqual(feed.next_frame(2,timeout=0),(3,None))

    def test_existing_viewer_retries_capture_after_disconnect(self):
        feed=VideoFeed();feed.clients=1
        calls=[]
        def capture():
            calls.append(True)
            if len(calls)==1:feed.error='USB disconnected'
            else:feed.error=None;feed.clients=0
        with patch.object(feed,'_capture',side_effect=capture),patch('luxoiii.desk.video.time.sleep'):
            feed._run()
        self.assertEqual(len(calls),2)
        self.assertIsNone(feed.error)
        self.assertIsNone(feed._thread)
