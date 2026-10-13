"""Run independent face and hand interpreters concurrently on the UNO Q CPU.

Each interpreter has exactly one owner per invocation; both jobs are drained on
failure before another frame can use either interpreter. Face checks are never
skipped by this scheduling optimization.
"""
from concurrent.futures import ThreadPoolExecutor, wait
import time


class Inference:
    def __init__(self, face, hands, parallel=True):
        self.face, self.hands = face, hands
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='luxoiii-model') if parallel else None
        self.timings = {}

    def _call(self, name, fn, frame):
        start = time.perf_counter()
        try:
            return fn(frame)
        finally:
            self.timings[name] = round((time.perf_counter() - start) * 1000, 2)

    def run(self, frame, run_faces=True, run_hands=True):
        self.timings = {}
        if self.pool and run_faces and run_hands:
            face = self.pool.submit(self._call, 'face_ms', self.face.detect, frame)
            hands = self.pool.submit(self._call, 'hands_ms', self.hands.detect, frame)
            try:
                result = face.result()
                hands.result()
                return result
            finally:
                wait((face, hands))
        faces = self._call('face_ms', self.face.detect, frame) if run_faces else []
        if run_hands:
            self._call('hands_ms', self.hands.detect, frame)
        return faces

    def close(self):
        if self.pool:
            self.pool.shutdown(wait=True)
