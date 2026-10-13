"""Read selected entries in public ZIP datasets using bounded HTTP Range requests."""
import io
import time
from urllib.request import Request, urlopen


class RemoteZipFile(io.RawIOBase):
    def __init__(self, url):
        self.url, self.pos, self.downloaded = url, 0, 0
        with urlopen(Request(url, method='HEAD'), timeout=30) as r:
            self.size = int(r.headers['Content-Length'])
            self.etag = r.headers.get('ETag')

    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos
    def seek(self, offset, whence=0):
        self.pos = offset if whence == 0 else self.pos + offset if whence == 1 else self.size + offset
        return self.pos

    def read(self, n=-1):
        n = min(self.size-self.pos, n if n >= 0 else self.size-self.pos)
        if n <= 0: return b''
        if n > 250_000_000: raise ValueError('Refusing an unbounded dataset download')
        headers = {'Range': f'bytes={self.pos}-{self.pos+n-1}'}
        if self.etag: headers['If-Match'] = self.etag
        for attempt in range(4):
            try:
                with urlopen(Request(self.url, headers=headers), timeout=120) as r:
                    if r.status != 206: raise ValueError('Server did not honor HTTP Range')
                    data = r.read(n+1)
                if len(data) != n: raise ValueError('Truncated or oversized range')
                self.pos += n
                self.downloaded += n
                return data
            except Exception:
                if attempt == 3: raise
                time.sleep(attempt+1)
