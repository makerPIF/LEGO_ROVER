"""Incremental, bounded parser for Content-Length multipart JPEG streams."""
class MjpegParser:
    def __init__(self, limit=1024*1024):
        self.limit = limit
        self.buffer = bytearray()
        self.expected = None

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            if self.expected is None:
                end = self.buffer.find(b'\r\n\r\n')
                if end < 0:
                    if len(self.buffer) > 8192:
                        raise ValueError('MJPEG header too large or missing')
                    break
                headers = bytes(self.buffer[:end]).lower().split(b'\r\n')
                del self.buffer[:end+4]
                length = [h.split(b':',1)[1].strip() for h in headers if h.startswith(b'content-length:')]
                if len(length) != 1:
                    raise ValueError('MJPEG Content-Length missing or duplicate')
                self.expected = int(length[0])
                if not 4 <= self.expected <= self.limit:
                    raise ValueError('MJPEG frame exceeds limit')
            if len(self.buffer) < self.expected:
                break
            frame = bytes(self.buffer[:self.expected])
            del self.buffer[:self.expected]
            self.expected = None
            if not (frame.startswith(b'\xff\xd8') and frame.endswith(b'\xff\xd9')):
                raise ValueError('Invalid JPEG frame')
            frames.append(frame)
        return frames
