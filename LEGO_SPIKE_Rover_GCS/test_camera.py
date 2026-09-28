import unittest
from mjpeg_parser import MjpegParser

def part(payload):
    return b'\r\n--roverframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(payload)).encode()+b'\r\n\r\n'+payload

class CameraParserTests(unittest.TestCase):
    def test_fragmented_header_and_jpeg(self):
        payload=b'\xff\xd8abcdef\xff\xd9'
        wire=part(payload)*2
        p=MjpegParser(); out=[]
        for byte in wire: out.extend(p.feed(bytes([byte])))
        self.assertEqual(out,[payload,payload])
    def test_many_frames_in_one_read(self):
        image=b'\xff\xd8a\xff\xd9'
        self.assertEqual(MjpegParser().feed(part(image)*10),[image]*10)
    def test_oversize_rejected(self):
        with self.assertRaises(ValueError):
            MjpegParser(limit=8).feed(part(b'\xff\xd8abcdefgh\xff\xd9'))
    def test_bad_jpeg_rejected(self):
        with self.assertRaises(ValueError): MjpegParser().feed(part(b'no image'))
    def test_header_bounded(self):
        with self.assertRaises(ValueError): MjpegParser().feed(b'x'*8193)
    def test_invalid_length(self):
        with self.assertRaises(ValueError):
            MjpegParser().feed(b'--x\r\nContent-Length: bad\r\n\r\n')

if __name__=='__main__': unittest.main(verbosity=2)
