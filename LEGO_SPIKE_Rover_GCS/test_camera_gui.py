"""Offscreen QtNetwork integration test against a local MJPEG server.
Run: QT_QPA_PLATFORM=offscreen python test_camera_gui.py
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from PySide6.QtCore import QBuffer, QIODevice
from PySide6.QtGui import QImage, QColor
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from camera_panel import CameraPanel

app = QApplication.instance() or QApplication([])
source = QImage(320,240,QImage.Format_RGB32)
source.fill(QColor('#285a80'))
buf = QBuffer(); buf.open(QIODevice.WriteOnly); source.save(buf,'JPEG')
jpeg = bytes(buf.data())
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type','multipart/x-mixed-replace;boundary=roverframe')
        self.end_headers()
        try:
            for _ in range(30):
                self.wfile.write(b'\r\n--roverframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg)
                self.wfile.flush()
                time.sleep(.1)
        except (BrokenPipeError,ConnectionResetError): pass

server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
panel=CameraPanel(); panel.log.connect(lambda a,b: print(a, flush=True)); panel.resize(650,450); panel.show()
saved_url=panel.settings.value('url')
panel.url.setText(f'http://127.0.0.1:{server.server_port}/stream')
losses=[]; panel.lost.connect(lambda:losses.append(True))
panel.start()
for _ in range(50):
    QTest.qWait(100)
    if panel.last_frame: break
assert not panel.image.isNull(), panel.status.text()
assert panel.image.width()==320 and panel.motion_allowed()
# Camera stall must visibly invalidate the frame and block motion.
panel.last_frame=time.monotonic()-4
panel.tick()
assert not panel.motion_allowed() and losses
assert '끊김' in panel.screen.text()
panel.stop(); assert panel.reply is None and not panel.requested
panel.settings.setValue('url',saved_url or 'http://192.168.4.1:81/stream')
panel.close(); server.shutdown()
print('PASS: Qt MJPEG receive/decode; frame loss; motion gate; disconnect')
