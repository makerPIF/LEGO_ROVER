"""ESP32-CAM MJPEG viewer using asynchronous QtNetwork; no OpenCV required."""
import sys
import time
from PySide6.QtCore import Qt, QTimer, QUrl, Signal, QSettings
from PySide6.QtGui import QImage, QPixmap, QDesktopServices
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkProxy
from PySide6.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QLineEdit,
    QPushButton, QLabel, QCheckBox, QFileDialog, QSizePolicy)
from mjpeg_parser import MjpegParser

class CameraPanel(QGroupBox):
    lost = Signal()
    log = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__('ESP32-CAM · 탐사 영상', parent)
        self.settings = QSettings('SPIKE-Rover', 'Camera')
        self.manager = QNetworkAccessManager(self)
        self.manager.setProxy(QNetworkProxy(QNetworkProxy.NoProxy))
        self.reply = None
        self.requested = False
        self.last_frame = 0.0
        self.attempt_at = 0.0
        self.image = QImage()
        self.jpeg = b''
        self.frames = 0
        self.rate_at = time.monotonic()
        self.warned = False
        self.parser = MjpegParser()
        layout = QVBoxLayout(self)
        help_text = QLabel('PC Wi-Fi → ROVER-CAM 연결 후 [영상 연결] · BLE 조종과 동시 사용')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        row = QHBoxLayout()
        self.url = QLineEdit(self.settings.value('url', 'http://192.168.4.1:81/stream'))
        self.url.setPlaceholderText('http://192.168.4.1:81/stream')
        row.addWidget(self.url, 1)
        self.connect_button = QPushButton('영상 연결')
        self.connect_button.clicked.connect(self.start)
        self.stop_button = QPushButton('영상 해제')
        self.stop_button.clicked.connect(self.stop)
        row.addWidget(self.connect_button); row.addWidget(self.stop_button)
        layout.addLayout(row)
        self.screen = QLabel('카메라 미연결')
        self.screen.setAlignment(Qt.AlignCenter)
        self.screen.setMinimumSize(280, 210)
        self.screen.setMaximumHeight(400)
        self.screen.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.screen.setStyleSheet('background:#05080c; color:#91a8bc; border:1px solid #294254;')
        layout.addWidget(self.screen, 1)
        self.status = QLabel('Wi-Fi AP: ROVER-CAM  /  기본 암호: rovercam123')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        row = QHBoxLayout()
        self.stop_on_loss = QCheckBox('영상 끊기면 정지 (3초)')
        self.stop_on_loss.setChecked(True)
        self.mirror = QCheckBox('좌우 반전')
        self.mirror.toggled.connect(self.render)
        row.addWidget(self.stop_on_loss); row.addWidget(self.mirror)
        snap = QPushButton('사진 저장'); snap.clicked.connect(self.snapshot)
        wifi = QPushButton('Wi-Fi 설정'); wifi.clicked.connect(self.wifi_settings)
        row.addWidget(snap); row.addWidget(wifi)
        layout.addLayout(row)
        for b in (self.connect_button, self.stop_button, snap, wifi, self.mirror, self.stop_on_loss):
            b.setFocusPolicy(Qt.NoFocus)
        self.retry = QTimer(self); self.retry.setSingleShot(True)
        self.retry.timeout.connect(self.open_stream)
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick)
        self.timer.start(250)

    def motion_allowed(self):
        return (not self.requested or not self.stop_on_loss.isChecked()
                or (self.last_frame > 0 and time.monotonic() - self.last_frame < 3.0))

    def start(self):
        url = QUrl(self.url.text().strip())
        if url.scheme() != 'http' or not url.host() or url.userName() or url.password():
            self.status.setText('카메라의 http://주소:81/stream 형식으로 입력하세요.')
            return
        self.stop()
        self.active_url = url
        self.settings.setValue('url', url.toString())
        self.requested = True
        self.warned = False
        self.last_frame = 0.0
        self.frames = 0
        self.rate_at = time.monotonic()
        self.jpeg = b''
        self.image = QImage()
        self.screen.setText('카메라 연결 중…')
        self.url.setEnabled(False)
        self.connect_button.setEnabled(False)
        self.open_stream()

    def open_stream(self):
        if not self.requested or self.reply is not None:
            return
        self.attempt_at = time.monotonic()
        self.parser = MjpegParser()
        request = QNetworkRequest(self.active_url)
        request.setRawHeader(b'Accept', b'multipart/x-mixed-replace')
        request.setRawHeader(b'Cache-Control', b'no-cache')
        request.setTransferTimeout(5000)
        reply = self.manager.get(request)
        reply.setReadBufferSize(256*1024)
        self.reply = reply
        reply.readyRead.connect(lambda: self.receive(reply))
        reply.finished.connect(lambda: self.finished(reply))
        self.status.setText('영상 연결 중… PC가 ROVER-CAM Wi-Fi에 연결되어 있어야 합니다.')

    def receive(self, reply):
        if reply is not self.reply or not self.requested:
            return
        try:
            code = reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)
            if code is not None and int(code) != 200:
                raise ValueError(f'HTTP {code}')
            kind = str(reply.header(QNetworkRequest.ContentTypeHeader) or '').lower()
            if 'multipart/x-mixed-replace' not in kind:
                raise ValueError('MJPEG 주소가 아닙니다. :81/stream을 확인하세요.')
            # Drain current network data, display only the newest complete frame.
            newest = None
            while reply.bytesAvailable():
                frames = self.parser.feed(bytes(reply.read(32768)))
                if frames:
                    newest = frames[-1]
                    self.frames += len(frames)
            if newest is None:
                return
            decoded = QImage.fromData(newest, 'JPEG')
            if decoded.isNull():
                raise ValueError('JPEG 디코딩 실패')
            self.image, self.jpeg = decoded, newest
            self.last_frame = time.monotonic()
            self.warned = False
            self.render()
        except Exception as exc:
            self.status.setText(str(exc))
            self.log.emit('카메라: ' + str(exc), 'warn')
            reply.abort()

    def finished(self, reply):
        if reply is not self.reply:
            reply.deleteLater()
            return
        self.reply = None
        reason = reply.errorString()
        reply.deleteLater()
        if self.requested:
            self.status.setText('영상 연결 종료 · 2초 후 재연결: ' + reason)
            self.retry.start(2000)

    def tick(self):
        if not self.requested:
            return
        now = time.monotonic()
        age = now - self.last_frame if self.last_frame else float('inf')
        if age >= 3:
            self.screen.clear()
            self.screen.setText('영상 수신 대기 / 끊김\nWi-Fi와 카메라 전원을 확인하세요.')
            if not self.warned and self.stop_on_loss.isChecked():
                self.warned = True
                self.lost.emit()
            if self.reply is not None and now - self.attempt_at > 6:
                self.reply.abort()
        elif now - self.rate_at >= 1:
            fps = self.frames / (now - self.rate_at)
            self.status.setText(f'{self.image.width()}×{self.image.height()} · 수신 {fps:.1f} FPS · 마지막 프레임 {age:.1f}초 전')
            self.frames = 0
            self.rate_at = now

    def render(self, *_):
        if self.image.isNull() or not self.last_frame or time.monotonic() - self.last_frame >= 3:
            return
        image = self.image.mirrored(True, False) if self.mirror.isChecked() else self.image
        self.screen.setPixmap(QPixmap.fromImage(image).scaled(
            self.screen.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.render()

    def snapshot(self):
        if not self.jpeg or not self.last_frame or time.monotonic() - self.last_frame >= 3:
            self.status.setText('현재 수신 중인 영상이 없습니다.')
            return
        path, _ = QFileDialog.getSaveFileName(self, '카메라 원본 사진 저장',
            time.strftime('rover_%Y%m%d_%H%M%S.jpg'), 'JPEG (*.jpg)')
        if path:
            try:
                with open(path, 'wb') as fp:
                    fp.write(self.jpeg)
                self.log.emit('카메라 사진 저장: ' + path, 'info')
            except OSError as exc:
                self.log.emit('사진 저장 실패: ' + str(exc), 'err')

    def wifi_settings(self):
        if sys.platform == 'win32':
            QDesktopServices.openUrl(QUrl('ms-settings:network-wifi'))
        else:
            self.status.setText('운영체제 Wi-Fi 메뉴에서 ROVER-CAM 선택 → 암호 rovercam123 입력')

    def stop(self):
        was_requested = self.requested
        self.requested = False
        self.retry.stop()
        old, self.reply = self.reply, None
        if old is not None:
            old.abort(); old.deleteLater()
        self.last_frame = 0.0
        self.jpeg = b''
        self.image = QImage()
        self.screen.clear(); self.screen.setText('카메라 미연결')
        self.status.setText('영상 해제 · BLE 로버 조종은 별도 연결입니다.')
        self.url.setEnabled(True)
        self.connect_button.setEnabled(True)
        if was_requested and self.stop_on_loss.isChecked():
            self.lost.emit()
