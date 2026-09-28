# ESP32-CAM AP 탐사 영상 — SPIKE Rover GCS

BLE 조종이 정상 동작한 tunnel-v2에 ESP32-CAM 영상 화면을 추가했습니다.
SPIKE 허브의 hub_rover.py는 변경하지 않았습니다. 이미 동작하는 허브 코드는 다시 업로드하지 않아도 됩니다.

## 연결 구성

- PC ↔ SPIKE Prime: 기존 Bluetooth BLE로 모터 조종과 텔레메트리.
- PC ↔ ESP32-CAM: 카메라가 만든 Wi-Fi AP로 접속하고 HTTP MJPEG 영상 수신.
- ESP32-CAM은 독립 전원만으로 실행. SPIKE 허브에 신호선 연결 불필요.
- 인터넷/공유기/휴대폰 핫스팟 없이 동작합니다. AP를 만드는 장치는 ESP32-CAM입니다.

## 기본 설정

| 항목 | 값 |
|---|---|
| 대상 보드 | AI-Thinker ESP32-CAM + OV2640 (다른 보드는 핀맵 수정 필요) |
| AP 이름 | ROVER-CAM |
| AP 암호 | rovercam123 |
| AP 주소 | 192.168.4.1 |
| GCS 영상 URL | http://192.168.4.1:81/stream |
| 브라우저 확인 | http://192.168.4.1/ |
| 상태 조회 | http://192.168.4.1/status |
| 해상도 | PSRAM 있음: 640×480 / 없음: 320×240 |
| 목표 FPS | 10 (실제 속도는 무선 상태·전원·촬영 조건에 따라 달라짐) |
| 플래시 LED | 기본 꺼짐 |

AP 암호/SSID/채널, JPEG 품질, 목표 FPS, 상하/좌우 반전은 esp32cam_ap.ino 상단에서 바꿉니다.
여러 카메라를 사용하면 SSID를 다르게 설정하세요.

## 1. ESP32-CAM에 최초 업로드

1. Arduino IDE의 보드 매니저에서 Espressif Systems의 esp32 패키지 3.x를 설치합니다.
2. `esp32cam_ap/esp32cam_ap.ino`를 엽니다. 스케치 폴더와 ino 이름을 그대로 유지하세요.
3. 보드는 `AI Thinker ESP32-CAM`, PSRAM 설정 항목이 있으면 Enabled를 선택합니다.
4. ESP32-CAM-MB USB 업로더가 있으면 그것을 사용해 업로드합니다. USB-UART를 쓰면 보드별 업로드 배선을 확인하고 UART 논리 전압은 3.3V로 맞춥니다. GPIO0을 GND에 연결한 채 리셋하여 다운로드 모드로 들어간 뒤 업로드합니다.
5. 수동 GPIO0-GND 연결을 했다면 **업로드 후 반드시 제거하고 리셋**합니다. 연결을 남기면 전원만 켰을 때 정상 실행되지 않습니다.
6. 시리얼 모니터 115200 baud에서 AP와 IP를 확인합니다.
7. 업로드가 끝나면 데이터 케이블 없이 독립 전원만 연결해 사용할 수 있습니다.

전원은 AI-Thinker 보드의 **5V와 GND에 안정된 5V**를 공급합니다. 5V를 3.3V 핀에 넣지 마세요. USB 전원이나 안정된 5V 전원 모듈을 사용할 수 있습니다. 작은 USB-UART의 3.3V 출력으로 카메라 전원을 공급하는 구성은 피하세요. 사용하는 보드의 전원 입력 표기를 확인하세요.
보조배터리는 저전류 자동 종료 기능 때문에 촬영 도중 꺼지는지 확인합니다.

펌웨어는 AP를 먼저 만들고 카메라를 초기화합니다. 카메라 초기화 실패 시에도 `/status`에서 camera_ready와 camera_error를 확인하도록 구성했습니다. Brownout 감지는 끄지 않았습니다.

## 2. PC 및 GCS에서 접속

1. 이 묶음의 PC 파일들을 같은 폴더에 풉니다. **rover_gcs.py만 복사하면 안 됩니다.** `camera_panel.py`, `mjpeg_parser.py`도 필요합니다.
2. 기존처럼 `python -m pip install -r requirements.txt` 후 `python rover_gcs.py`를 실행합니다. 추가 OpenCV 패키지는 필요 없습니다.
3. ESP32-CAM에 전원을 넣습니다.
4. PC Wi-Fi 메뉴에서 ROVER-CAM을 선택하고 암호 rovercam123을 입력합니다. 인터넷 없음으로 표시되어도 이 연결을 유지하세요.
5. GCS의 카메라 주소가 `http://192.168.4.1:81/stream`인지 확인하고 [영상 연결]을 누릅니다.
6. 기존 방식으로 SPIKE 허브를 BLE 연결하여 `조종 준비 완료`를 확인합니다. Wi-Fi와 BLE는 함께 사용합니다.
7. 영상으로 진행 방향을 확인하며 방향 패드 또는 WASD로 조종합니다.

GCS의 [Wi-Fi 설정]은 Windows의 Wi-Fi 설정 화면을 엽니다. **GCS가 운영체제의 Wi-Fi 연결을 자동으로 변경하는 기능은 아닙니다.** macOS/Linux에서는 OS의 Wi-Fi 메뉴를 사용하세요. [영상 연결]은 이미 접속된 네트워크에서 스트림에 연결하는 버튼입니다.

하나의 Wi-Fi 어댑터를 사용하면 일반적으로 기존 공유기 Wi-Fi 연결을 대신해 카메라 AP에 연결합니다. 이 경우 인터넷이 끊길 수 있지만 로버 조종과 영상에는 인터넷이 필요 없습니다.

## 화면 기능

- 비율 유지 영상 표시와 수신 FPS/마지막 프레임 경과 시간.
- 최신 프레임 위주 표시. 표시된 시간은 수신 후 경과 시간이며 실제 촬영부터 화면 표시까지의 전체 지연 측정값이 아닙니다.
- 영상 오류 시 2초 후 재연결 시도.
- 3초 이상 새 프레임이 없으면 이전 영상을 지우고 끊김 표시.
- [영상 끊기면 정지] 기본 ON. 영상 연결을 요청한 상태에서 프레임이 없으면 주행을 차단합니다. 끊기면 E를 보내고 허브 워치독도 적용됩니다. 영상이 돌아온 뒤 새 입력으로 조종합니다.
- [영상 해제]를 누르면 정지 요청 후 카메라 감시를 종료합니다. 이후 BLE만으로 조종할 수 있습니다. 처음부터 카메라 연결을 누르지 않았다면 BLE만 사용하는 것도 가능합니다.
- [좌우 반전]은 GCS 표시만 바꿉니다.
- [사진 저장]은 현재 수신 JPEG 원본을 저장합니다. 화면의 좌우 반전은 저장 사진에 적용하지 않습니다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| ROVER-CAM이 안 보임 | 5V 전원, 업로드 성공, GPIO0-GND 해제, 재시작, 실제 보드 핀맵 |
| AP는 보이는데 영상 없음 | PC가 해당 AP에 연결됐는지, URL의 :81/stream, /status 응답 |
| HTTP 503 | 카메라 초기화 실패. 플랫 케이블/핀맵/전원/시리얼 오류 확인 |
| 브라우저에서는 보이나 GCS에서 안 보임 | 브라우저 영상 탭을 닫고 GCS에서 다시 연결. 이 펌웨어는 영상 시청자 1명을 기준으로 함 |
| 영상이 자주 멎거나 재부팅 | 전원과 Wi-Fi 신호 확인, AP_CHANNEL 1/6/11 비교, 해상도를 QVGA로 낮추거나 TARGET_FPS를 낮춤 |
| Wi-Fi는 연결됐는데 주소 접속 불가 | 다른 네트워크/VPN의 192.168.4.x 대역 충돌 여부, PC Wi-Fi IP 확인 |
| 영상이 없을 때 모터도 안 움직임 | 영상 연결 요청 상태에서 [영상 끊기면 정지]가 켜져 있는지 확인. BLE만 쓸 때는 [영상 해제] |
| 카메라 AP 연결 후 인터넷 안 됨 | 정상적인 독립 AP 동작. 인터넷 제공 기능은 없음 |

## 검증

- 기존 BLE/허브 회귀 테스트 20개 통과.
- MJPEG 분할 헤더/프레임·크기 제한·오류 처리 테스트 6개 통과.
- 실제 PySide6/QtNetwork와 로컬 모의 HTTP 카메라 서버로 JPEG 수신/디코딩, 영상 끊김 표시, 주행 차단, 연결 해제 통합 테스트 통과.
- GCS 창 생성 및 화면 렌더링 확인.
- 사용자 확인으로 기존 tunnel-v2의 실제 BLE 조종은 동작한 상태입니다. 이번 카메라 변경의 실제 ESP32-CAM 촬영 및 Wi-Fi/BLE 동시 운용은 장비에서 추가 확인이 필요합니다.
- 이 환경에 Arduino ESP32 툴체인이 없어 ino의 보드용 컴파일/업로드는 실행하지 못했습니다.

```bash
python test_regression.py
python test_camera.py
python test_camera_gui.py
```

## 참고

Espressif 공식 CameraWebServer의 카메라 설정/핀맵과 HTTP 서버 API, SoftAP API를 기준으로 작성했습니다.
- https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
- https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html
