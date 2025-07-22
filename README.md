# Doosan Robotics Boot Camp(2025.01.06 ~ 2025.07.06)
## 1. ROKEY B-1조 협동-2 Project (ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트) AI_Cobot_Automation
&nbsp;
## 🧠 RAAPS : Rokey Ai Automatic Pharmacy System


&nbsp;

## 🔗 출처 및 라이선스

이 프로젝트는 **두산로보틱스(Doosan Robotics Inc.)**에서 배포한 **ROS 2 기반 패키지**를 바탕으로 개발되었습니다.  
소스코드는 [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)에 따라 공개되어 있으며,  
본 저장소 또한 동일한 라이선스를 따릅니다. 자세한 내용은 `LICENSE` 파일을 참고하세요.

> ⚠️ 해당 저장소는 두산로보틱스의 **공식 저장소가 아니며**,  
> 학습 및 테스트를 위한 **비공식 수정본**을 일부 포함하고 있습니다.  
> 공식 자료는 [두산로보틱스 홈페이지](https://www.doosanrobotics.com/) 및  
> [Doosan GitHub 저장소](https://github.com/DoosanRobotics/doosan-robot2)를 참고해 주세요.

&nbsp;

## 📑 목차

1. [📌 프로젝트 개요](#1--프로젝트-개요)
2. [🔧 프로젝트 수행 절차 및 방법](#2--프로젝트-수행-절차-및-방법)  
3. [🔧 구성 요소](#3--구성-요소)  
4. [💻 사용 기술](#4--사용-기술)  
5. [🧭 동작 흐름 요약](#5--동작-흐름-요약)  
6. [💻 코드 실행 방법](#6--코드-실행-방법)  
7. [📷 시연 영상/이미지](#7--시연-영상--이미지)  
8. [🌟 기대 효과/ 한계점 및 개선점](#8--기대-효과)  

   
&nbsp;
## 1. 📌 프로젝트 개요

의약품의 조제와 복용은 사람의 생명과 직결되며, 그 정확성과 안전성이 매우 중요합니다.
본 프로젝트 **RAAPS(Rokey Ai Automatic Pharmacy System)** 는 AI Vision과 협동로봇(Cobot) 기술을 활용해 약 조제 과정의 오류를 방지하고, 고령층 및 의료인력 부족에 대응하는 스마트 약 복용 보조 시스템입니다.


&nbsp;
### 🎯 기획 의도

- 유사한 약 이름, 잘못된 복용, 약 비닐 삽입 오류 등으로 인한 **의료 사고**를 방지하고자 하였습니다.
- **AI Vision 기반 인식 기술**과 **로봇 제어 기술**을 통해 약 조제 정확도를 높이고, 사고를 예방합니다.
- **AI Voice 기반 음성 안내 시스템**을 통해 고령층 및 장애인 사용자가 스스로 약을 복용할 수 있도록 지원합니다.
- 약사의 설명 및 복약 상담 부담을 줄이고, 사용자의 **복약 편의성과 안전성**을 강화하는 것이 목적입니다.
  
&nbsp;
### 🏭 기존 기술의 활용과 협동로봇의 확장 가능성
- 기존에는 약 조제 및 복약 안내가 **수작업 및 직접 대면** 방식에 의존되어 왔으며,  
  이는 약사 부족 및 시간 제약으로 인해 오류 발생 가능성이 존재했습니다.

- RAAPS는 다음과 같은 방식으로 기존 기술을 확장하여 적용했습니다:
  - **AI Vision**으로 약품을 정확하게 인식하고,
  - **협동로봇**이 지정된 약품을 자동으로 집어 전달하며,
  - **AI Voice**가 사용자의 증상에 따라 일반약을 추천하고 복약 방법을 안내합니다.

- 협동로봇은 **안전 펜스 없이도 사람과 근접 작업**이 가능하며,  
  병원, 약국, 복지시설 등의 **좁은 환경에서도 유연하게 작동**할 수 있도록 설계되었습니다.

- 본 시스템은 향후 다음과 같은 분야로도 확장 가능성이 있습니다:
  - 스마트 병원
  - 무인 약국
  - 고령자 복약 도우미
  - 지역 약국의 자동 조제 시스템 등
    
- 향후 RAAPS는 스마트 병원, 무인 약국, 고령자 복약 지원 로봇 등 다양한 영역에 확대 적용 가능성이 있으며,
특히 의료 인력 부족 및 고령화 사회 문제에 실질적인 해결책을 제시할 수 있습니다.

&nbsp;
## 2. 🔧 프로젝트 수행 절차 및 방법 
<img width="1225" height="734" alt="image" src="https://github.com/user-attachments/assets/808cfce9-ff99-4337-b135-b82a6afbee5b" />

&nbsp;
## 3. 🔧 구성 요소

| 구성 요소 | 설명 |
|-----------|------|
| 💊 **약 재료** | 일반/전문의약품 |
| 🗄 **약 선반 및 서랍** | 일반/전문의약품 분리 저장 구조 |
| 👁 **Intel RealSense D435i** | 약 탐지 및 서랍/QR 인식을 위한 RGB-D 카메라 |
| 🎤 **Logitech HD Webcam C270** | AI 음성 인터랙션 (마이크: 음성 명령 인식) |
| 📢 **Bluetooth Speaker** | AI 음성 인터랙션 (스피커: 음성 안내 출력) |
| ✋ **OnRobot RG2 Gripper** | 다양한 크기의 의약품을 정밀하게 집기 위한 그리퍼 |
| 🤖 **Doosan M0609 협동로봇** | 의약품 선반 간 이동 및 약 그리핑 수행 |
| 📶 **HC-SRO4 Ultrasonic Sensor** | 사용자 존재 감지용 초음파 거리 센서 |
| 💡 **Raspberry Pi 4 (4GB)** | ROS2 기반 로컬 컨트롤러, LCD 및 스피커 통신 제어 |

&nbsp;
<img width="1300" height="605" alt="image" src="https://github.com/user-attachments/assets/b5a3aa08-b525-4cba-a151-5d121134610d" />

<img width="789" height="308" alt="image" src="https://github.com/user-attachments/assets/7aa27063-67d8-4f0d-b1dc-a6c906ab3975" />

<img width="748" height="324" alt="image" src="https://github.com/user-attachments/assets/e4a60790-c55f-4fd2-a668-3910116ce96c" />

<img width="586" height="341" alt="image" src="https://github.com/user-attachments/assets/7eb2deb0-e2f3-46b2-b2a5-7b268564ac44" />

<img width="670" height="375" alt="image" src="https://github.com/user-attachments/assets/a5e5671f-8083-45bc-ba5b-1c425687560c" />


&nbsp;
## 4. 💻 사용 기술

| 기술 | 내용 |
|------|------|
| 🖥 **OS 및 플랫폼** | Ubuntu 22.04, ROS2 Humble |
| 📡 **통신 미들웨어** | ROS2 Pub/Sub 기반, DDS 통신 |
| 💬 **사용 언어** | Python, DSR, C++ |
| 📷 **Vision** | YOLOv11 기반 Object Detection & Segmentation, ResNet18 Classification |
| 🗣 **Voice Recognition** | Whisper (STT), OpenWakeWord (웨이크워드 감지), GPT-4o (질의 응답 처리), Microsoft Edge TTS |
| 🧠 **AI 모델** | GPT-4o 기반 복약 설명/질의 응답, 증상-약 분류 |
| 🧪 **로봇 제어** | Force Sensor 기반 순응 제어, DSR ROS2 API 사용 |

&nbsp;
### 📷 Vision model 
**1. Object Detection (YOLOv11)**  
- 목적: 약 서랍의 라벨 텍스트(예: dermatitis, cold 등)를 박스 단위로 탐지  
- 모델: `yolov11n.pt`  
- Dataset: 20장 → 증강하여 총 60장 (Train 70% / Val 30%)  
- 하이퍼파라미터:  
  - Epoch: 200  
  - Batch size: 16  
  - IOU threshold: 0.5  
- 성능 지표:  
  - mAP@0.5 = **0.995**  
- 결과: 약 서랍 위에 부착된 라벨을 정확히 탐지하여 위치 기반 분류 가능  

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/66f84a6b-4087-4709-824d-bd150fb0c091" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/8e911d69-6535-48f5-a8ee-90e72b922055" />

&nbsp;

**2. Text Classification (ResNet18)**  
- 목적: 탐지된 라벨 이미지(text 박스)를 4종류 약 분류로 분류  
- 약 종류: cold, dermatitis, dyspepsia, diarrhea  
- 모델: `ResNet18`  
- Dataset: 20장 → 증강하여 총 80장  
- 하이퍼파라미터:  
  - Epoch: 22  
- 성능 지표:  
  - Accuracy = **1.00**  
- 결과: OCR된 라벨 이미지를 정확하게 약 카테고리로 분류

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/e71ab87e-50ae-4790-8da9-262e386c6833" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/0881a9b6-bd4d-4b7a-87de-723dd089fecd" />

&nbsp;

**3. Segmentation (YOLOv11s)**  
- 목적: 약 서랍 내부 의약품 패키지를 탐지 및 회전 각도 추정  
- 모델: `yolov11s.pt`  
- Dataset: 20장 → 증강하여 총 60장 (Train 70% / Val 30%)  
- 하이퍼파라미터:  
  - Epoch: 200  
  - Batch size: 16  
  - IOU threshold: 0.5  
- 성능 지표:  
  - mAP@0.5 ≈ **0.992 ~ 0.993**  

###  🤧 [1. Cold]  
- 탐지 클래스: `amoxicile_tab`, `ponstar_tab`  
- mAP@0.5 = **0.993**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/8b037393-0f9a-4d26-9057-0d45f7e7565d" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/5c5dcb3d-b395-46d7-a0c6-c1176115ed90" />

---

### 🤕 [2.Dermatitis]  
- 탐지 클래스: `monodoxy_cap`, `ganakan_tab`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/307c1f0a-282a-494d-b599-4d93ee1b6a0a" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/1057fded-ca00-4450-acf2-76cc6cb6fdb8" />


---

### 🤢 [3.Dyspepsia]  
- 탐지 클래스: `mogum_tab`, `medicostenter`, `nexilen_tab`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/e7440bc4-85fe-4044-a7bc-4d44d5a025e5" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/ba31c0f3-0e59-4341-ab07-5a2da85ebbef" />

---

### 💩 [4.Diarrhea]  
- 탐지 클래스: `otillen_tab`, `famodine`, `somnux_scop`  
- mAP@0.5 = **0.992**

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/65cdba23-eee7-4705-8c98-2864ebea89bc" />

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/12101f88-5f75-426c-90d3-277298caa9a6" />


&nbsp;

### 🗣 Voice model 

#### 🎙 1. OpenWakeWord  
- **모델명**: `hello_rokey_8332_32.tflite`  
- **기능**: 웨이크워드 “hello rokey” 감지를 위한 TFLite 기반 모델  
- **동작 방식**:  
  - 0.1초 간격으로 마이크 입력 버퍼에서 오디오 청크 수신  
  - `model.predict()`를 통해 inference 수행  
  - confidence score ≥ 0.6 → 웨이크워드 감지로 간주  

---

#### 📝 2. OpenAI Whisper  
- **모델명**: `whisper-1`  
- **기능**: 녹음된 오디오 파일 (예: `input.wav`)을 텍스트로 변환 (STT)  

---

#### 🤖 3. GPT-4o  
- **모델명**: `gpt-4o`  
- **기능**:  
  - 사용자의 음성 명령에서 의약품 이름 및 수량 추출  
  - 의약품 종류 분류 (전문의약품 vs 일반의약품)  
  - 증상 입력 시 약 추천  
  - 약 설명 요청 시 효능·주의사항 안내  

---

#### 🔊 4. Microsoft Edge TTS  
- **모델명**: `ko-KR-SunHiNeural`  
- **기능**:  
  - TTS(Text-to-Speech)를 통해 사용자에게 음성 안내 출력
  
&nbsp;
## 4. 🧭 동작 흐름 요약
<img width="740" height="276" alt="image" src="https://github.com/user-attachments/assets/164a3641-52d8-489a-9c19-20ac57fe4375" />

<img width="783" height="1131" alt="ROKEY_Pharmacy_detail drawio" src="https://github.com/user-attachments/assets/e68cf733-3392-4f3a-99f1-5344afc34456" />

&nbsp;

### 📋 전문의약품 노드 

1. **사람 감지**  
   - 초음파 센서 → robot  
   - 상태 메시지: `state="detected"` ROS topic publish

2. **QR 및 안내 음성 출력**  
   - robot → vision (`detect_qr`)  
   - robot → voice (처방전 안내 음성 출력)  
   - `robot_state = "check_qr"`

3. **QR 코드 인식**

4. **서랍 텍스트 인식 위치 지정**  
   - vision → robot (`qr_info`)

5. **서랍 텍스트 인식 실행**  
   - robot → vision  
   - `robot_state = "check_text"`

6. **서랍 열기 및 약 탐지 위치로 이동**  
   - robot → 서랍 앞

7. **약 위치 탐지 (YOLOv11)**  
   - robot → vision  
   - `robot_state = "detect_pill"`

8. **약 위치로 이동 및 자세 정보 추출**  
   - vision → robot  
   - 결과값: (x, y, theta)

9. **약 집기 + 비닐봉투에 담기 + 서랍 닫기**

10. **복약 설명 음성 출력**  
    - robot → voice (`task_state`)

&nbsp;

### 📋 일반의약품 노드

1. **사람 감지**  
   - 초음파 센서 → robot  
   - 상태 메시지: `state="detected"`

2. **QR 인식 및 일반약 요청 음성 출력**  
   - robot → vision / voice  
   - `robot_state = "check_qr"`

3. **음성 명령 수신 (의약품 요청)**  
   - voice → robot  
   - topic: `medicine`

4. **로봇이 선반 앞으로 이동**

5. **선반 인식 준비**  
   - robot → vision  
   - `robot_state = "shelf_state"`

6. **약 위치 탐지 (YOLOv11)**  
   - vision → robot  
   - 결과: `medicine_loc`

7. **약 pick & place**  
   - robot → vision  
   - `robot_state = "pick_medicine"`

8. **외력 인식 시 로봇 순응 제어 실행 (Gripper Release)**

9. **약 설명 음성 출력**  
   - robot → voice (`task_state`)

&nbsp;

### 💊 전문의약품 동작 과정
1. 초음파로 사람 감지
- 5~37cm 거리의 사용자가 3초 이상 머물면 감지
- Moving average 필터 사용

2. 처방전 QR 인식 자세 & 음성 안내
- 로봇 동작은 `movejs`로 자연스럽게 연결
- 음성 안내:
  `안녕하세요. rokey약국입니다. QR을 스캔하거나 "hello rokey"를 말해주세요.`

3. QR 찍기
- 처방전 파싱 → QR 코드 생성
- JSON 파일 내용: 이름, 주민등록번호, ATC 코드, 1회 투약량, 1일 투약 횟수, 총 투여일수

예시: 
정서윤 012340-4893726
A02X1 1 1 1
A07FA01 1 2 1
A02AA04 1 3 1

4. 서랍 바라보는 모션
- QR을 인식한 후 서랍 방향으로 자세 이동

5. 서랍 text 인식 후 열기
- 처방전 ATC 코드, 이름, 증상을 딕셔너리 형태로 저장
- YOLO로 텍스트 증상 인식
- (x, y) 좌표를 기준으로 4개 서랍 중 하나로 이동
-

6. 서랍 안에 바라보는 위치
- 4개의 서랍 segmentation 좌표로 이동

7. 약 탐지
- A02X1 1 1 1 → nexilen_tab 탐지
- 타원형 → 2d center x, y, theta 전송
- camera calibration → 3d 좌표로 로봇 이동

8. 약 이동 점심
- gripper 약약 크기에 맞춰 조정
- force control과 compliance로 약약 세밀하게 집기
- A02X1 1 1 1 → 1번 1번 투약 1일치 → nexilan_tab(index=1/total1) → 점심 이동
- 흔드는 모션 → 약이 잘 떨어지지 않음 방지

9. 약 탐지
- A07FA01 1 2 1 → medilacsenteric_tab 탐지
- 타원형 캡슐 → center x, y, theta 전송
- camera calibration → 3d 좌표로 로봇 이동

10. 약 이동 아침 저녁
- gripper 약약 크기에 맞춰 조정
- A07FA01 1 2 1 → 1번 2번 투약 1일치 →  
  nexilan_tab(index=1/total2) → 아침  
  nexilan_tab(index=2/total2) → 저녁

11. 약 탐지
- A02AA04 1 3 1 → magmil_tab 탐지
- 원형 캡슐 → center x, y 전송 (원형은 theta 무시)
- camera calibration → 3d 좌표로 로봇 이동

12. 약 이동 아침 점심 저녁
- gripper 약약 크기에 맞춰 조정
- A02AA04 1 3 1 → 1번 3번 투약 1일치  
  magmil_tab(index=1/total3) → 아침  
  magmil_tab(index=2/total3) → 점심  
  magmil_tab(index=3/total3) → 저녁

13. 선반 넣기
- 로봇이 약약을 force control로 집음  
→ 선반이 밑으로 내려감 → 잡고 올리기 모션 → 밀어넣기 모션

14. 포장 대기 상태 이동
- 모든 약 개수 약 주격으로 이동하면 포장 상태로 이동

15. 약사가 약 포장 후 외력
- 약사가 약 검사 & 고데기로 약 비닐 포장
- `check_force_condition` x축 외력 감지

16. 약 봉투로 이동 및 해당 약 설명
- `check_force_condition` x축 외력 감지  
- voice_nexilan_tab:  
  “해당 약은 위염치료제이며 다른 약 복용 시 위 손상을 막아줍니다. 감사합니다. 안녕히가세요.”
  
## 5. 💻 코드 실행 방법

### 🤖 Robot Control Node
- 코드: [`robot_control_node.py`](./rokey_project/rokey_project/robot_control_node.py)

```bash
ros2 run rokey_project robot_control_node
```
### 🍓 Raspberry Pi Node
- 코드: [`feedback_node.py`](./rokey_project/rokey_project/feedback_node.py)

```bash
ros2 run rokey_project feedback_node
```
&nbsp;
## 6. 📷 시연 영상 / 이미지
> https://youtu.be/bbBvETzXTgY

> <img width="600" height="415" alt="image" src="https://github.com/user-attachments/assets/52fde78e-1d48-4131-9ba5-a52d8baa4287" />

> <img width="600" height="415" alt="image" src="https://github.com/user-attachments/assets/24839a30-8b8b-4170-aeda-596b4a016ea2" />

> <img width="600" height="392" alt="image" src="https://github.com/user-attachments/assets/a3af83a5-ea75-4e62-b817-4eddd1cb01de" />

> <img width="300" height="300" alt="image" src="https://github.com/user-attachments/assets/bfb44ccb-1988-4563-abb9-64399405e04d" />


&nbsp;
## 7. 🌟 기대 효과

- 일상생활에 협동로봇 도입 가능성 증진
- 출퇴근 시간의 불편함 해소
- 다양한 물품 및 센서로의 확장성 기대

### ⚠️ 한계점 및 개선점

- 그리퍼의 중량, 가속도/속도에 따른 force 조절 미흡 → 물체 밀림 발생
- 선반 높이 제한 → 워크스페이스 조정 필요

&nbsp;
## 🙌 팀원

- 백홍하,정서윤,정민섭,서형원


