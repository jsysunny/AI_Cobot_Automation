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
| **약 선반 및 서랍** | 일반/전문의약품 분리 저장 구조 |
| 👁 **Intel RealSense D435i, C270 Webcam** | 약 탐지 및 서랍/QR 인식을 위한 RGB-D 카메라 |
| 🎤 **Logitech HD Webcam C270** | AI 음성 인터랙션(마이크:음성 명령 인식) |
| 📢 **Bluetooth Speaker** | AI 음성 인터랙션(스피커:음성 안내 출력) |
| ✋ **OnRobot RG2 Gripper** | 다양한 크기의 의약품을 정밀하게 집기 위한 그리퍼 |
| 🤖 **Doosan M0609 협동로봇** | 의약품 선반 간 이동 및 약 그리핑 수행 |
| 📶 **HC-SRO4 Ultrasonic Sensor** | 사용자 존재 감지용 초음파 거리 센서 |
| 💡 **Raspberry Pi 4 (4GB)** | ROS2 기반 로컬 컨트롤러, LCD 및 스피커 통신 제어 |

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

Vision model 

&nbsp;
## 4. 🧭 동작 흐름 요약


&nbsp;
### 🏠 퇴근 모드

0. **사용자 아이템 리스트**  
   - 카드키, 껌, 지갑, 스낵, 텀블러

1. **홈 위치 대기**  
   - Force 센서를 활용해 충돌 감지 상태에서 대기

2. **수납 알고리즘 시작**  
   - y축 방향 외력 감지 (Check Force Condition) → 수납 알고리즘 진입  
   - 💬 음성 출력: `"Good night! Have a sweet dream"`  
   - 📺 LCD 출력: `good night!`

3. **인사 동작 (Good night)**  
   - `Move_periodic` 동작으로 인사 수행  
   - 💬 음성 출력: `"Good night!"`  
   - 📺 LCD 출력: `"Good night!"`

4. **사용자 입력**  
   - 원하는 물체 및 선반 위치 입력  
   - 예: `텀블러 1`

5. **물체 탐색**  
   - ㄹ자 구조로 반복 탐색 수행  
     - Movel 명령으로 x축 400mm, y축 50mm 탐색  
   - 📺 LCD 출력: `"Searching"` Gage 애니메이션 표시

6. **물체 분류 및 Grip 동작**  
   - 비동기 탐색 중 `Get tool force`로 외력 감지 → 물체 존재 확인  
   - 📺 LCD 출력: `"grabbed object!: {Detected name}"`  
   - 💬 음성 출력: `{Detected name}`  
   - 순응제어로 z축 위치 파악 → `height_dict`와 비교하여 분류  
   - `Release` → `Grip` 동작으로 물체 집기

7. **입력 위치에 물품 수납**  
   - **비어있는 경우**: 원래 위치 (`placed_list`)에 수납  
   - **이미 물건이 있는 경우**:  
     - 예: `stacked = [1, 0, 0, 0]` → `stacked = [2, 0, 0, 0]`  
     - x축으로 떨어진 지점에 수납  
   - 📺 LCD 출력: `"Placed object: {Detected name}"`  
   - 💬 음성 출력: `{Detected name}`

8. **그리퍼 홈 위치 복귀**  
   - 수납 완료 후, 그리퍼가 홈 위치로 이동하여 대기  
   - 📺 LCD 출력: `"Request complete"`  
   - 💬 음성 출력: `"Request complete"`

&nbsp;
### 🚪 출근 모드
0. **홈 위치 대기**  
   - Force 센서를 활용해 충돌 감지 상태에서 대기

1. **꺼내기 알고리즘 시작**  
   - x축 방향 외력 감지 (Check Force Condition) → 꺼내기 알고리즘 진입  
   - 💬 음성 출력: `"Have a nice day!"`  
   - 📺 LCD 출력: `"Hello, Have a nice day!"`

2. **인사 동작 (Hello)**  
   - `Move_periodic` 동작으로 인사 수행  
   - 💬 음성 출력: `"Hello!"`  
   - 📺 LCD 출력: `"Hello!"`

3. **사용자 입력**  
   - 원하는 물체 및 선반 위치 입력  
   - 예: `텀블러 1`

4. **물품 위치 비교 및 꺼내기**  
   - **입력값과 위치가 일치하는 경우** → 해당 위치에서 물건 꺼냄  
     - 📺 LCD 출력: `{Detected name} out`  
     - 💬 음성 출력: `{Detected name}`  
   - **불일치하는 경우** → 동작 수행하지 않음

5. **꺼낸 물품 배치**  
   - 최대 5개까지 꺼낼 수 있음  
   - 입력된 순서대로, 홈 위치에서 일정 간격으로 떨어진 위치에 배치

6. **그리퍼 홈 위치 복귀**  
   - 꺼내기 완료 후, 그리퍼가 홈 위치로 복귀  
   - 📺 LCD 출력: `"Request complete"`  
   - 💬 음성 출력: `"Request complete"`

&nbsp;
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


