## 🎯 프로젝트 개요
이 프로젝트는 LLM 기반 음성 명령 → 테이블 상태 분석 → 좌표 기반 로봇 이동 → 식기 수거/세팅 → GUI 연동까지
하나의 통합 파이프라인으로 구성된 음성 기반 Bussing Robot 시스템입니다.

### 프로젝트 동작 흐름
: 음성 명령으로 테이블 번호와 작업(clean/setting)을 지정하면,
로봇이 해당 테이블을 스캔하여 식기를 수거/세팅하고 GUI로 움직임을 시각화합니다
1. Wake-up Word 인식: 사용자가 "알렉사" 등의 호출어를 말하면 음성 인식 노드가 활성화됩니다.

2. 음성 명령 처리 (LLM): Whisper가 음성을 텍스트로 변환, LLM이 “테이블 번호 + 작업(clean/setting)”을 해석
3. 로봇의 테이블 이동: 강화학습(RL) 기반 이동 → 물리 경로 따라 테이블로 접근
4. 식기 존재 여부 확인: 로봇 도착 후 YOLO 카메라로 테이블 상태 분석
   - 식기가 있다면 → 좌표 계산 후 로봇 팔 이동 → 집기 → 수거함 이동
   - 식기가 없다면 → 해당 테이블 작업 완료로 판단
5. 퇴식구 복귀 & 세팅 여부 처리
   - 수거 로직 완료 후 로봇은 퇴식구로 복귀
   - 세팅 작업 명령이었다면 세팅 작업 수행
6. 작업 완료 → 다시 대기 상태 복귀: 새로운 음성 명령을 받을 준비 상태로 돌아갑니다.
---

## 🧩 환경 및 설치 가이드 (Installation)

### 1. 사용 환경 (Environment)

- OS: Ubuntu 24.04 LTS
- ROS2: Jazzy  
- Python: 3.12.x
- Node.js: 24.11.1
- npm: 11.6.2

> ⚠ 이 프로젝트는 **ROS2 + Python 노드 + FastAPI(WebSocket) + React GUI** 를 함께 사용합니다.  
> ROS2 워크스페이스와 Node.js 환경이 모두 필요합니다.


### 2. ROS2 워크스페이스 & 패키지 구성

```bash
# ROS2 워크스페이스 생성 (이미 있다면 생략)
mkdir -p ~/llm_ws/src
cd ~/llm_ws/src
```
```bash
# 프로젝트 클론
git clone https://github.com/your-org/YongYiHanYong.git
cd YongYiHanYong
```


### 3. Python 가상환경 생성 (Strongly Recommended)
Ubuntu 24.04에서는 pip가 시스템 환경을 보호하기 때문에
반드시 가상환경(venv) 사용을 권장합니다.
``` bash
cd ~/llm_ws/src/YongYiHanYong

python3 -m venv .venv
source .venv/bin/activate
```


### 4. Python 의존성 설치 (requirements.txt)

아래는 프로젝트에서 필요한 파이썬 의존성입니다:

requirements.txt:
``` txt
scipy==1.16.3
openwakeword==0.4.0
sounddevice==0.5.3
python-dotenv==1.2.1
PyAudio==0.2.13
onnxruntime==1.23.2
langchain_openai==1.1.0
langchain_core==1.1.0
openai==2.8.1
fastapi==0.122.0
uvicorn==0.38.0
numpy==2.3.5
```

설치:
``` bash
pip install --upgrade pip
pip install -r requirements.txt
---

### 5. OpenAI API 키 설정 (.env)

음성 명령 파싱을 위한 LLM 호출은 .env 파일에서 API 키를 읽습니다.
```bash
nano ~/llm_ws/src/YongYiHanYong/llm_for_pick_place_voice/resource/.env
```
예시:
``` env
OPENAI_API_KEY=sk-xxxxxx_your_key_here
```

### 6. ROS2 빌드
``` bash
cd ~/llm_ws

# ROS2 기본 환경 로드
source /opt/ros/jazzy/setup.bash

# 워크스페이스 빌드
colcon build --symlink-install

# 오버레이 적용
source install/setup.bash

```
⚠ package.xml과 setup.py에서
get_keyword · ros_web_bridge 노드를 entry point로 등록해야
ros2 run 명령을 사용할 수 있습니다.

---

## 📘 실행 가이드 1 (런치파일)
### 1. launch 파일 실행
``` bash
cd ~/llm_ws

# ROS2 기본 환경 로드
source /opt/ros/jazzy/setup.bash

# 오버레이 적용
source install/setup.bash

ros2 launch 런치 파일(추가 예정).py

```
---

## 📗 실행 가이드 2 (개별 ROS2 노드 실행)
### 1. React GUI 실행
이 GUI는 **React + Vite** 기반으로 구현되어 있으며,  
모든 프론트엔드 의존성은 `package.json` 에 정의되어 있습니다.

GUI 소스 위치:
``` bash
cd ~/llm_ws/src/YongYiHanYong/src/bussing_gui/robot-topview
```
#### 1-1) Node 패키지 설치
``` bash
npm install
```
#### 1-2) 실행
``` bash
npm run dev
```
브라우저에서 표시된 주소(예: `http://localhost:5173`)로 접속하면
Top View GUI와 음성 상태 패널을 확인할 수 있습니다.

### 2. ROS2 클라이언트 노드 실행
#### 2-1) 음성 명령 통합 제어 클라이언트 노드 (get_keyword_client)
ROS2 서버 노드에 서비스 요청을 보내,
음성 → 키워드 → 타겟 좌표 → RL 이동 → 일반 이동을 수행시키는 클라이언트 노드입니다.
(웨이크업 워드 감지 → STT → LLM 파싱을 한 사이클 수행시키는 클라이언트 노드입니다.)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice get_keyword_client
```
참고: get_keyword_client 대신 직접 서비스 호출로 테스트할 수도 있습니다.
``` bash
ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
```
### 3. ROS2 서버 노드 실행
#### 3-1) ROS & Web Bridge (FastAPI WebSocket 서버)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice ros_web_bridge
```
#### 3-2) LLM + STT 음성 처리 서버 노드 (get_keyword)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice get_keyword
```
#### 3-3) Pick 동작 수행 서버 노드 (ppo_position_control_node)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice pick_rl_node
```
#### 3-4) Place 동작 수행 서버 노드 (ppo_position_control_node)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice place_rl_node
```
#### 3-4) 로봇 제어 서버 노드 (doosan_robot_api_controller)
``` bash
cd ~/llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice 
```

---


## 🔁 GUI 프로세스 및 사용 가이드

이 프로젝트는 **ROS2 + LLM → WebSocket → React** 구조로 동작하며,
음성 입력에서 GUI 로봇 이동까지 다음 순서로 진행됩니다.

1. **웨이크업 워드 감지**  
   ROS2 노드가 마이크 스트림에서 wakeup word("Alexa")를 감지합니다.

2. **음성 인식(STT)**  
   감지 후 6초간 음성을 녹음하고 텍스트로 변환합니다.

3. **LLM 기반 명령 파싱**  
   변환된 문장을 LLM(GPT-4o)에 전달해  
   `table` 과 `action(clean/setting)` 을 추출합니다.
   table의 경우 1~4번만 유효합니다.

4. **ROS → WebSocket 브로드캐스트**  
   추출된 명령과 음성 상태는 WebSocket(`/ws/keywords`)으로 전송됩니다.

5. **React GUI 업데이트**  
   브라우저는 WebSocket 이벤트를 받아 
   테이블/작업을 표시하고 로봇 Dot 이동을 시작합니다.

6. **로봇 Dot 애니메이션 이동**  
   로봇 Dot은 경유지(PATH_MAP)를 따라 목표 테이블까지 애니메이션으로 이동합니다.

7. **도착 이벤트 React → ROS2 전송**  
   - 도착 시 WebSocket(`/ws/robot_events`)으로 ROS에  
   `"arrived:<table>"` 메시지를 보내 후속 로봇 작업을 트리거합니다.
   - ROS의 로봇 제어 노드가 해당 테이블에서 실제 동작(clean/setting)을 수행합니다.
   - **모든 작업이 완료되면, 다시 웨이크업 워드로 새로운 음성 명령을 입력할 수 있습니다.**
---

## 📸 시현 영상
### 1. 실제 실행 화면 (GUI)
- GUI는 2D top-view 기반이며, 좌측 패널에서 음성 입력 상태, 인식된 명령,  
및 현재 이동 중인 목표 테이블 정보를 확인할 수 있습니다.

#### 1-1) 시작 화면 (Idle)
프로그램 실행 직후 로봇은 기본 위치에 있으며 마이크는 웨이크업 워드 대기 상태입니다.

![Idle](./src/bussing_gui/robot-topview/src/assets/UI_1.png)

#### 1-2) 웨이크업 워드 감지 후 음성 명령 입력 화면 (Listening + STT)
웨이크업 워드가 감지되면 마이크가 Listening 상태로 전환되고 사용자의 음성 명령(STT 결과)이 화면에 실시간으로 표시됩니다.

![Listening + STT](./src/bussing_gui/robot-topview/src/assets/UI_2.png)

#### 1-3) 로봇 이동 화면 (Moving)
LLM이 분석한 테이블/작업 정보에 따라 로봇 Dot이 해당 테이블까지 애니메이션으로 이동하는 장면입니다.

![Moving](./src/bussing_gui/robot-topview/src/assets/UI_3.png)
