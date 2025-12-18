## 🧩 환경 및 설치 가이드 (Installation)

### 1. 사용 환경 (Environment)

- OS: Ubuntu 24.04 LTS
- ROS2: Jazzy  
- Python: 3.12.x
- Node.js: 24.11.1
- npm: 11.6.2

> ⚠ 이 프로젝트는 **ROS2 + Python 노드 + FastAPI(WebSocket) + React GUI** 를 함께 사용합니다.  
> ROS2 워크스페이스와 Node.js 환경이 모두 필요합니다.

---

### 2. 두산 로봇 설정

```bash
# 아래 두산 github에 자신과 맞는 ros2 버전에 따라 설치
https://github.com/DoosanRobotics/doosan-robot2/tree/humble
```
```bash
# 두산 로봇 움직임을 위한 package 덮어 씌우기
ros2_ws/src/doosan-robot2/dsr_bringup과 dsr_example에 현 깃허브에 ros2_ws폴더의 파일 붙여 넣기
```

---

### 3. isaaclab 설정
학습을 위해 isaaclab 폴더에 현 깃허브 isaacsim 폴더에 있는 파일 붙여넣기 

---

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
```
---

### 5. OpenAI API 키 설정 (.env)

음성 명령 파싱을 위한 LLM 호출은 .env 파일에서 API 키를 읽습니다.
```bash
nano ~/YongYiHanYong/colcon_ws/llm_for_pick_place_voice/resource/.env
```
예시:
``` env
OPENAI_API_KEY=your_key_here
```

---

### 6. ROS2 빌드
``` bash
cd ~/YONGYIHANYONG/colcon_ws

# ROS2 기본 환경 로드
source /opt/ros/jazzy/setup.bash

# 워크스페이스 빌드
colcon build --symlink-install

# 오버레이 적용
source install/setup.bash

```

---

### 7. insterface_pkg 
llm_for_pick_place_voice패키지와 동일한 서비스 인터페이스를 가지기 위해 interface_pkg를 ros2_ws/src에 복사한다.

## 8. 실행
## 8-0. 설치
```
cd ~/YONGYIHANYONG/colcon_ws
colcon build
cd ~/YONGYIHANYONG/ros2_ws
colcon build
source ~/YONGYIHANYONG/colcon_wsinstall/setup.bash
source ~/YONGYIHANYONG/ros2_ws/install/setup.bash
```
### 8-1 ROS & Web Bridge (FastAPI WebSocket 서버)
``` bash
cd ~/YONGYIHANYONG/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run llm_for_pick_place_voice ros_web_bridge
```
---

### 8-2. React GUI 실행
이 GUI는 **React + Vite** 기반으로 구현되어 있으며,  
모든 프론트엔드 의존성은 `package.json` 에 정의되어 있습니다.

GUI 소스 위치:
``` bash
cd ~/YongYiHanYong/colcon_ws/src/bussing_gui/robot-topview
```
##### Node 패키지 설치
``` bash
npm install
```
##### 실행
``` bash
npm run dev
```
브라우저에서 표시된 주소(예: `http://localhost:5173`)로 접속하면
Top View GUI와 음성 상태 패널을 확인할 수 있습니다.

---
### 8-3. 로봇 동작 노드 실행
```bash
ros2 launch llm_for_pick_place_voice llm_pick_place_voice.launch.py
```
다른 2개의 터미널에 아래 명령어 각각 실행
```

cd ~/YONGYIHANYONG/ros2_ws
ros2 run dsr_example pick
```
```
cd ~/YONGYIHANYONG/ros2_ws
ros2 run dsr_example place
```
---
### 발표자료

[![PPT 미리보기](ppt_thumbnail.png)](https://docs.google.com/presentation/d/1gKDBglbkKiGBZN7u62VMBC9nMkdloyiE/edit?usp=drive_link&ouid=113187910485490670619&rtpof=true&sd=true)

---
### 최종 시연 영상

## 🎥 Demo Video
[![yongyihan 조 시연 영상 bussing mate](https://img.youtube.com/vi/wBLrudZ3EMo/0.jpg)](https://youtu.be/wBLrudZ3EMo?si=vk7Fah76RO8QvoYz)


