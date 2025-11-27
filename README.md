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


## 📸 실제 실행 화면 (GUI)
- GUI는 2D top-view 기반이며, 좌측 패널에서 음성 입력 상태, 인식된 명령,  
및 현재 이동 중인 목표 테이블 정보를 확인할 수 있습니다.

### 1) 시작 화면 (Idle)
프로그램 실행 직후 로봇은 기본 위치에 있으며  
마이크는 웨이크업 워드 대기 상태입니다.
![Idle](./src/bussing_gui/robot-topview/src/assets/UI_1.png)

### 2) 웨이크업 워드 감지 후 음성 명령 입력 화면 (Listening + STT)
웨이크업 워드가 감지되면 마이크가 Listening 상태로 전환되고  
사용자의 음성 명령(STT 결과)이 화면에 실시간으로 표시됩니다.
![Listening + STT](./src/bussing_gui/robot-topview/src/assets/UI_2.png)

### 3) 로봇 이동 화면 (Moving)
LLM이 분석한 테이블/작업 정보에 따라  
로봇 Dot이 해당 테이블까지 애니메이션으로 이동하는 장면입니다.
![Moving](./src/bussing_gui/robot-topview/src/assets/UI_3.png)
