# 사용 가이드: 하이브리드 방식 (서비스 + 자동 감지)

## 시스템 구조

### 서버 (get_keyword)
- **자동 모드**: 타이머로 wakeup word를 자동 감지하고 처리
- **서비스 모드**: 클라이언트가 서비스 요청 시 즉시 처리
- 두 모드 모두 활성화 가능 (하이브리드)

### 클라이언트 (get_keyword_client)
- get_keyword 서비스를 반복 호출
- 응답 받으면 다른 노드로 전달 (TODO)
- 자동으로 재호출하여 연속 동작

## 사용 방법

### 방법 1: 서비스 기반 (클라이언트 사용)

```bash
# 터미널 1: 서버 실행
ros2 run llm_for_pick_place_voice get_keyword

# 터미널 2: 클라이언트 실행
ros2 run llm_for_pick_place_voice get_keyword_client
```

**흐름**:
1. 클라이언트가 서비스 호출
2. 서버가 wakeup word 대기
3. 감지 후 STT → 키워드 추출
4. 클라이언트에게 응답 전송 (table:action 형식)
5. 클라이언트가 다른 노드로 전달
6. 다시 1번으로 (무한 반복)

### 방법 2: 자동 감지 (서버만 사용)

```bash
# 서버만 실행
ros2 run llm_for_pick_place_voice get_keyword
```

**흐름**:
1. 서버가 자동으로 wakeup word 감지
2. 감지 후 STT → 키워드 추출
3. `keyword_topic`으로 발행
4. 다시 1번으로 (무한 반복)

토픽 확인:
```bash
ros2 topic echo /keyword_topic
```

### 방법 3: 하이브리드 (서버 + 클라이언트 + 토픽 구독)

```bash
# 터미널 1: 서버 실행
ros2 run llm_for_pick_place_voice get_keyword

# 터미널 2: 클라이언트 실행 (서비스로 제어)
ros2 run llm_for_pick_place_voice get_keyword_client

# 터미널 3: 토픽 모니터링
ros2 topic echo /keyword_topic
```

- 클라이언트 요청 시에도 `keyword_topic` 발행
- 여러 노드가 동시에 결과 수신 가능

## 서비스 인터페이스

### get_keyword 서비스
- **타입**: `std_srvs/srv/Trigger`
- **요청**: 없음 (Trigger)
- **응답**:
  ```
  bool success      # 성공 여부
  string message    # "table1:clean table3:setting" 형식
  ```

### 응답 형식
```
table1:clean table3:setting
```
- 각 테이블과 액션을 `:`으로 구분
- 여러 테이블은 공백으로 구분

## 토픽 인터페이스

### /keyword_topic
- **타입**: `std_msgs/msg/String`
- **내용**: JSON 형식
```json
{
  "type": "keyword",
  "tables": ["table1", "table3"],
  "command": "clean",
  "text": "1번이랑 3번 테이블 치워줘"
}
```

### /voice_state
- **타입**: `std_msgs/msg/String`
- **값**:
  - `"waiting"`: Wakeup word 대기 중
  - `"listening"`: 음성 녹음 중 (STT 진행)

## 주요 변경 사항

### get_keyword.py (서버)
- ✅ 타이머 루프로 자동 감지 (0.1초 주기)
- ✅ 서비스 콜백 활성화 (클라이언트 요청 처리)
- ✅ `is_processing` 플래그로 동시 처리 방지
- ✅ `auto_mode` 플래그로 자동 감지 제어

### get_keyword_client.py (클라이언트)
- ✅ Trigger 서비스 반복 호출
- ✅ 응답 파싱 (table:action)
- ✅ 다른 노드로 전달 준비 (TODO 구현 필요)
- ✅ 에러 처리 및 자동 재시도

## 클라이언트에서 다른 노드로 전달하기

### 1. 커스텀 서비스 생성 (권장)

`srv/KeywordCommand.srv`:
```
# Request
string[] tables
string command
string text
---
# Response
bool success
string message
```

### 2. 클라이언트 수정

```python
class GetKeywordClient(Node):
    def __init__(self):
        # ...
        # 로봇 제어 서비스 클라이언트 추가
        self.robot_client = self.create_client(
            KeywordCommand, 
            'robot_control'
        )
    
    def forward_to_other_node(self, tables, actions):
        req = KeywordCommand.Request()
        req.tables = tables
        req.command = actions[0] if actions else ""
        req.text = self.latest_keyword.get("text", "")
        
        future = self.robot_client.call_async(req)
        future.add_done_callback(self.forward_response_callback)
```

### 3. 로봇 제어 노드 생성

```python
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.srv = self.create_service(
            KeywordCommand,
            'robot_control',
            self.control_callback
        )
    
    def control_callback(self, req, res):
        # 로봇 제어 로직
        self.get_logger().info(
            f"Received: tables={req.tables}, command={req.command}"
        )
        
        # TODO: 실제 로봇 제어
        
        res.success = True
        res.message = "Robot control completed"
        return res
```

## 장점

### 서비스 방식 (클라이언트 사용)
- ✅ 명확한 요청-응답 흐름
- ✅ 클라이언트가 제어권 보유
- ✅ 다른 노드로 체계적 전달 가능
- ✅ 에러 처리 및 재시도 용이

### 자동 감지 방식 (서버만)
- ✅ 간단한 사용 (서버만 실행)
- ✅ 낮은 지연시간
- ✅ 여러 노드 동시 구독 가능

### 하이브리드
- ✅ 두 방식의 장점 모두 활용
- ✅ 유연한 확장성

## 문제 해결

### Q: 서비스가 응답하지 않아요
```bash
# 서비스 확인
ros2 service list

# 서비스 타입 확인
ros2 service type /get_keyword

# 수동 호출 테스트
ros2 service call /get_keyword std_srvs/srv/Trigger
```

### Q: "busy" 응답이 나와요
- 서버가 이미 처리 중입니다
- 잠시 후 자동으로 재시도됩니다

### Q: 마이크 오류가 발생해요
```bash
# 마이크 디바이스 확인
python3 -c "import pyaudio; p = pyaudio.PyAudio(); print([(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())])"
```

## 빌드 및 실행

```bash
cd /home/yong/YongYiHanYong
colcon build --packages-select llm_for_pick_place_voice
source install/setup.bash

# 서비스 기반 사용
ros2 run llm_for_pick_place_voice get_keyword &
ros2 run llm_for_pick_place_voice get_keyword_client

# 자동 감지 사용
ros2 run llm_for_pick_place_voice get_keyword
```
