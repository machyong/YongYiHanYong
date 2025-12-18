# 마이크 재개 처리 설명

## 문제 상황
기존에는 서비스 응답 후 마이크가 대기 상태로 돌아가지 않아 다시 wakeup word를 감지할 수 없었습니다.

## 해결 방법
처리 완료 후 자동으로 마이크 스트림 상태를 확인하고 재개하는 로직을 4곳에 추가했습니다.

---

## 마이크 재개 처리 위치

### [처리 1] `wakeup_detection_loop()` - 스트림 확인 및 재개
**파일**: `get_keyword.py`  
**라인**: ~175-185

```python
# ============================================================
# [마이크 재개 처리 1] 스트림이 닫혀있으면 다시 열기
# ============================================================
if not self.mic_controller.stream or not self.mic_controller.stream.is_active():
    try:
        self.mic_controller.open_stream()
        self.wakeup_word.set_stream(self.mic_controller.stream)
        self.publish_voice_state("waiting")
        self.get_logger().info("[Auto Mode] Waiting for wakeupword...")
    except OSError as e:
        self.get_logger().error(f"Error: Failed to open audio stream: {e}")
        return
```

**설명**: 타이머 루프(0.1초 주기)에서 매번 스트림 상태를 확인합니다. 스트림이 닫혀있거나 비활성화되어 있으면 자동으로 다시 엽니다.

---

### [처리 2] `wakeup_detection_loop()` - 처리 완료 후 플래그 해제
**파일**: `get_keyword.py`  
**라인**: ~190-193

```python
if self.wakeup_word.is_wakeup():
    self.is_processing = True
    self.get_logger().info("[Auto Mode] Wakeupword detected!")
    self.process_voice_command()
    # ============================================================
    # [마이크 재개 처리 2] 처리 완료 후 플래그 해제 → 다음 루프에서 다시 감지 가능
    # ============================================================
    self.is_processing = False
```

**설명**: 음성 명령 처리가 완료되면 `is_processing` 플래그를 `False`로 변경합니다. 이렇게 하면 다음 타이머 루프에서 다시 wakeup word를 감지할 수 있습니다.

---

### [처리 3] `get_keyword_callback()` - 서비스 모드 스트림 확인
**파일**: `get_keyword.py`  
**라인**: ~254-267

```python
# ============================================================
# [마이크 재개 처리 3] 서비스 모드에서도 스트림 확인 및 재개
# ============================================================
if not self.mic_controller.stream or not self.mic_controller.stream.is_active():
    try:
        self.mic_controller.open_stream()
        self.wakeup_word.set_stream(self.mic_controller.stream)
    except OSError as e:
        self.get_logger().error(f"Error: Failed to open audio stream: {e}")
        res.success = False
        res.message = f"audio_error: {e}"
        self.is_processing = False
        return res
```

**설명**: 클라이언트가 서비스를 호출했을 때도 스트림 상태를 확인하고 필요하면 재개합니다.

---

### [처리 4] `get_keyword_callback()` - 서비스 처리 완료 후 플래그 해제
**파일**: `get_keyword.py`  
**라인**: ~318-322

```python
except Exception as e:
    self.get_logger().error(f"Error: Failed processing: {e}")
    res.success = False
    res.message = f"error: {e}"
finally:
    # ============================================================
    # [마이크 재개 처리 4] 서비스 처리 완료 후 플래그 해제
    # ============================================================
    self.is_processing = False

return res
```

**설명**: 서비스 처리가 완료되면(성공하든 실패하든) `finally` 블록에서 `is_processing` 플래그를 해제합니다. 이렇게 하면 다음 서비스 호출이나 자동 감지 모드가 정상 작동합니다.

---

## 동작 원리

### 자동 모드 (타이머 루프)
```
1. 타이머(0.1초) 실행
2. [처리 1] 스트림 상태 확인 → 닫혀있으면 재개
3. wakeup word 감지 대기
4. 감지되면 처리
5. [처리 2] 플래그 해제
6. 1번으로 돌아감 (반복)
```

### 서비스 모드 (클라이언트 요청)
```
1. 클라이언트 서비스 호출
2. [처리 3] 스트림 상태 확인 → 닫혀있으면 재개
3. wakeup word 감지 대기
4. 감지되면 처리
5. [처리 4] finally 블록에서 플래그 해제
6. 응답 전송
7. 클라이언트가 다시 호출 (반복)
```

---

## 핵심 포인트

1. **is_processing 플래그**: 동시 처리 방지 + 처리 완료 신호
2. **스트림 상태 확인**: 매 루프/호출마다 스트림이 열려있는지 확인
3. **finally 블록**: 에러가 발생해도 반드시 플래그 해제
4. **타이머 루프**: 0.1초마다 자동으로 상태 확인 및 감지

이 4가지 처리로 인해 마이크가 항상 대기 상태로 유지되며, 한 번 처리 후에도 계속 wakeup word를 감지할 수 있습니다.
