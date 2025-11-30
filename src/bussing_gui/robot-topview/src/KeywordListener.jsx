import { useEffect } from "react";

export const KeywordListener = ({ onKeyword, onVoiceState }) => {
  useEffect(() => {
    // 현재 페이지 호스트에 맞춰서 WebSocket 연결 (시연용 IP 대응)
    const host = window.location.hostname;
    const ws = new WebSocket(`ws://${host}:8000/ws/keywords`);

    ws.onopen = () => {
      console.log("✅ WebSocket connected");
    };

    ws.onmessage = (event) => {
      console.log("📨 received:", event.data);
      try {
        const data = JSON.parse(event.data);

        // 타입이 명시된 경우
        if (data.type === "keyword") {
          onKeyword && onKeyword(data);
        } else if (data.type === "voice_state") {
          // data.state: "waiting" | "listening"
          onVoiceState && onVoiceState(data.state);
        } else {
          // 타입 없으면 예전 포맷이라고 보고 키워드로 처리
          onKeyword && onKeyword(data);
        }
      } catch (e) {
        console.error("❌ failed to parse", e);
      }
    };

    ws.onclose = () => {
      console.log("❌ WebSocket disconnected");
    };

    return () => {
      ws.close();
    };
  }, [onKeyword, onVoiceState]);

  // 화면에 아무것도 안 그리는 “보이지 않는” 컴포넌트
  return null;
};