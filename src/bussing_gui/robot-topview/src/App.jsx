import { useState, useRef, useEffect} from "react";
import "./App.css";
import { KeywordListener } from "./KeywordListener"; 
import bussingMate from "./assets/bussing_mate.png";
import returnIcon from "./assets/return.png";
import servingIcon from "./assets/serving.png";
import kitchenIcon from "./assets/kitchen.png";
import counterIcon from "./assets/counter.png";
import tableImage from "./assets/table.png"; 
import restaurantBg from "./assets/restaurant.png";

const TABLE_POSITIONS = {
  0: { x: 343, y: 150 },
  1: { x: 745, y: 485 },
  2: { x: 1335, y: 485 },
  3: { x: 745, y: 870 },
  4: { x: 1335, y: 870 },
};

const WAYPOINTS = {
  T1: { x: 548,  y: 485 },
  T2: { x: 1040, y: 485 },
  T3: { x: 548,  y: 870 },
  T4: { x: 1040, y: 870 },
};

const PATH_MAP = {
  0: {
    0:[],
    1:["T1", 1],
    2:["T1", 1, "T2", 2],
    3:["T3", 3],
    4:["T1", 1, "T2", "T4", 4]
  },

  1: {
    0:["T1", 0],
    1:[],
    2:["T2", 2],
    3:["T1", "T3", 3],
    4:["T2", "T4", 4]
  },

  2: {
    0:["T2", 1, "T1", 0],
    1:["T2", 1],
    2:[],
    3:["T2", "T4", 3],
    4:["T2", "T4", 4]
  },

  3:{
    0:["T3", 0],
    1:["T3", "T1", 1],
    2:["T4", "T2", 2],
    3:[],
    4:["T4", 4]
  },

  4:{
    0:["T4", 3, "T3", 0],
    1:["T4", "T2", 1],
    2:["T4", "T2", 2],
    3:["T4", 3],
    4:[]
  }
};

const ALLOWED_COMMANDS = ["clean", "setting", "return"];

const lerp = (a, b, t) => a + (b - a) * t;

const animateTo = (start, end, duration, onUpdate, onDone) => {
  const startTime = performance.now();

  const step = (now) => {
    const elapsed = now - startTime;
    const t = Math.min(elapsed / duration, 1);

    const x = lerp(start.x, end.x, t);
    const y = lerp(start.y, end.y, t);

    onUpdate({ x, y });

    if (t < 1) requestAnimationFrame(step);
    else onDone && onDone();
  };

  requestAnimationFrame(step);
};

function App() {
  const [robotPos, setRobotPos] = useState(TABLE_POSITIONS[0]);
  const [target, setTarget] = useState(0);
  const [status, setStatus] = useState("Idle");
  const [lastCmd, setLastCmd] = useState("-");

  // 🔵 신규: 마이크 상태 + STT/키워드 표시용 상태
  const [micState, setMicState] = useState("waiting");       // "waiting" | "listening"
  const [transcript, setTranscript] = useState("");          // STT 전체 텍스트
  const [keywordTables, setKeywordTables] = useState([]);    // 추출된 테이블 배열
  const [keywordCommand, setKeywordCommand] = useState("");  // 추출된 명령

  const wsRef = useRef(null);

  /* -------------- robot_events WebSocket 연결 -------------- */
  useEffect(() => {
    const host = window.location.hostname;
    wsRef.current = new WebSocket(`ws://${host}:8000/ws/robot_events`);

    wsRef.current.onopen = () => console.log("🤖 robot_events connected");
    wsRef.current.onclose = () => console.log("🤖 robot_events disconnected");

    return () => wsRef.current && wsRef.current.close();
  }, []);


  /* -------------- 도착 메시지 보내는 함수 -------------- */
  const notifyArrived = (dest) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.warn("robot_events WebSocket not ready");
      return;
    }

    wsRef.current.send(
      JSON.stringify({
        type: "robot_event",
        event: "arrived",
        table: dest,
      })
    );

    console.log(`📨 Sent: arrived at table ${dest}`);
  };


  const displayTarget = (value) => {
    return value === 0 ? "퇴식구" : `${value}번 테이블`;
  };

  /* ---------- KeywordListener에서 온 payload 처리 ---------- */
const handleKeyword = (payload) => {
  console.log("Keyword from ROS:", payload);

  const { tables, command, text } = payload || {};

  // 1) STT 전체 문장은 항상 표시
  if (typeof text === "string") {
    setTranscript(text);
  }

  // 2) 테이블/명령 정보가 아예 없으면 그냥 UI만 업데이트하고 종료
  if (!Array.isArray(tables) || tables.length === 0) {
    console.warn("LLM에서 테이블 정보가 안 왔음:", payload);
    setKeywordTables([]);
    setKeywordCommand("");
    setLastCmd("인식된 테이블/명령 없음");
    return;
  }

  // 3) 테이블 문자열들에서 숫자만 뽑아서 0~4 범위만 남기기
  const parsedTableNums = tables
    .map((t) => {
      const num = parseInt(String(t).replace(/\D/g, ""), 10);
      return Number.isNaN(num) ? null : num;
    })
    .filter((num) => num !== null && num >= 0 && num <= 4);

  // 화면에 보여줄 용도로 "n번" 형태로 다시 만들기
  const displayTables = parsedTableNums.map((n) => `${n}번`);

  // 4) command 유효성 체크 (clean / setting 만 허용)
  const normalizedCommand =
    typeof command === "string" ? command.trim().toLowerCase() : "";
  const isValidCommand = ALLOWED_COMMANDS.includes(normalizedCommand);

  // 5) 하나라도 이상하면 UI에 "-" 찍고 로봇은 안 움직임
  if (displayTables.length === 0 || !isValidCommand) {
    console.warn("유효한 테이블/명령을 찾지 못함:", { tables, command });

    setKeywordTables([]);   // → info-panel 에서 "-"로 보임
    setKeywordCommand("");  // → info-panel 에서 "-"로 보임
    setLastCmd("인식된 테이블/명령 없음");
    return;
  }

  // 6) 여기까지 왔으면 유효한 입력
  setKeywordTables(displayTables);
  setKeywordCommand(normalizedCommand);
  setLastCmd(`${displayTables[0]} / ${normalizedCommand}`);

  // 첫 번째 테이블 번호로 이동
  const dest = parsedTableNums[0];
  if (dest >= 0 && dest <= 4) {
    go(dest);
  }
};

  const handleVoiceState = (state) => {
    console.log("Voice state:", state);
    setMicState(state);   // "waiting" 또는 "listening"
  };


  /* ---------------------- go() 이동 시스템 ---------------------- */
  const go = (dest) => {
    if (status === "Moving") return;

    const startId = target;
    const pathKeys = PATH_MAP[startId][dest];
    if (!pathKeys) return;

    setTarget(dest);
    setStatus("Moving");

    const points = pathKeys.map(k =>
      typeof k === "string" ? WAYPOINTS[k] : TABLE_POSITIONS[k]
    );

    let current = robotPos;
    const SPEED = 100;

    const moveNext = (i) => {
      if (i >= points.length) {
        setStatus("Idle");
        notifyArrived(dest);  // 🔥 도착 이벤트 전송
        return;
      }

      const nextPoint = points[i];
      const dx = nextPoint.x - current.x;
      const dy = nextPoint.y - current.y;
      const dist = Math.hypot(dx, dy);

      const duration = (dist / SPEED) * 1000;

      animateTo(current, nextPoint, duration, setRobotPos, () => {
        current = nextPoint;
        moveNext(i + 1);
      });
    };

    moveNext(0);
  };

  // useEffect(() => {
  //   window.go = (dest) => go(dest);
  //   window.testCmd = (table, cmd = "clean") =>
  //     handleKeyword({ tables: [String(table)], command: cmd, text: "test" });

  //   console.log(
  //     "%c[DEBUG] window.go(table), window.testCmd(table, cmd) 사용 가능",
  //     "color: #22c55e; font-weight: bold;"
  //   );

  //   return () => {
  //     delete window.go;
  //     delete window.testCmd;
  //   };
  // }, [go, handleKeyword]);

  return (
    <div className="map-container"
    //  style={{ backgroundImage: `url(${restaurantBg})` }}
    >
      {/* 🔵 WebSocket → onKeyword + onVoiceState 연결 */}
      <KeywordListener
        onKeyword={handleKeyword}
        onVoiceState={handleVoiceState}
      />

      {/* 상단 TopView 영역 */}
      <div className="topview">
        <div className="top-badge return">
          <img src={returnIcon} alt="퇴식구 아이콘" className="top-icon" />
          <span>퇴식구</span>
        </div>

        <div className="top-badge serving">
          <img src={servingIcon} alt="배식구 아이콘" className="top-icon" />
          <span>배식구</span>
        </div>

        <div className="top-badge kitchen">
          <img src={kitchenIcon} alt="주방 아이콘" className="top-icon" />
          <span>주방</span>
        </div>

        <div className="top-badge counter">
          <img src={counterIcon} alt="카운터 아이콘" className="top-icon" />
          <span>카운터</span>
        </div>

        {/* 로봇 점 */}
        <div
          className="robot-dot"
          style={{
            transform: `translate(${robotPos.x - 65}px, ${robotPos.y - 65}px)`,
            backgroundImage: `url(${bussingMate})`,
          }}
        />

        {/* 테이블 2×2 */}
        <div className="tables">
          {[1, 2, 3, 4].map((id) => (
            <div className="table-block" key={id}>
              <div className="table-title">테이블 {id}</div>

              <div className="table-rect">
                <img
                  src={tableImage}  // 혹은 동일 이미지
                  alt={`${id}번 테이블`}
                  className="table-img"
                />
              </div>

            </div>
          ))}
        </div>

               {/* 왼쪽 패널 모음 */}
        <div className="control-overlay">
          {/* 1. 제일 위: 큰 숫자/목표 패널 */}
          <div className="highlight-panel">{displayTarget(target)}</div>

          {/* 2. 마이크 상태 패널 (투명 배경, 중앙 정렬) */}
          <div className="mic-panel">
            <div className="mic-label">
              {micState === "listening" ? "말씀 중입니다..." : "웨이크업 워드 대기 중"}
            </div>

            {micState === "listening" ? (
              <div className="wave-container">
                {Array.from({ length: 16 }).map((_, i) => (
                  <div
                    key={i}
                    className="wave-bar"
                    style={{ animationDelay: `${i * 60}ms` }}
                  />
                ))}
              </div>
            ) : (
              <div className="mic-circle">
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  className="mic-icon"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="#1f6fd8"
                  strokeWidth="1.8"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <path d="M12 1a3 3 0 0 0-3 3v7a3 3 0 0 0 6 0V4a3 3 0 0 0-3-3z" />
                  <path d="M19 10a7 7 0 0 1-14 0" />
                  <line x1="12" y1="17" x2="12" y2="23" />
                  <line x1="8" y1="23" x2="16" y2="23" />
                </svg>
              </div>
            )}
          </div>

          {/* 3. STT를 크게 보여주는 패널 */}
          <div className="stt-panel">
            <div className="stt-title">음성 인식 결과</div>
            <div className="stt-main">
              {transcript || (
                <span className="placeholder">아직 인식된 문장이 없습니다.</span>
              )}
            </div>
          </div>

          {/* 4. 테이블 / 작업 정보 패널 */}
          <div className="info-panel">
            <div className="info-line">
              테이블:&nbsp;
              {keywordTables && keywordTables.length > 0
                ? keywordTables.join(", ")
                : "-"}
            </div>
            <div className="info-line">
              작업:&nbsp;{keywordCommand || "-"}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;
