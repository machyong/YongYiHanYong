# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import json
from rclpy.node import Node
import pyaudio

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
# from langchain_core.runnables import RunnablePassthrough
# from langchain_classic.chains import LLMChain

from std_srvs.srv import Trigger
# 자체 서비스 인터페이스 생성 모듈
# from llm_node_pkg.srv import TriggerLLM
from llm_for_pick_place_voice.MicController import MicController, MicConfig
from llm_for_pick_place_voice.wakeup_word import WakeupWord
from llm_for_pick_place_voice.stt import STT

from std_msgs.msg import String


############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("llm_for_pick_place_voice")
is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")
# openai_api_key 경로
# env_path = "/home/hun/ws/llm_ws/src/llm_for_pick_place_voice"


############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ get_keyword_server (서버) Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_server")
        # openai_api_key 로드 확인
        if not is_load:
            self.get_logger().error("Failed to load openai_api_key from .env file.")
        else:
            self.get_logger().info("openai_api_key loaded successfully.")

        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("Wait for get_keyword_client's request...")

        # LangChain LLM 모델 초기화
        self.llm = ChatOpenAI(
            model="gpt-4o",  # 원하는 모델
            temperature=0.2, # 다음 단어 선택을 위한 확률(0일 경우 확률 높은 단어)
            openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 문장에서 특정 테이블 번호와 각 테이블의 작업(action)을 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 테이블 번호를 최대한 정확히 추출하세요.
            - 각 테이블 번호와 함께 해당 테이블에 요청한 작업(action: clean 또는 setting)을 매핑하세요.

            <테이블 번호 리스트>
            - table1 table2 table3 table4
            
            <작업(action) 리스트>
            - clean, setting

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [table1:action1 table2:action2 ... / primary_action]
            - 각 테이블 번호 뒤에 콜론(:)과 그 테이블의 작업을 지정합니다.
            - 테이블들은 공백으로 구분합니다.
            - 마지막에 '/' 뒤에는 주요 작업(가장 먼저 언급된 작업)을 입력합니다.
            - 테이블 번호가 없으면 앞쪽은 공백 없이 비우고, 작업이 명확하지 않으면 'NONE'으로 출력합니다.
            - 테이블 번호의 순서는 문장에 등장한 순서를 따릅니다.
            - 각 테이블마다 명확한 작업이 없으면 그 테이블은 'NONE'으로 표시합니다.
            - 답은 테이블 번호 리스트 내에서만 추출하세요.
            - "출력"이라는 단어를 포함하지 마세요.

            <특수 규칙>
            - "몇 번 테이블", "3번", "3번 자리" 등 숫자 기반 표현을 table3과 같이 리스트 항목으로 변환하세요.
            - "치워줘, 정리해줘, 비워줘, 채워줘, 재워줘" → clean
            - "세팅해줘, 준비해줘, 셋팅해줘, 세트해줘" → setting
            - 여러 테이블 번호가 동시에 등장할 경우 각각에 맞는 작업을 지정하세요.
            - "1번 세팅하고 2번 정리해줘" 같은 문장에서는 각 테이블마다 다른 작업을 지정합니다.
            - "1번이랑 2번 테이블 정리" 같은 경우 두 테이블 모두 같은 작업입니다.
            - 작업이 명확하지 않은 특정 테이블은 NONE으로 표시합니다.

            <예시>
            - 입력: "3번 테이블 좀 치워줘"
            출력: table3:clean / clean

            - 입력: "1번이랑 3번 테이블 세팅해줘"
            출력: table1:setting table3:setting / setting

            - 입력: "1번 세팅하고 2번 테이블 정리해줘"
            출력: table1:setting table2:clean / setting

            - 입력: "저기 2번 정리 좀 부탁해"
            출력: table2:clean / clean

            - 입력: "4번 테이블 준비해놔"
            출력: table4:setting / setting

            - 입력: "2번 좀 봐줘 3번은 세팅해줘"
            출력: table2:NONE table3:setting / setting

            - 입력: "그냥 테이블 좀 보자"
            출력: / NONE

            <사용자 입력>
            "{user_input}"                   
        """
        # user_input 받아서 프롬프트 템플릿 생성
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template=prompt_content
        )

        # LangChain을 이용한 LLM model과 Prompt Template 연결
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)


        # 서버 서비스 생성
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword_callback
        )
        # wakeupword 탐지
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        self.keyword_pub = self.create_publisher(String, "keyword_topic", 10)
        self.voice_state_pub = self.create_publisher(String, "voice_state", 10)

    ### 중요 키워드(table num, action) 추출
    def extract_keyword(self, output_message):
        llm_response = self.lang_chain.invoke({"user_input": output_message})
        result = llm_response.content.strip()
        
        tables = []
        actions = []

        if result:
            table_action_pairs = result.split()
            for pair in table_action_pairs:
                if ":" in pair:
                    table, action = pair.split(":")
                    tables.append(table)
                    actions.append(action)
        
        # 디버깅 출력
        print(f"llm's response: {result}")
        print(f"object: {tables}")
        print(f"action: {actions}")

        return {
            "tables": tables,
            "actions": actions
        }
    

    ### 서비스 동작 콜백 (request, response 필요)
    def get_keyword_callback(self, req, res):
        # audio stream 열기
        try:
            # print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("Please check your device index")
            return None

        # 서비스 처리
        try:
            # wakeup word 대기
            self.publish_voice_state("waiting")
            self.get_logger().info("Waiting for wakeupword...")
            while not self.wakeup_word.is_wakeup():
                pass
            self.get_logger().info("Wakeupword detected!")

            # STT --> Keword Extract --> Embedding
            self.publish_voice_state("listening")
            output_message = self.stt.speech2text()
            self.publish_voice_state("waiting")
            keyword = self.extract_keyword(output_message)


            # object나 action이 None인 경우 경고
            if keyword:
                tables = keyword.get("tables", [])
                actions = keyword.get("actions", [])
            else:
                tables = []
                actions = []
                
            if not tables or tables == []:
                self.get_logger().warn("Warning: No tables detected (object is empty)")
            if not actions or all(a == "NONE" for a in actions):
                self.get_logger().warn("Warning: Action is NONE (not cleanly specified)")

            # 서비스 보내기
            res.success = True
            if tables and actions:
                table_action_pairs = " ".join([f"{t}:{a}" for t, a in zip(tables, actions)])
                res.message = table_action_pairs
                objects_str = " ".join(tables)
                actions_str = " ".join(actions)
            else:
                res.message = ""
                objects_str = "NONE"
                actions_str = "NONE"

            payload = {

                "type": "keyword",

                "tables": tables,            # 예: ['1번']

                "command": actions[0] if actions else "",# 예: 'clean'

                "text": output_message,                # STT 전체 텍스트

            }

            msg = String()

            msg.data = json.dumps(payload, ensure_ascii=False)

            self.keyword_pub.publish(msg)

            self.get_logger().info(f"Published keyword_topic: {msg.data}")
            
            self.get_logger().info(
                f"[Service response] success={res.success}, object={objects_str}, action={actions_str}"
            )
        except Exception as e:
            self.get_logger().error(f"Error: Failed processing: {e}")
            res.success = False
            res.message = f"Error: {e}"
        finally:
            # 오디오 stream 닫기
            self.mic_controller.close_stream()
        
        return res
    
    def publish_voice_state(self, state: str):

        msg = String()

        msg.data = state  # "wakeup_waiting", "listening"

        self.voice_state_pub.publish(msg)

        self.get_logger().info(f"[VOICE_STATE] {state}")

    


### get_keyword.py 실행
def main(args=None):
    rclpy.init(args=args)
    node = GetKeyword()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()