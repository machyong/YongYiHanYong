# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import json
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain_core.runnables import RunnablePassthrough

from std_srvs.srv import Trigger
from .MicController import MicController, MicConfig

from .wakeup_word import WakeupWord
from .stt import STT

from std_msgs.msg import String

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice_new")
# env_path = "/home/hun/ws/llm_ws/src/llm_for_pick_place_voice"
is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 문장에서 특정 테이블 번호와 사용자의 명령을 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 번호를 정확히 추출하시오.
            - 문장에 등장하는 테이블 번호와 명령(어디를 치우라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - 0번 1번 2번 3번 4번 clean setting

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [테이블 번호] / [명령]
            - 명령과 위치는 각각 공백으로 구분
            - 답은 도구 리스트에서만 추출해줘

            <특수 규칙>
            - 명확한 명령이 없지만 문맥상 유추 가능한 경우(예: "정리" → clean)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "1번 테이블 치워줘"  
            출력: 1번 / clean

            - 입력: "2번이랑 4번 세팅해줘"  
            출력: 2번 4번 / setting

            - 입력: "3번 정리해줘"  
            출력: 3번 / clean

            <사용자 입력>
            "{user_input}"                   
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)

        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        self.keyword_pub = self.create_publisher(String, "keyword_topic", 10)
        self.voice_state_pub = self.create_publisher(String, "voice_state", 10)
        

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content if hasattr(response, 'content') else str(response)

        extracted_object, target = result.strip().split("/")

        print(f"[TEST] input message: {output_message}")

        extracted_object = extracted_object.split()
        target = target.split()

        print(f"llm's response: {result}")
        print(f"object: {extracted_object}")
        print(f"target: {target}")
        return extracted_object, target
    
    def get_keyword(self, request, response):
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            response.success = False
            response.message = "Failed to open audio stream"
            return response

        try:
            # 웨이크업 워드 대기
            self.publish_voice_state("waiting")
            self.get_logger().info("Waiting for wakeup word...")
            while not self.wakeup_word.is_wakeup():
                pass
            self.get_logger().info("Wakeup word detected!")

            # STT --> Keyword Extract
            self.publish_voice_state("listening")
            self.get_logger().info("Recording speech...")
            output_message = self.stt.speech2text()

            self.publish_voice_state("waiting")

            extracted_object, target = self.extract_keyword(output_message)

            self.get_logger().warn(f"Detected keywords: {extracted_object}, {target}")

            payload = {
                "type": "keyword",
                "tables": extracted_object,            # 예: ['1번']
                "command": target[0] if target else "",# 예: 'clean'
                "text": output_message,                # STT 전체 텍스트
            }

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.keyword_pub.publish(msg)
            self.get_logger().info(f"Published keyword_topic: {msg.data}")


            # 응답 객체 설정
            response.success = True
            response.message = " ".join(extracted_object + target)
            
            self.get_logger().info(f"Service response sent: {response.message}")
            
        except Exception as e:
            self.get_logger().error(f"Error during processing: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            self.publish_voice_state("waiting")
        finally:
            self.mic_controller.close_stream()
            self.get_logger().info("Stream closed. Ready for next request.")
        
        return response
    
    def publish_voice_state(self, state: str):
        msg = String()
        msg.data = state  # "wakeup_waiting", "listening"
        self.voice_state_pub.publish(msg)
        self.get_logger().info(f"[VOICE_STATE] {state}")



def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
