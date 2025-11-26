import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv

# from llm_node_pkg.srv import TriggerLLM
from std_srvs.srv import Trigger

from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("llm_for_pick_place_voice")
# env_path = "/home/hun/ws/llm_ws/src/llm_for_pick_place_voice"
is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

class TriggerLLMServer(Node):
    def __init__(self):
        # llm_trigger_server 노드 생성
        super().__init__('llm_trigger_server')
        self.srv = self.create_service(Trigger, 'llm_trigger', self.handle_request)
        # 서비스 준비 완료 알림
        self.get_logger().info("서비스 서버 준비됨")


        # LangChain LLM 모델 초기화
        self.llm = ChatOpenAI(
            model="gpt-4o",  # 원하는 모델
            temperature=0.2, # 다음 단어 선택을 위한 확률(0일 경우 확률 높은 단어)
            openai_api_key=openai_api_key
        )
        # 📌 메시지 템플릿 생성
        prompt_template = """
        당신은 뛰어난 키워드 추출기입니다.

        <사용자 입력>
        "{user_input}"
        """

        self.prompt = PromptTemplate(
            input_variables=["user_input"], template=prompt_template
        )

        self.chain = self.prompt | self.llm  # 최신 구조

    def handle_request(self, request, response):
        # 서버가 직접 입력 받음!
        user_text = input("\n👉 질문을 입력하세요: ")

        # LLM 키워드 추출
        ai_msg = self.chain.invoke({"user_input": user_text})
        llm_out = ai_msg.content


        # object / target 나누기
        try:
            obj, tgt = llm_out.strip().split("/")
        except:
            obj, tgt = llm_out, ""

        obj_list = obj.split()

        response.success = True
        response.message = " ".join(obj_list)

        print(f"LLM 결과: {llm_out}")
        print(f"object: {obj_list}")

        return response
    # # 
    # def handle_request(self, request, response):
    #     user_input = request.prompt
    #     self.get_logger().info(f"[LLM] Received prompt: {user_input}")

    #     chain = self.prompt_template | self.llm

    #     llm_result = chain.invoke({"user_prompt": user_input})

    #     response.answer = llm_result.content
    #     return response


# trigger_server.py 실행
def main(args=None):
    rclpy.init(args=args)
    node = TriggerLLMServer()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()