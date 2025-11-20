import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate


############## 환경 설정 ##############
package_path = "/home/up/cam_ws/src/llm_pkg/resource/.env"
load_dotenv(package_path)
openai_api_key = os.getenv("OPENAI_API_KEY")


############## 서버 노드 ##############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        # LLM 준비
        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.3,
            openai_api_key=openai_api_key,
        )

        prompt_template = """
        당신은 뛰어난 키워드 추출기입니다.

        <사용자 입력>
        "{user_input}"
        """

        self.prompt = PromptTemplate(
            input_variables=["user_input"], template=prompt_template
        )

        self.chain = self.prompt | self.llm  # 최신 구조


        # Trigger 서비스 생성
        self.service = self.create_service(
            Trigger, "llm_trigger", self.process_trigger
        )

        self.get_logger().info("LLM Trigger 서비스 준비 완료.")

    def process_trigger(self, request, response):
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


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
