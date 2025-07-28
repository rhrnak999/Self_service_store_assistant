# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):


        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 담배의 종류를 같은 종류로 구분 해야합니다. 예를들어 팔라멘트,parliament, アクア, 아쿠아 등등이 parliament로 출력해줬으면해
            또한 갯수를 정확히 인식하여 추출해야합니다.

            <목표>
            - 문장에서 담배의 여러이름을 하나의 이름으로 정하여 반환 해주세요.
            - 문장에 등장하는 갯수를 추출하여 반환해 주세요.

            <담배 리스트>
            -  theone, parliament, africa, dunhill, bohem, halla, esse, raison, mevius, thisplus

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [담배1 담배2 ... / 갯수1 갯수2 ...]
            - 담배와 갯수는 공백으로 구분하세요.
            - 담배가 없으면 앞쪽은 공백 없이 비우고, 갯수 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 단 여러개의 담배를 구매 한다고하는데 각1개씩 이라던가 몇개만 누락하고 말한다면 누락된곳은 공백에,를 추가하여 순서에 문제 없게 해주세요.
            - 각 몇개 이런 식의로의 것은 예를 들어 팔라 아프리카 던힐 각 1개 그리고 보햄 두 갑 주세요 하면 parliament africa dunhill bohem/1 1 1 2 이렇게 출력해주세요.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 담배 명칭이 없지만 문맥상 유추 가능한 경우(예: "한라산" → halla)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 옛날 명칭을 사용하시는 분들도 있어요 감안해서 해당 브랜드를 연결 시켜반환 해주세요.
            - 줄여서 말하는 분들도 있어요 가령 마쎄 마세 -> 마일드 세븐 -> 메비우스 거든요 이걸 잘 처리 해주세요.
            - 다수의 담배가 등장 시 담배-갯수, 담배-갯수 순으로 사람이 말할태니 잘 구분해서 정확히 매칭하여 순서대로 출력해 주세요.
            - the one 과 this plus는 "the one" "this plus"로 하나로 인식 되어야 합니다
            - ['-', '입력:', '"디', '한갑"', '출력:', 'dunhill'] 이런식의 출력은 잘못되었습니다 담배 리스트에 있는것만 출력해 주세요

            <예시>
            - 입력: "더원 줘"  
            출력: theone / 

            - 입력: "아쿠아 한갑 줘"  
            출력: parliament / 1

            - 입력: "아프리카랑 던힐 각 두갑씩줘"  
            출력: afirca dunhill /2 2

            - 입력: "보햄 한보루줘"  
            출력: hammer /10

            <사용자 입력>
            "{user_input}"               
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
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
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
