#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import PIL
import cv2
import base64
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from langchain_google_vertexai import ChatVertexAI
from langchain_core.messages import HumanMessage
from io import BytesIO
from std_msgs.msg import String


def ai_pointer_response(llm, image, text):
    buffered = BytesIO()
    image.save(buffered, format="JPEG")
    image_bytes = buffered.getvalue()

    image_base64 = base64.b64encode(image_bytes).decode('utf-8')
    image_data_url = f"data:image/jpeg;base64,{image_base64}"

    image_message = {
        "type": "image_url",
        "image_url": {
            "url": image_data_url
        }
    }
    text_message = {"type": "text", "text": text}

    message = HumanMessage(content=[text_message, image_message])

    output = llm.invoke([message])
    result = output.content

    return result


class AiPointerResponse(Node):
    def __init__(self):
        super().__init__('ai_pointer_response')
        self.sub = self.create_subscription(Image, '/image_raw', self.pointer_recognition, 10)
        self.pointer_publisher_ = self.create_publisher(String, 'direction_of_pointer', 10)

    def pointer_recognition(self, msg):
        condition_input_prompt = """
        There is a pointer in the image, either pointing to the left or right.

        The output should follow this format:

        [condition]

        Where:
        - [condition] is either "no", "left", or "right" depending on the direction the pointer
        is pointing in the image.

        If no pointer is detected (empty image), output "no".
        If the pointer is pointing to the left, output "left".
        If the pointer is pointing to the right, output "right".

        Your output should accurately reflect the true direction the pointer is pointing in the
        image, and follow the specified format exactly.
        """

        multi_model = ChatVertexAI(model="gemini-pro-vision")
        self.bridge = CvBridge()
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = PIL.Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        condition_response = ai_pointer_response(multi_model, image=image, text=condition_input_prompt)
        self.get_logger().info(f"condition response: {condition_response}")

        message = String()
        condition = condition_response
        message.data = condition
        self.pointer_publisher_.publish(message)


def main():
    rclpy.init()
    minimal_service = AiPointerResponse()
    rclpy.spin(minimal_service)


if __name__ == '__main__':
    main()