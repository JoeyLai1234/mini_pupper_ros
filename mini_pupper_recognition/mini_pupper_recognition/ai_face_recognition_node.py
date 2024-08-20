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
import os
import yt_dlp
from googleapiclient.discovery import build
import google.auth
from dotenv import load_dotenv 


def extract_keyword_constant(input_string):
    input_string = input_string.strip()
    parts = input_string.split()

    condition = ""
    singer = ""
    dancer = ""
    name = ""

    for part in parts:
        if part.lower() in ['yes', 'no']:
            condition = part.lower()
        elif part.lower() in ['singer', 'not_singer']:
            singer = part.lower()
        elif part.lower() in ['dancer', 'not_dancer']:
            dancer = part
        else:
            name = part

    return condition, singer, dancer, name


def ai_face_response(llm, image, text):
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


def music_download(singer_name, credentials):
    output_dir = os.path.join(os.path.dirname(os.path.dirname(os.getcwd())), 'mini_pupper_music', 'audio')

    youtube = build('youtube', 'v3', credentials=credentials)
    request = youtube.search().list(q=singer_name, part='id,snippet', maxResults=1)
    response = request.execute()

    if 'items' in response and len(response['items']) > 0:
        item = response['items'][0]
        if 'id' in item and 'videoId' in item['id']:
            video_id = item['id']['videoId']
            video_url = f"https://www.youtube.com/watch?v={video_id}"

            output_file = os.path.join(output_dir, singer_name)
            ydl_opts = {
                'format': 'bestaudio/best',
                'outtmpl': output_file,
                'postprocessors': [{'key': 'FFmpegExtractAudio', 'preferredcodec': 'wav'}],
            }

            if os.path.exists(output_file):
                os.remove(output_file)

            with yt_dlp.YoutubeDL(ydl_opts) as ydl:
                ydl.download([video_url])

            return f"{output_file}.wav"
        else:
            print(f"'videoId' key not found in the API response for singer: {singer_name}")
    else:
        print(f"No items found in the API response for singer: {singer_name}")

    return None


def init_credentials(key_json_path):
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = key_json_path
    credentials, project_id = google.auth.default()
    return credentials, project_id


class AifaceResponse(Node):
    def __init__(self):
        super().__init__('ai_face_response')
        self.sub = self.create_subscription(Image, '/image_raw', self.face_recognition, 10)
        self.condition_publisher_ = self.create_publisher(String, 'condition_is_face', 10)
        self.singer_publisher_ = self.create_publisher(String, 'is_singer', 10)
        self.dancer_publisher_ = self.create_publisher(String, 'is_dancer', 10)
        self.song_publisher_ = self.create_publisher(String, 'song', 10)

        current_file_path = os.path.abspath(__file__)
        os.chdir(os.path.dirname(current_file_path))
        self.get_logger().info(f"init chdir: {os.path.dirname(current_file_path)}")

        load_dotenv(dotenv_path='./.env')

        cloud_config = {
            'api_key_path': os.environ.get('API_KEY_PATH', ''),
        }
        api_path = str(cloud_config['api_key_path'])
        if os.path.exists(api_path):
            self.get_logger().info("init credentials start.")
            init_credentials(api_path)
            self.get_logger().info("init credentials end.")
        else:
            self.get_logger().info("credentials file not exist.")

    def face_recognition(self, msg):

        condition_input_prompt = """
        You are an advanced computer vision and facial recognition AI assistant. Your task is to continuously analyze an
        image stream and detect human faces, determine if the detected faces belong to singers, and if the person is a dancer.
        The output should follow this strict format:

        [condition] [singer] [dancer] [full_name]

        Where:
        - [condition] is "yes" if a human face is detected, or "no" if no face is detected.
        - [singer] is "singer" if the detected face belongs to a singer, or "not_singer" if it does not.
        - [dancer] is "dancer" if the person is a dancer, or "not_dancer" if they are not.
        - [full_name] is the full name of the person, with an underscore between the first and last name, if it can be consistently and accurately identified across the image stream. If the person's identity cannot be reliably determined, use "unknown".

        The key challenge in this task is that the image stream may contain inconsistent or inaccurate face detections and identifications. Your model should strive to provide the most accurate and consistent output possible, but be prepared to output "unknown" for the full name if the identity cannot be reliably determined.

        Examples:
        - If a face of Beyoncé Knowles is detected consistently throughout the stream: "yes singer dancer Beyoncé_Knowles"
        - If the face detection is inconsistent and alternates between a singer and a non-singer: "yes singer not_dancer unknown"
        - If no face is detected in some frames: "no not_singer not_dancer unknown"

        You should output this information for each frame of the image stream, continuously, until the stream ends. Do not
        provide any other information or commentary. Simply output the information in the exact format specified above.
        """
        
        global credentials
        credentials, _ = google.auth.default()

        multi_model = ChatVertexAI(model="gemini-pro-vision")
        self.bridge = CvBridge()
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = PIL.Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        condition_response = extract_keyword_constant(
            ai_face_response(multi_model, image=image, text=condition_input_prompt)
        )
        self.get_logger().info(f"condition response: {condition_response}")

        singer_name = condition_response[3]

        if singer_name != '' and singer_name != 'unknown' :
            song = music_download(singer_name, credentials)

        message1 = String()
        condition = condition_response[0]
        message1.data = condition
        self.condition_publisher_.publish(message1)

        message2 = String()
        singer = condition_response[1]
        message2.data = singer
        self.singer_publisher_.publish(message2)

        message3 = String()
        dancer = condition_response[2]
        message3.data = dancer
        self.dancer_publisher_.publish(message3)

        message4 = String()
        song = singer_name
        message4.data = song
        self.song_publisher_.publish(message4)


def main():
    rclpy.init()
    minimal_service = AifaceResponse()
    rclpy.spin(minimal_service)


if __name__ == '__main__':
    main()