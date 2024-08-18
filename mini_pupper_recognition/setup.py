from setuptools import find_packages, setup
from glob import glob

package_name = 'mini_pupper_recognition'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='This package does line following with gen-ai image recognition.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_line_recognition_node = mini_pupper_recognition.ai_line_recognition_node:main',
            'movement_node = mini_pupper_recognition.movement_node:main',
            'ai_face_recognition_node = ai_face_recognition_node.ai_face_recognition_node:main',
            'music_dance_node = music_dance_node.music_dance_node:main'
        ],
    },
)
