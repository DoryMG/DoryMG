
https://saint-swithins-day.tistory.com/53?category=869581

https://m.blog.naver.com/icbanq/221082121182

raspicam_node

Raspberry Pi 카메라 모듈용 ROS 노드. 모듈의 V1.x 및 V2.x 버전 모두에서 작동합니다. v2.x 카메라는 자동 게인이 더 좋고 일반적인 이미지 품질이 더 좋으므로 사용을 권장합니다.
설치

바이너리는 에서 찾을 수 있습니다 https://packages.ubiquityrobotics.com/ . 지침에 따라 저장소를 추가하십시오.

그런 다음 실행 sudo apt install ros-kinetic-raspicam-node
빌드 지침

바이너리를 사용하는 대신 소스에서 빌드하려면 이 섹션을 따르세요.

이 노드는 주로 ROS Kinetic 및 Ubuntu 16.04에서 지원되며, 이것이 이러한 지침에서 가정하는 것입니다.

catkin_ws로 이동 cd ~/catkin_ws/src.

다음을 실행하여 이 노드의 소스를 다운로드하십시오.

git clone https://github.com/UbiquityRobotics/raspicam_node.git

ros에서 인식하지 못하는 종속성이 있으므로 파일을 생성해야 합니다. /etc/ros/rosdep/sources.list.d/30-ubiquity.list그리고 이것을 추가하십시오.

yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml

그런 다음 실행 rosdep update.

ros 종속성을 설치하고,

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

다음을 사용하여 코드를 컴파일하십시오. catkin_make.
노드 실행

노드가 구축되면 시작 파일을 사용하여 실행할 수 있습니다.

V2.x 카메라의 경우 다음을 실행합니다. roslaunch raspicam_node camerav2_1280x960.launch

V1.x 카메라의 경우 다음을 실행합니다. roslaunch raspicam_node camerav1_1280x720.launch

사용하다 rqt_image_view연결된 컴퓨터에서 게시된 이미지를 봅니다.
동적 재구성으로 노드 구성

NS raspicam_node카메라 매개변수를 동적으로 재구성하는 것을 지원합니다.

연결된 컴퓨터에서 동적 재구성 노드를 실행합니다.

rosrun rqt_reconfigure rqt_reconfigure

아래와 같은 사용자 인터페이스가 나타나야 합니다. 매개변수는 이 인터페이스를 통해 동적으로 조정할 수 있습니다.

rqt_reconfigure
문제 해결

    사용자가 video실행하여 그룹화 groups|grep video.

    다음과 같은 오류가 발생하는 경우: Failed to create camera component, 카메라 케이블이 양쪽 끝에 제대로 연결되어 있고 케이블에 핀이 없는지 확인하십시오.

    네트워크를 통한 이미지의 게시 속도가 예상보다 낮으면 더 낮은 해상도를 사용하여 필요한 대역폭 양을 줄이는 것이 좋습니다.

Node Information

주제:

    ~/image/compressed: 출판하다 sensor_msgs/CompressedImage카메라 모듈에서 jpeg로.

    ~/image: 출판하다 sensor_msgs/Image카메라 모듈에서(매개변수인 경우 enable_raw설정됨).

    ~/motion_vectors: 출판하다 raspicam_node/MotionVectors카메라 모듈에서(매개변수인 경우 enable_imv설정됨).

    ~/camera_info: 출판하다 sensor_msgs/CameraInfo각 프레임에 대한 카메라 정보.

서비스:

    ~/set_camera_info: 카메라의 캘리브레이션 정보를 업데이트하기 위해 사용합니다.

매개변수:

    ~private_topics(bool): 기본적으로 주제는 비공개입니다. 즉, 모든 주제 이름 앞에 노드 이름이 추가됩니다. 주제를 비공개로 설정하지 않으려면 이 매개변수를 "true"로 설정할 수 있습니다. 이 매개변수는 주로 이전 버전과의 호환성을 유지하기 위해 존재합니다.

    ~camera_frame_id(tf frame): 카메라를 연결할 프레임 식별자입니다.

    ~camera_info_url: 카메라 보정 URL .yaml파일.

    ~camera_name(문자열): 카메라의 이름으로, camera_info 파일의 이름과 일치해야 합니다.

    ~framerate(fps): 캡처할 프레임 속도입니다. 최대 90fps

    ~height(픽셀): 이미지를 캡처할 높이입니다.

    ~width(픽셀): 이미지를 캡처할 너비입니다.

    ~quality(0-100): 캡처한 이미지의 품질입니다.

    ~enable_raw(bool): 원시 이미지 게시(더 많은 CPU 및 메모리 사용)

    ~enable_imv(bool): GPU에서 계산한 인라인 모션 벡터 게시

    ~camera_id(int): 카메라 ID(Compute Module에서만 지원됨)

구경 측정

raspicam_node 패키지에는 라즈베리에 대한 보정 파일이 포함되어 있습니다. PI 카메라 버전 1 및 2.

튜토리얼 단안 카메라 보정 튜토리얼 단일 카메라를 보정하는 방법을 보여줍니다.

NS 8x6 바둑판 그리고 7x6 바둑판 크기가 크고 인쇄하려면 특수 프린터가 필요합니다. 전체 규모. 더 일반적인 프린터 크기로 인쇄할 수 있습니다. 자동 스케일링이 켜진 상태에서 신중하게 측정하십시오 밀리미터 단위의 정사각형 크기를 1000으로 나누어 미터로 변환합니다.

보정을 실행하려면 원시 게시가 활성화되어 있어야 합니다. 추가하다 enable_raw:=true카메라 roslaunch 명령에.

어떤 시작 파일을 사용할지 확실하지 않은 경우 camerav2_1280x960_10fps.launch아마도 당신이 찾고있는 것입니다.

파이에서

roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true

워크스테이션에서:

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.074 image:=/raspicam_node/image camera:=/raspicam_node

모션 벡터

raspicam_node는 를 출력할 수 모션 벡터 Raspberry Pi의 하드웨어 비디오 인코더에서 계산된 있습니다. 이러한 움직임 벡터는 움직임 감지와 같은 다양한 응용 분야에 사용할 수 있습니다.

파이에 추가 enable_imv:=true카메라 roslaunch 명령:

roslaunch raspicam_node camerav2_410x308_30fps.launch enable_imv:=true

워크스테이션에서 raspicam_node를 빌드하여 MotionVectorsROS 메시지는 Python에서 인식됩니다.

cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

마지막으로 스크립트 실행 imv_view.py모션 벡터를 시각화하려면:

rosrun raspicam_node imv_view.py
