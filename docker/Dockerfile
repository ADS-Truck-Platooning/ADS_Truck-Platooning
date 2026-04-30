# ROS 2 humble이 설치된 공식 이미지 사용
FROM osrf/ros:humble-desktop

# 필요한 패키지 및 LLVM 저장소 추가
RUN apt-get update && \
    apt-get install -y wget gnupg software-properties-common lsb-release && \
    wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    add-apt-repository "deb http://archive.ubuntu.com/ubuntu focal main universe" && \
    apt-get update && \
    apt-get install -y \
        build-essential \
        clang-10 \
        lld-10 \
        g++-7 \
        cmake \
        ninja-build \
        libvulkan1 \
        python \
        python-dev \
        python3-dev \
        python3-pip \
        libpng-dev \
        libtiff5-dev \
        libjpeg-dev \
        tzdata \
        sed \
        curl \
        unzip \
        autoconf \
        libtool \
        rsync \
        libxml2-dev \
        git && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180

# Python 3.8 및 관련 라이브러리 설치 (deadsnakes PPA로 Ubuntu 22.04에서 3.8 지원)
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
        python3.8 \
        python3.8-dev \
        python3.8-distutils \
        python3.8-venv \
        python3-pip && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 && \
    curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py | python3.8

# 파이썬 패키지 설치
RUN pip3.8 install --no-cache-dir --upgrade --ignore-installed \
        setuptools==47.3.1 \
        wheel \
        distro \
        pygame \
        typing-extensions \
        numpy

# ROS 2 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# ros-humble-pcl-ros 설치
RUN apt-get update && apt-get install -y ros-humble-pcl-ros

# entrypoint 스크립트 복사
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 워크스페이스 생성
WORKDIR /ADS_Truck-Platooning/platoon_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
