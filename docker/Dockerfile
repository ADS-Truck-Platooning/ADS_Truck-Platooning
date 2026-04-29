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

# 파이썬 패키지 설치
RUN pip install --upgrade pip && \
    pip3 install --upgrade pip && \
    pip install --no-cache-dir setuptools && \
    pip3 install --no-cache-dir -Iv setuptools==47.3.1 && \
    pip install --no-cache-dir distro && \
    pip3 install --no-cache-dir distro && \
    pip install --no-cache-dir wheel && \
    pip3 install --no-cache-dir wheel auditwheel && \
    pip3 install --no-cache-dir pygame && \
    pip3 install typing-extensions && \
    pip3 install --no-cache-dir setuptools==58.3.0

# ROS 2 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# ros-humble-pcl-ros 설치
RUN apt-get update && apt-get install -y ros-humble-pcl-ros

# 워크스페이스 생성
WORKDIR /ADS_Truck-Platooning

# 기본 실행 셸
CMD ["bash"]
