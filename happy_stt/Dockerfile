FROM ros:humble

# --- 必要な Linux パッケージ ---
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    portaudio19-dev \
    libsndfile1 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# --- torch/torchaudio をローカルの .whl からインストール ---
COPY wheels /tmp/wheels
RUN pip3 install --no-index --find-links=/tmp/wheels torch torchaudio

# --- 他のPython依存パッケージ ---
COPY requirements.txt /tmp/
RUN pip3 install --default-timeout=100 --retries=10 --no-cache-dir -r /tmp/requirements.txt

# --- ROSワークスペース ---
WORKDIR /workspace
COPY models/silero_vad.pt /workspace/models/silero_vad.pt
COPY models/distil-whisper /workspace/models/distil-whisper
COPY . .

RUN . /opt/ros/humble/setup.sh && \
    colcon build

# --- 環境設定 ---
ENV ROS_DOMAIN_ID=0
SHELL ["/bin/bash", "-c"]
CMD ["bash"]

# --- 起動時の自動source ---
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/hm_msgs_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'cd /workspace' >> ~/.bashrc

# --- FastRTPSプロファイル設定 ---
COPY fastrtps_profile.xml /workspace/fastrtps_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fastrtps_profile.xml
