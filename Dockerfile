FROM nvidia/cuda:8.0-devel-ubuntu16.04

# Reference to
# https://github.com/ikester/blender-docker/blob/master/Dockerfile

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
        curl \
        bzip2 \
        libfreetype6 \
        libgl1-mesa-dev \
        libglu1-mesa \
        libxi6 \
        python3.7 \
        curl \
        tmux \
        libxrender1 && \
    apt-get -y autoremove && \
    rm -rf /var/lib/apt/lists/*

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3.7 get-pip.py && rm get-pip.py

ENV BLENDER_BZ2_URL https://builder.blender.org/download/blender-2.80-a38205fbd801-linux-glibc224-x86_64.tar.bz2

RUN mkdir /usr/local/blender && \
    curl -SL "$BLENDER_BZ2_URL" -o blender.tar.bz2 && \
    tar -jxvf blender.tar.bz2 -C /usr/local/blender --strip-components=1 && \
    rm blender.tar.bz2

RUN pip3 install numpy scipy attrs zmq
RUN cp -r /usr/local/lib/python3.7/dist-packages/attr* /usr/local/blender/2.80/python/lib/python3.7/site-packages && \
    cp -r /usr/local/lib/python3.7/dist-packages/zmq* /usr/local/blender/2.80/python/lib/python3.7/site-packages && \
    cp -r /usr/local/lib/python3.7/dist-packages/pyzmq* /usr/local/blender/2.80/python/lib/python3.7/site-packages


ENV BLENDER_PATH=/usr/local/blender/blender

ADD ./ /blender_server
ENV PYTHONPATH="/blender_server:${PYTHONPATH}"
ENTRYPOINT cd /blender_server && /bin/bash ./run_blender_server.sh
