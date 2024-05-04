#! /bin/bash
xhost +
docker run --runtime nvidia -it --rm \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    --pid=host \
    --ipc=host \
    --net=host \
    --gpus all \
    --privileged \
    -v /dev/shm:/dev/shm \
    -v /bin/systemctl:/bin/systemctl \
    -v /run/systemd/system:/run/systemd/system \
    -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
    -v /sys:/sys:ro \
    -v /usr/lib/gstreamer-1.0/:/usr/lib/gstreamer-1.0 \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority:rw \
    -v /tmp/argus_socket:/tmp/argus_socket \
    --cap-add SYS_PTRACE \
    -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so:/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so \
    -e DISPLAY=$DISPLAY \
    -e LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 \
    csmmaterobotics/rov:latest
# docker run -it --rm --runtime nvidia -v /tmp/.X11-unix/:/tmp/.X11-unix csmmaterobotics/rov:latest
