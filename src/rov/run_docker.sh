#! /bin/bash
xhost +
docker run --runtime nvidia -it --rm \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    --pid=host \
    --ipc=host \
    --gpus all \
    -v /dev/shm:/dev/shm \
    -v /bin/systemctl:/bin/systemctl \
    -v /run/systemd/system:/run/systemd/system \
    -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
    -v /sys:/sys:ro \
    -v /usr/lib/gstreamer-1.0/:/usr/lib/gstreamer-1.0 \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so:/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so \
    -e DISPLAY=$DISPLAY \
    -e LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 \
    csmmaterobotics/competition:latest
# docker run -it --rm --runtime nvidia -v /tmp/.X11-unix/:/tmp/.X11-unix csmmaterobotics/rov:latest
