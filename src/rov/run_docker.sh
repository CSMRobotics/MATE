#! /bin/bash
#xhost +
docker run --runtime nvidia -it --rm -v /bin/systemctl:/bin/systemctl -v /run/systemd/system:/run/systemd/system -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /sys/fs/cgroup:/sys/fs/cgroup -v /usr/lib/gstreamer-1.0/:/usr/lib/gstreamer-1.0 --net=host --pid=host -v /dev:/dev --privileged --gpus all -e LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so:/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvcompositor.so -e DISPLAY=$DISPLAY -e LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 --entrypoint /bin/bash csmmaterobotics/rov:latest
# docker run -it --rm --runtime nvidia -v /tmp/.X11-unix/:/tmp/.X11-unix csmmaterobotics/rov:latest
