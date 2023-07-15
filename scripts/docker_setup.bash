run-container() {

usage="usage: $FUNCNAME NAME IMAGE [SCRIPT]"

if [[ $# -lt 2 ]]; then
       echo "$usage" >&2
else
       NAME="$1"
       IMAGE="$2"
       SCRIPT="x-terminal-emulator"

       if [[ $# -gt 2 ]]; then
              SCRIPT="$3"
       fi

       docker run -it --rm \
       --env="DISPLAY" \
       --env="ROS_MASTER_URI" \
       --env="ROS_IP" \
       -v $HOME/.ssh:/home/user/.ssh \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v "$(pwd)"/scripts:/home/user/scripts \
       -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
       --network host \
       --privileged -v /dev:/dev \
       --name "$NAME" \
       "$IMAGE" \
       /bin/bash -ic "$SCRIPT"
fi
}

attach-bash-to-container() {

usage="usage: $FUNCNAME NAME"

if [[ $# -lt 1 ]]
then
       echo "$usage" >&2
else
       docker cp ~/.ssh "$1":/home/user && docker exec -it --env="DISPLAY" --privileged "$1" /bin/bash -c "source /home/user/.bashrc; x-terminal-emulator"
fi
}
