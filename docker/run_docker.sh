#!/bin/bash
DOCKER_TAG="ransaclib"
DOCKER_NAME="ransaclib"
DOCKER_ENV=""
read -r -a DOCKER_RUN_ARGS <<< "$DOCKER_ENV"

DOCKER_RUN_ARGS+=("--name" "${DOCKER_NAME}"
                  "--rm"
                  "--tty"
                  "--init"
                  "--interactive"
                  "--privileged"
                  "--net=host"
                  "--gpus" "all"
                  "--hostname" "$DOCKER_NAME"
                  "-e" "DISPLAY"
                  "-e" "HOME"
                  "-e" "LANG=C.UTF-8"
                  "-e" "TERM=xterm-256color"
                  "-v" "/tmp/.X11-unix:/tmp/.X11-unix"
                  "-v" "$HOME:$HOME"
                  "-w" "$PWD")

MATLAB_BASE_PATH="/usr/local/MATLAB"


if [ -d "${MATLAB_BASE_PATH}" ]; then
    MATLAB_VER=$( ls ${MATLAB_BASE_PATH} | tail -1 )
    MATLAB_PATH="${MATLAB_BASE_PATH}/${MATLAB_VER}"
    DOCKER_RUN_ARGS+=("-v" "${MATLAB_PATH}:${MATLAB_PATH}")
    PATH="${PATH}:${MATLAB_PATH}/bin"
fi


# The following is a hack that creates a linux group and user with the same
# uid:gid as the host. It then logs in this user so that all files created have
# the host user as owner and group.
PATH_ENV="$PATH"
CONTAINER_USERNAME=$(whoami)
CONTAINER_GROUPNAME=$(id -gn)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
read -r -d '' CREATE_USER_COMMAND << EOF || true
groupadd --force --gid "$GROUP_ID" "$CONTAINER_GROUPNAME" &&
useradd --no-create-home \
        --uid "$USER_ID" \
        --gid "$GROUP_ID" \
        --groups sudo \
        --home-dir "$HOME" \
        --shell /bin/bash \
        "$CONTAINER_USERNAME" &&
echo "root:root" | chpasswd &&
echo "$CONTAINER_USERNAME:$CONTAINER_USERNAME" | chpasswd &&
# Set PATH for both root and local user (since su does not preserve PATH)
echo PATH="$PATH_ENV" | tee /etc/profile /etc/environment &&
su --login --preserve-environment "$CONTAINER_USERNAME"
EOF

#If commands are given on the command line, then run them and exit, otherwise just start a terminal
if [ "$#" -gt 0 ]; then
    CREATE_USER_COMMAND="${CREATE_USER_COMMAND} -c \"$@\""
fi

docker run "${DOCKER_RUN_ARGS[@]}" "${DOCKER_TAG}" bash -ci "$CREATE_USER_COMMAND"
