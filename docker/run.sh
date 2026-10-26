#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${DOCKER_IMAGE:-outdoor_nav_humble}"
command -v docker >/dev/null || { echo '请先安装 Docker Engine。' >&2; exit 1; }
docker info >/dev/null || { echo 'Docker 不可用；请检查服务与当前用户的访问权限。' >&2; exit 1; }
docker image inspect "$image" >/dev/null 2>&1 || {
    echo "找不到镜像 $image，请先运行 ./docker/build.sh。" >&2; exit 1;
}
image_uid="$(docker run --rm --entrypoint id "$image" -u)"
image_gid="$(docker run --rm --entrypoint id "$image" -g)"
if [[ "$image_uid" != "$(id -u)" || "$image_gid" != "$(id -g)" || "$image_uid" == 0 ]]; then
    echo '镜像 UID/GID 与当前用户不一致，请使用当前普通用户重新运行 ./docker/build.sh。' >&2
    exit 1
fi
domain="${ROS_DOMAIN_ID:-10}"
if [[ ! "$domain" =~ ^[0-9]+$ || ${#domain} -gt 3 ]] || (( 10#$domain > 232 )); then
    echo 'ROS_DOMAIN_ID 必须为 0 到 232 的整数。' >&2; exit 1
fi
args=(run --rm --init --network host --shm-size 1g --workdir /workspace
    --env "ROS_DOMAIN_ID=$domain" --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    --env "DOCKER_GAZEBO_MASTER_URI=${DOCKER_GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
    --env QT_X11_NO_MITSHM=1
    --mount "type=bind,source=$project_dir,target=/workspace")
[[ -t 0 && -t 1 ]] && args+=(-it)

# Nested bind mounts hide ALL previous native build artifacts without deleting them.
artifact_dir="${DOCKER_ARTIFACT_DIR:-$project_dir/.docker-workspace}"
mkdir -p "$artifact_dir"
artifact_dir="$(cd "$artifact_dir" && pwd)"
for name in build install log; do
    mkdir -p "$artifact_dir/$name" "$project_dir/$name"
    args+=(--mount "type=bind,source=$artifact_dir/$name,target=/workspace/$name")
done

# Only explicit DDS settings are forwarded. Host paths must be in /workspace,
# or use DOCKER_DDS_DIR for a read-only /dds mount and container paths in variables.
if [[ -n "${DOCKER_DDS_DIR:-}" ]]; then
    args+=(--mount "type=bind,source=$DOCKER_DDS_DIR,target=/dds,readonly")
fi
for key in ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER CYCLONEDDS_URI \
           FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; do
    if [[ -v "$key" ]]; then
        value="${!key}"
        # A host shell may already have sourced this repository's ros_env.sh.
        # Preserve the selected file, translating its mount path for the container.
        case "$value" in
            "$project_dir"/*) value="/workspace/${value#"$project_dir"/}" ;;
            "file://$project_dir/"*) value="file:///workspace/${value#"file://$project_dir/"}" ;;
        esac
        args+=(--env "$key=$value")
    fi
done

xhost_granted=0
auth_temp=''
cleanup() {
    if (( xhost_granted )); then xhost "-si:localuser:$(id -un)" >/dev/null 2>&1 || true; fi
    if [[ -n "$auth_temp" ]]; then rm -f -- "$auth_temp"; fi
}
trap cleanup EXIT
case "${DOCKER_GUI:-auto}" in auto|0|1) ;; *) echo 'DOCKER_GUI: auto / 0 / 1' >&2; exit 2;; esac
if [[ "${DOCKER_GUI:-auto}" != 0 && -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    args+=(--env "DISPLAY=$DISPLAY" --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly)
    # Use the desktop session's cookie when available (including Xwayland).
    auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
    auth_ready=0
    if [[ -r "$auth_file" && -s "$auth_file" ]] && command -v xauth >/dev/null; then
        # FamilyWild cookies also work with the container's different hostname.
        auth_temp="$(mktemp /tmp/outdoor-nav-xauth.XXXXXX)"
        cookies="$(xauth -f "$auth_file" nlist "$DISPLAY")"
        if [[ -n "$cookies" ]]; then
            sed 's/^..../ffff/' <<< "$cookies" | xauth -f "$auth_temp" nmerge -
            args+=(--env XAUTHORITY=/tmp/container.xauth
                --mount "type=bind,source=$auth_temp,target=/tmp/container.xauth,readonly")
            auth_ready=1
        fi
    fi
    if (( ! auth_ready )) && command -v xhost >/dev/null; then
        # Scope access to this UID and revoke only grants created by this script.
        if ! xhost | grep -Fq "SI:localuser:$(id -un)"; then
            xhost "+si:localuser:$(id -un)" >/dev/null
            xhost_granted=1
        fi
    elif (( ! auth_ready )); then
        echo '缺少 Xauthority 与 xhost；X11 可能拒绝窗口连接。' >&2
    fi
elif [[ "${DOCKER_GUI:-auto}" == 1 ]]; then
    echo 'GUI 要求 DISPLAY 和 /tmp/.X11-unix；请在宿主机桌面终端运行。' >&2; exit 1
else
    echo '未启用 X11；仿真请传 gui:=false rviz:=false。' >&2
fi

if [[ -d /dev/dri ]]; then
    args+=(--device /dev/dri:/dev/dri)
    # Device groups may have different names inside the image; numeric IDs work.
    while IFS= read -r device_gid; do args+=(--group-add "$device_gid"); done \
        < <(find /dev/dri -type c -printf '%G\n' | sort -u)
fi
case "${DOCKER_GPU:-auto}" in auto|0|1) ;; *) echo 'DOCKER_GPU: auto / 0 / 1' >&2; exit 2;; esac
gpu=0
if [[ "${DOCKER_GPU:-auto}" != 0 ]] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
    # A real launch probe checks driver AND toolkit; Docker runtime names alone
    # do not prove --gpus works. No robot/ROS process is launched by this probe.
    if timeout 20 docker run --rm --gpus all --env NVIDIA_DRIVER_CAPABILITIES=utility \
        --entrypoint nvidia-smi "$image" -L >/dev/null 2>&1; then gpu=1; fi
fi
if (( gpu )); then
    args+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display)
elif [[ "${DOCKER_GPU:-auto}" == 1 ]]; then
    echo 'NVIDIA GPU/Container Toolkit 探测失败；检查驱动或使用 DOCKER_GPU=0。' >&2; exit 1
else
    echo '使用普通图形设备；无可用硬件渲染时可设置 LIBGL_ALWAYS_SOFTWARE=1。' >&2
fi
if [[ -n "${LIBGL_ALWAYS_SOFTWARE:-}" ]]; then args+=(--env "LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"); fi
if [[ $# == 0 ]]; then set -- bash; fi
docker "${args[@]}" "$image" "$@"
