#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v docker >/dev/null || { echo '请先安装 Docker Engine。' >&2; exit 1; }
if [[ "$(id -u)" == 0 || "$(id -g)" == 0 ]]; then
    echo '请用有 Docker 访问权限的普通用户构建，镜像 UID/GID 不能为 0。' >&2
    exit 1
fi
docker info >/dev/null || { echo 'Docker 不可用；请检查服务与当前用户的访问权限。' >&2; exit 1; }
exec docker build --file "$project_dir/docker/Dockerfile" \
    --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)" \
    --tag "${DOCKER_IMAGE:-outdoor_nav_humble}" "$@" "$project_dir"
