#!/bin/bash
# =============================================================================
# 把 Orca 的 Isaac ROS 映像建好、推到 registry，讓 Jetson 只需要 pull
# =============================================================================
# 問題：在 Orin NX 上第一次 `isa` 要 40 分鐘以上，換人／換機／重灌就再來一次，
# 而且 Jetson 的 CPU 與散熱都不適合長時間編譯。映像本身有 20 GB 內容，
# 其中 86% 來自 NVIDIA 的 base（Triton + CUDA devel + PyTorch + TensorRT），
# 不是我們能砍掉的東西。
#
# 解法：建一次、推 registry、其他機器 pull。這是唯一能把「40 分鐘 build」
# 變成「一次 pull」的做法，而且完全不需要動任何 Dockerfile。
#
# 用法：
#   ./orca_registry.sh build            # 為本機架構建映像
#   ./orca_registry.sh build --arm64    # 在 x86 機器上交叉建 Jetson 用的映像
#   ./orca_registry.sh push             # 推到 registry
#   ./orca_registry.sh pull             # 從 registry 取回並打上本地 tag
#
# registry 位址與 tag 來自環境變數，預設走 GHCR（對 org 免費）：
#   ORCA_REGISTRY   預設 ghcr.io/nctu-auv
#   ORCA_IMAGE_TAG  預設 latest
#
# 推之前要先登入：
#   echo $GITHUB_TOKEN | docker login ghcr.io -u <你的帳號> --password-stdin
# -----------------------------------------------------------------------------
set -euo pipefail

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

REGISTRY="${ORCA_REGISTRY:-ghcr.io/nctu-auv}"
TAG="${ORCA_IMAGE_TAG:-latest}"
IMAGE_KEY="${CONFIG_IMAGE_KEY:-ros2_humble.realsense.orca25}"

usage() {
    sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

COMMAND="${1:-}"
shift || true

ARCH="$(uname -m)"
CROSS_ARM64=0
for arg in "$@"; do
    case "$arg" in
        --arm64) CROSS_ARM64=1 ;;
        -h|--help) usage 0 ;;
        *) echo "unknown argument: $arg" >&2; usage 1 ;;
    esac
done

if [[ $CROSS_ARM64 -eq 1 ]]; then
    ARCH="aarch64"
fi

REMOTE_IMAGE="${REGISTRY}/orca-autonomy:${TAG}-${ARCH}"
LOCAL_IMAGE="isaac_ros_dev-${ARCH}"

case "$COMMAND" in
build)
    if [[ $CROSS_ARM64 -eq 1 && "$(uname -m)" != "aarch64" ]]; then
        # 交叉建置需要 binfmt/qemu。Isaac 的 base 很大，這一步會很慢
        # （幾小時起跳），但它跑在開發機上而不是 Jetson 上，這才是重點。
        echo "==> 設定 qemu binfmt（交叉建置 arm64）"
        docker run --privileged --rm tonistiigi/binfmt --install arm64
        echo "==> 交叉建置 arm64 映像：${LOCAL_IMAGE}"
        "${ROOT}/build_image_layers.sh" \
            --image_key "aarch64.${IMAGE_KEY}" \
            --image_name "${LOCAL_IMAGE}" \
            --build_arg "PLATFORM=arm64" \
            --docker_arg "--platform=linux/arm64"
    else
        echo "==> 建置本機架構映像：${LOCAL_IMAGE}"
        "${ROOT}/build_image_layers.sh" \
            --image_key "${ARCH}.${IMAGE_KEY}" \
            --image_name "${LOCAL_IMAGE}"
    fi
    docker tag "${LOCAL_IMAGE}" "${REMOTE_IMAGE}"
    echo "==> 完成：${LOCAL_IMAGE}（已標記為 ${REMOTE_IMAGE}）"
    ;;

push)
    if [[ -z "$(docker image ls --quiet "${REMOTE_IMAGE}")" ]]; then
        echo "找不到映像 ${REMOTE_IMAGE}，請先執行 build" >&2
        exit 1
    fi
    echo "==> 推送 ${REMOTE_IMAGE}"
    docker push "${REMOTE_IMAGE}"
    ;;

pull)
    echo "==> 取回 ${REMOTE_IMAGE}"
    docker pull "${REMOTE_IMAGE}"
    # 打上 run_dev.sh 預期的本地名稱，這樣 `isa` 會直接沿用，
    # 不會再去跑 build_image_layers.sh。
    docker tag "${REMOTE_IMAGE}" "${LOCAL_IMAGE}"
    echo "==> 已標記為 ${LOCAL_IMAGE}；接下來 run_dev.sh 會直接用它"
    echo "    （記得加 --skip_image_build，或確認 Dockerfile 沒有本地改動）"
    ;;

""|-h|--help)
    usage 0
    ;;

*)
    echo "unknown command: ${COMMAND}" >&2
    usage 1
    ;;
esac
