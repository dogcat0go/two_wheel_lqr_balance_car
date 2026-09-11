#!/usr/bin/env bash
# ROS 2 Jazzy 服务器版一键安装（无 GUI）
# Raspberry Pi 4B + Ubuntu 24.04 Server
#
# 不修改 Ubuntu 系统源（ubuntu.sources / sources.list）。
# rosdep 走清华镜像，避免 raw.githubusercontent.com DNS 失败。
#
# 用法（普通用户，不要整脚本 sudo）：
#   bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh
#
set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

ROS_DISTRO="jazzy"
TUNA_ROSDEP_LIST="https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list"
TUNA_ROSDISTRO_INDEX="https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"

if [[ "${EUID}" -eq 0 ]]; then
  echo -e "${RED}请用普通用户运行，不要 sudo 整份脚本（脚本内部会自己 sudo）。${NC}"
  exit 1
fi

if ! grep -q "24.04" /etc/os-release; then
  echo -e "${RED}错误：当前系统不是 Ubuntu 24.04${NC}"
  exit 1
fi

SUDO=(sudo)
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a
export NEEDRESTART_SUSPEND=1

echo "=============================================="
echo "  ROS 2 Jazzy 服务器版一键安装（干净版）"
echo "  Raspberry Pi 4B + Ubuntu 24.04 Server"
echo "  无 GUI，不改系统 apt 源"
echo "=============================================="
echo

# ---------- rosdep：不要走官方 rosdep init（会去 GitHub） ----------
write_rosdep_list() {
  local dest="/etc/ros/rosdep/sources.list.d/20-default.list"
  local tmp
  tmp="$(mktemp)"
  "${SUDO[@]}" mkdir -p /etc/ros/rosdep/sources.list.d
  if [[ -f "${dest}" ]]; then
    echo "  rosdep 源列表已存在，跳过写入"
    rm -f "${tmp}"
    return 0
  fi
  if curl -fsSL --retry 3 --retry-delay 2 --max-time 30 -o "${tmp}" "${TUNA_ROSDEP_LIST}"; then
    echo "  已从清华镜像下载 rosdep 源列表"
  else
    echo "  清华镜像下载失败，写入内置列表"
    cat >"${tmp}" <<'EOF'
# os-specific listings first
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
# generic
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/base.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/python.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/ruby.yaml
EOF
  fi
  "${SUDO[@]}" cp "${tmp}" "${dest}"
  "${SUDO[@]}" chmod 644 "${dest}"
  rm -f "${tmp}"
}

download_ros_key() {
  local dest="/usr/share/keyrings/ros-archive-keyring.gpg"
  local tmp u
  if [[ -s "${dest}" ]]; then
    echo "  ROS GPG key 已存在，跳过下载"
    return 0
  fi
  tmp="$(mktemp)"
  for u in \
    "https://cdn.jsdelivr.net/gh/ros/rosdistro@master/ros.key" \
    "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"; do
    echo "  下载 ROS GPG key: ${u}"
    if curl -fsSL --retry 3 --retry-delay 2 --max-time 30 -o "${tmp}" "${u}"; then
      if grep -q 'BEGIN PGP PUBLIC KEY' "${tmp}"; then
        gpg --batch --yes --dearmor -o "${tmp}.gpg" "${tmp}"
        "${SUDO[@]}" cp "${tmp}.gpg" "${dest}"
        rm -f "${tmp}.gpg"
      else
        "${SUDO[@]}" cp "${tmp}" "${dest}"
      fi
      "${SUDO[@]}" chmod 644 "${dest}"
      rm -f "${tmp}"
      return 0
    fi
  done
  rm -f "${tmp}"
  echo -e "${RED}无法下载 ROS GPG key（DNS/GitHub 不通）。${NC}"
  exit 1
}

echo -e "${YELLOW}[1/6] 设置 Locale...${NC}"
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y locales curl ca-certificates gnupg
"${SUDO[@]}" locale-gen en_US en_US.UTF-8
"${SUDO[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

echo -e "${YELLOW}[2/6] 添加 ROS 2 软件源...${NC}"
download_ros_key
if [[ ! -f /etc/apt/sources.list.d/ros2.list && ! -f /etc/apt/sources.list.d/ros2-jazzy-rpi.sources ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}") main" \
    | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
else
  echo "  ROS 2 apt 源已存在，跳过写入"
fi

echo -e "${YELLOW}[3/6] 更新系统并安装开发工具...${NC}"
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get upgrade -y
"${SUDO[@]}" apt-get install -y \
  ros-dev-tools \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-argcomplete \
  git \
  wget \
  curl \
  build-essential \
  cmake

echo -e "${YELLOW}[4/6] 安装 ROS 2 Jazzy 核心（ros-base）...${NC}"
"${SUDO[@]}" apt-get install -y ros-jazzy-ros-base

echo -e "${YELLOW}[5/6] 安装 SLAM 与导航相关包（无 GUI）...${NC}"
"${SUDO[@]}" apt-get install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-cartographer \
  ros-jazzy-cartographer-ros \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  ros-jazzy-laser-filters \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-rmw-cyclonedds-cpp

echo -e "${YELLOW}[6/6] 初始化 rosdep 并配置环境...${NC}"
# 官方 `rosdep init` 会访问 raw.githubusercontent.com，国内树莓派经常
# Temporary failure in name resolution。这里直接写清华镜像列表。
write_rosdep_list
export ROSDISTRO_INDEX_URL="${TUNA_ROSDISTRO_INDEX}"
# pkg_resources DeprecationWarning 可忽略
if ! rosdep update --rosdistro "${ROS_DISTRO}"; then
  echo -e "${RED}rosdep update 仍失败。可稍后手动：${NC}"
  echo "  export ROSDISTRO_INDEX_URL=${TUNA_ROSDISTRO_INDEX}"
  echo "  rosdep update --rosdistro ${ROS_DISTRO}"
fi

BASHRC="${HOME}/.bashrc"
touch "${BASHRC}"
if ! grep -q "source /opt/ros/jazzy/setup.bash" "${BASHRC}"; then
  {
    echo ""
    echo "# ROS 2 Jazzy"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "export ROS_DOMAIN_ID=0"
  } >>"${BASHRC}"
fi
if ! grep -q "ROSDISTRO_INDEX_URL" "${BASHRC}"; then
  echo "export ROSDISTRO_INDEX_URL=\"${TUNA_ROSDISTRO_INDEX}\"" >>"${BASHRC}"
fi
if ! grep -q "colcon_argcomplete" "${BASHRC}"; then
  echo 'if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi' >>"${BASHRC}"
fi

mkdir -p "${HOME}/ros2_ws/src"
# 空工作空间 colcon 失败不阻断
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
(
  cd "${HOME}/ros2_ws"
  colcon build --symlink-install || true
)
if ! grep -q "ros2_ws/install/setup.bash" "${BASHRC}"; then
  echo 'if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi' >>"${BASHRC}"
fi

echo
echo -e "${GREEN}=============================================="
echo "  安装完成！（纯服务器版，无 GUI）"
echo "==============================================${NC}"
echo
echo "请新开一个终端，或执行：  source ~/.bashrc"
echo
echo "验证命令："
echo "  ros2 --help"
echo "  ros2 pkg list | grep -E 'slam|nav2|cartographer'"
echo
echo "说明：rosdep init 报 GitHub DNS 失败是正常现象，本脚本已改用清华镜像。"
echo "      pkg_resources DeprecationWarning 可以忽略。"
echo
