#!/usr/bin/env bash
# Raspberry Pi 4B + Ubuntu 24.04 Server (noble / arm64) → ROS 2 Jazzy 一键安装
#
# 本仓库 ESP32 固件文档里写的是 Humble；Ubuntu 24.04 官方二进制只有 Jazzy。
# 树莓派上请装 Jazzy。micro-ROS agent 若仍要对接 Humble 固件，用 --docker 拉 humble 镜像。
#
# 会先修好 Ubuntu 24 树莓派镜像上常见的 apt 源问题，例如：
#   - ubuntu.sources 里两段配置完全重复（Target ... configured multiple times）
#   - 错误套件 noble-backports-security / noble-updates-security（根本不存在）
#   - 缺 noble-updates、noble-backports（装 ROS 2 时依赖对不上）
#   - 把 DEB822 写进 sources.list，或把 amd64 的 archive.ubuntu.com 用到 arm64 上
#
# 用法：
#   sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh
#   sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --fix-apt-only
#   sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --mirror tuna
#   sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --variant desktop
#   sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --docker
#
set -euo pipefail

ROS_DISTRO="jazzy"
UBUNTU_CODENAME="noble"
VARIANT="ros-base"          # ros-base | desktop
MIRROR="auto"               # auto | tuna | ustc | aliyun | official
FIX_APT_ONLY=0
SKIP_UPGRADE=0
WITH_DOCKER=0
NO_BASHRC=0
FORCE=0
FORCE_IPV4=0
PRINT_SOURCES=0

BACKUP_DIR=""
REAL_USER="${SUDO_USER:-${USER}}"
REAL_HOME="$(getent passwd "${REAL_USER}" | cut -d: -f6 || true)"
[[ -n "${REAL_HOME}" ]] || REAL_HOME="${HOME}"

log()  { printf '\033[1;32m[ros2-install]\033[0m %s\n' "$*" >&2; }
warn() { printf '\033[1;33m[ros2-install]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[ros2-install]\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
树莓派 4B / Ubuntu 24.04 Server / arm64 → ROS 2 Jazzy 一键安装

用法:
  sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh [选项]

选项:
  --mirror auto|tuna|ustc|aliyun|official
                          apt / ROS 2 镜像。默认 auto（测速后选最快的）
  --variant ros-base|desktop
                          默认 ros-base（Server 无桌面，树莓派建议这个）
  --fix-apt-only          只重写 Ubuntu 源并 apt update，不装 ROS 2
  --skip-upgrade          修源后不跑 apt full-upgrade
  --docker                额外安装 docker.io，并拉取 micro-ros-agent:humble
  --no-bashrc             不改用户 ~/.bashrc
  --ipv4                  apt 强制 IPv4（IPv6 不通时用）
  --force                 跳过树莓派 / 架构检查
  --print-sources         只打印将要写入的 ubuntu.sources，不改系统
  -h, --help              显示帮助

示例:
  sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --fix-apt-only --mirror tuna
  sudo bash scripts/install_ros2_jazzy_rpi4_ubuntu24.sh --mirror tuna --docker
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mirror)          MIRROR="${2:?}"; shift 2 ;;
    --variant)         VARIANT="${2:?}"; shift 2 ;;
    --fix-apt-only)    FIX_APT_ONLY=1; shift ;;
    --skip-upgrade)    SKIP_UPGRADE=1; shift ;;
    --docker)          WITH_DOCKER=1; shift ;;
    --no-bashrc)       NO_BASHRC=1; shift ;;
    --ipv4)            FORCE_IPV4=1; shift ;;
    --force)           FORCE=1; shift ;;
    --print-sources)   PRINT_SOURCES=1; shift ;;
    -h|--help)         usage; exit 0 ;;
    *) die "未知参数: $1  （--help 查看用法）" ;;
  esac
done

case "${VARIANT}" in
  ros-base|desktop) ;;
  *) die "--variant 只能是 ros-base 或 desktop" ;;
esac
case "${MIRROR}" in
  auto|tuna|ustc|aliyun|official) ;;
  *) die "--mirror 只能是 auto|tuna|ustc|aliyun|official" ;;
esac

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a
export NEEDRESTART_SUSPEND=1
export LANG="${LANG:-C.UTF-8}"

need_root() {
  [[ "${EUID}" -eq 0 ]] || die "请用 sudo 运行"
}

# ---------- 镜像 ----------
# 官方 ports 只有 http；国内镜像用 https。
# 合法套件只有: noble / noble-updates / noble-backports / noble-security
# 不存在 noble-backports-security、noble-updates-security。
ubuntu_ports_uri() {
  case "$1" in
    official) echo "http://ports.ubuntu.com/ubuntu-ports" ;;
    tuna)     echo "https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports" ;;
    ustc)     echo "https://mirrors.ustc.edu.cn/ubuntu-ports" ;;
    aliyun)   echo "https://mirrors.aliyun.com/ubuntu-ports" ;;
    *) die "内部错误: 未知 Ubuntu 镜像 $1" ;;
  esac
}

ros2_uri() {
  case "$1" in
    official) echo "http://packages.ros.org/ros2/ubuntu" ;;
    tuna)     echo "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu" ;;
    ustc)     echo "https://mirrors.ustc.edu.cn/ros2/ubuntu" ;;
    aliyun)   echo "https://mirrors.aliyun.com/ros2/ubuntu" ;;
    *) die "内部错误: 未知 ROS 2 镜像 $1" ;;
  esac
}

rosdistro_index_url() {
  case "$1" in
    official) echo "" ;;
    *)        echo "https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml" ;;
  esac
}

probe_ms() {
  local url="$1"
  local t
  t="$(curl -o /dev/null -sS --max-time 3 -w '%{time_total}' "$url" 2>/dev/null || true)"
  [[ -n "${t}" ]] || { echo 9999; return; }
  awk -v t="${t}" 'BEGIN { printf "%d", t * 1000 }'
}

pick_mirror() {
  if [[ "${MIRROR}" != "auto" ]]; then
    echo "${MIRROR}"
    return
  fi
  if ! command -v curl >/dev/null 2>&1; then
    echo tuna
    return
  fi
  local best="tuna" best_ms=9999 name ms
  for name in tuna ustc aliyun official; do
    ms="$(probe_ms "$(ubuntu_ports_uri "${name}")/")"
    log "测速 ${name}: ${ms} ms"
    if [[ "${ms}" -lt "${best_ms}" ]]; then
      best="${name}"
      best_ms="${ms}"
    fi
  done
  if [[ "${best_ms}" -ge 9000 ]]; then
    warn "测速全部超时，改用 tuna"
    best="tuna"
  fi
  echo "${best}"
}

ubuntu_sources_text() {
  local uri="$1"
  cat <<EOF
# 由 scripts/install_ros2_jazzy_rpi4_ubuntu24.sh 生成
# 树莓派 / arm64 必须走 ubuntu-ports，不要用 archive.ubuntu.com
Types: deb
URIs: ${uri}
Suites: ${UBUNTU_CODENAME} ${UBUNTU_CODENAME}-updates ${UBUNTU_CODENAME}-backports
Components: main restricted universe multiverse
Architectures: arm64
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: ${uri}
Suites: ${UBUNTU_CODENAME}-security
Components: main restricted universe multiverse
Architectures: arm64
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
}

ros2_sources_text() {
  local uri="$1"
  cat <<EOF
# 由 scripts/install_ros2_jazzy_rpi4_ubuntu24.sh 生成
Types: deb
URIs: ${uri}
Suites: ${UBUNTU_CODENAME}
Components: main
Architectures: arm64
Signed-By: /usr/share/keyrings/ros-archive-keyring.gpg
EOF
}

# ---------- 环境检查 ----------
check_os() {
  [[ -r /etc/os-release ]] || die "找不到 /etc/os-release"
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${VERSION_CODENAME:-}" != "${UBUNTU_CODENAME}" ]]; then
    die "当前系统是 ${PRETTY_NAME:-unknown}，本脚本只支持 Ubuntu 24.04 (${UBUNTU_CODENAME})"
  fi
  local arch
  arch="$(dpkg --print-architecture)"
  if [[ "${arch}" != "arm64" && "${FORCE}" -ne 1 ]]; then
    die "当前架构是 ${arch}。树莓派 4B 的 Ubuntu 24 Server 应为 arm64。确认无误可加 --force"
  fi
  if [[ "${FORCE}" -ne 1 && -r /proc/device-tree/model ]]; then
    local model
    model="$(tr -d '\0' </proc/device-tree/model)"
    if ! grep -qi 'raspberry pi' <<<"${model}"; then
      warn "设备型号是「${model}」，不是树莓派；继续安装。不是本机可加 --force 跳过检查"
    else
      log "设备: ${model}"
    fi
  fi
  log "系统: ${PRETTY_NAME}  架构: ${arch}"
}

# ---------- 备份 / 清理错误源 ----------
stamp_backup_dir() {
  BACKUP_DIR="/etc/apt/backup-ros2-install-$(date +%Y%m%d-%H%M%S)"
  mkdir -p "${BACKUP_DIR}"
  log "apt 配置备份目录: ${BACKUP_DIR}"
}

backup_if_exists() {
  local f="$1"
  [[ -e "${f}" ]] || return 0
  cp -a "${f}" "${BACKUP_DIR}/$(echo "${f}" | tr '/' '_')"
}

# 把会跟 ubuntu.sources 打架、或格式错误的旧源挪走
disable_conflicting_apt_files() {
  local f base
  if [[ -f /etc/apt/sources.list ]]; then
    backup_if_exists /etc/apt/sources.list
    if grep -qE '^[[:space:]]*Types:' /etc/apt/sources.list; then
      warn "sources.list 里写了 DEB822（Types:），apt 会读失败，已移走"
      mv /etc/apt/sources.list "${BACKUP_DIR}/sources.list.deb822-misplaced"
      printf '# Ubuntu sources live in /etc/apt/sources.list.d/ubuntu.sources\n' \
        >/etc/apt/sources.list
    elif grep -qE '^[[:space:]]*deb(-src)?[[:space:]]' /etc/apt/sources.list; then
      warn "sources.list 里还有旧式 deb 行，已注释，避免和 ubuntu.sources 重复"
      sed -i -E 's/^[[:space:]]*(deb(-src)?[[:space:]].*)/# \1/' /etc/apt/sources.list
    fi
  fi

  shopt -s nullglob
  for f in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
    base="$(basename "${f}")"
    case "${base}" in
      ubuntu.sources|ros2-jazzy-rpi.sources) continue ;;
    esac
    if grep -qE 'ports\.ubuntu\.com|archive\.ubuntu\.com|security\.ubuntu\.com|ubuntu-ports|mirrors\.(tuna|ustc|aliyun)|ros2/ubuntu|packages\.ros\.org' "${f}"; then
      backup_if_exists "${f}"
      warn "停用冲突源文件: ${f}"
      mv "${f}" "${f}.disabled"
    fi
  done
  shopt -u nullglob
}

write_ubuntu_sources() {
  local uri="$1"
  mkdir -p /etc/apt/sources.list.d
  if [[ -f /etc/apt/sources.list.d/ubuntu.sources ]]; then
    backup_if_exists /etc/apt/sources.list.d/ubuntu.sources
  fi
  ubuntu_sources_text "${uri}" >/etc/apt/sources.list.d/ubuntu.sources
  chmod 644 /etc/apt/sources.list.d/ubuntu.sources
  log "已写入 /etc/apt/sources.list.d/ubuntu.sources"
  log "  URIs: ${uri}"
  log "  Suites: ${UBUNTU_CODENAME} ${UBUNTU_CODENAME}-updates ${UBUNTU_CODENAME}-backports + ${UBUNTU_CODENAME}-security"
}

maybe_force_ipv4() {
  if [[ "${FORCE_IPV4}" -eq 1 ]]; then
    printf 'Acquire::ForceIPv4 "true";\n' >/etc/apt/apt.conf.d/99force-ipv4
    log "已写入 /etc/apt/apt.conf.d/99force-ipv4"
  fi
}

downgrade_ubuntu_sources_to_http() {
  local uri
  uri="$(ubuntu_ports_uri "${MIRROR}" | sed 's|^https://|http://|')"
  warn "https 源失败，改写为 http: ${uri}"
  write_ubuntu_sources "${uri}"
}

apt_update_strict() {
  local rc=0 out=""
  set +e
  out="$(apt-get update 2>&1)"
  rc=$?
  set -e
  printf '%s\n' "${out}"

  if [[ "${rc}" -ne 0 && "${FORCE_IPV4}" -ne 1 ]]; then
    warn "apt update 失败，改用 IPv4 再试"
    FORCE_IPV4=1
    maybe_force_ipv4
    set +e
    out="$(apt-get update 2>&1)"
    rc=$?
    set -e
    printf '%s\n' "${out}"
  fi

  if [[ "${rc}" -ne 0 ]] && grep -qE 'Certificate verification|SSL|https' <<<"${out}"; then
    if grep -q 'https://' /etc/apt/sources.list.d/ubuntu.sources; then
      downgrade_ubuntu_sources_to_http
      set +e
      out="$(apt-get update 2>&1)"
      rc=$?
      set -e
      printf '%s\n' "${out}"
    fi
  fi

  if grep -q 'does not have a Release file' <<<"${out}"; then
    die "apt 源仍有不存在的套件（常见于 noble-backports-security）。备份在 ${BACKUP_DIR}，当前文件：/etc/apt/sources.list.d/ubuntu.sources"
  fi
  if grep -q 'configured multiple times' <<<"${out}"; then
    warn "仍有重复源警告。请检查 /etc/apt/sources.list.d/ 是否还有未停用的 ubuntu 源"
  fi
  printf '%s\n' "${out}" >/tmp/ros2-install-apt-update.log
  [[ "${rc}" -eq 0 ]] || die "apt update 失败 (exit ${rc})。完整日志：/tmp/ros2-install-apt-update.log"
}

ensure_bootstrap_pkgs() {
  apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 lsb-release locales apt-utils
}

# ---------- locale / swap ----------
ensure_locale() {
  if ! locale | grep -q 'UTF-8'; then
    warn "当前 locale 不是 UTF-8，正在生成 en_US.UTF-8"
  fi
  sed -i 's/^# *en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen
  grep -q '^en_US.UTF-8 UTF-8' /etc/locale.gen || echo 'en_US.UTF-8 UTF-8' >>/etc/locale.gen
  locale-gen en_US.UTF-8 >/dev/null
  update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  log "locale: ${LANG}"
}

ensure_swap() {
  local mem_kb swap_kb
  mem_kb="$(awk '/MemTotal:/ {print $2}' /proc/meminfo)"
  swap_kb="$(awk '/SwapTotal:/ {print $2}' /proc/meminfo)"
  if [[ "${mem_kb}" -ge 3500000 ]]; then
    return 0
  fi
  if [[ "${swap_kb}" -ge 1000000 ]]; then
    log "内存 ${mem_kb} kB，已有 swap ${swap_kb} kB"
    return 0
  fi
  local swapfile="/swapfile-ros2"
  if [[ ! -f "${swapfile}" ]]; then
    log "内存较小 (${mem_kb} kB)，创建 2G swap，避免装 ROS 时 OOM"
    fallocate -l 2G "${swapfile}" 2>/dev/null || dd if=/dev/zero of="${swapfile}" bs=1M count=2048 status=none
    chmod 600 "${swapfile}"
    mkswap "${swapfile}" >/dev/null
  fi
  swapon "${swapfile}" 2>/dev/null || true
  if ! grep -q "^${swapfile} " /etc/fstab; then
    echo "${swapfile} none swap sw 0 0" >>/etc/fstab
  fi
}

# ---------- ROS 2 源 ----------
download_ros_key() {
  local dest="/usr/share/keyrings/ros-archive-keyring.gpg"
  local tmp urls u
  tmp="$(mktemp)"
  urls=(
    "https://cdn.jsdelivr.net/gh/ros/rosdistro@master/ros.key"
    "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
    "https://github.com/ros/rosdistro/raw/master/ros.key"
  )
  for u in "${urls[@]}"; do
    log "下载 ROS GPG key: ${u}"
    if curl -fsSL --retry 3 --retry-delay 2 --max-time 30 "$u" -o "${tmp}"; then
      if grep -q 'BEGIN PGP PUBLIC KEY' "${tmp}"; then
        gpg --batch --yes --dearmor -o "${dest}" "${tmp}"
      else
        cp "${tmp}" "${dest}"
      fi
      chmod 644 "${dest}"
      rm -f "${tmp}"
      if [[ -s "${dest}" ]]; then
        log "ROS GPG key → ${dest}"
        return 0
      fi
    fi
  done
  rm -f "${tmp}"
  die "无法下载 ROS GPG key（GitHub 可能被墙）。可改用 --mirror tuna 后重试，或自行把 ros.key 放到 ${dest}"
}

write_ros2_sources() {
  local uri="$1"
  download_ros_key
  ros2_sources_text "${uri}" >/etc/apt/sources.list.d/ros2-jazzy-rpi.sources
  chmod 644 /etc/apt/sources.list.d/ros2-jazzy-rpi.sources
  log "已写入 ROS 2 源: ${uri}  (suite ${UBUNTU_CODENAME})"
}

# ---------- 安装 ----------
apt_full_upgrade() {
  [[ "${SKIP_UPGRADE}" -eq 1 ]] && { log "跳过 apt full-upgrade"; return 0; }
  log "apt full-upgrade（ROS 2 依赖 noble-updates 里的库版本）"
  apt-get -y \
    -o Dpkg::Options::=--force-confdef \
    -o Dpkg::Options::=--force-confold \
    full-upgrade
}

install_ros() {
  local pkg="ros-${ROS_DISTRO}-ros-base"
  if [[ "${VARIANT}" == "desktop" ]]; then
    pkg="ros-${ROS_DISTRO}-desktop"
    warn "desktop 含 RViz 等 GUI，Ubuntu Server 无桌面时意义不大，且更吃内存"
  fi
  log "安装 ${pkg} + ros-dev-tools"
  apt-get install -y \
    "${pkg}" \
    ros-dev-tools \
    python3-argcomplete \
    python3-pip \
    python3-venv
}

setup_rosdep() {
  local index
  index="$(rosdistro_index_url "${MIRROR}")"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init || true
  fi
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    mkdir -p /etc/ros/rosdep/sources.list.d
    if ! curl -fsSL --retry 3 --max-time 30 \
      -o /etc/ros/rosdep/sources.list.d/20-default.list \
      "https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list"; then
      cat >/etc/ros/rosdep/sources.list.d/20-default.list <<'EOF'
# default sources list for rosdep, generated by install_ros2_jazzy_rpi4_ubuntu24.sh
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte
EOF
    fi
  fi
  if [[ -n "${index}" ]]; then
    log "rosdep 使用清华 rosdistro 索引"
    sudo -u "${REAL_USER}" -H env ROSDISTRO_INDEX_URL="${index}" rosdep update --rosdistro "${ROS_DISTRO}" \
      || warn "rosdep update 失败（多半是 GitHub 网络），可稍后手动执行"
  else
    sudo -u "${REAL_USER}" -H rosdep update --rosdistro "${ROS_DISTRO}" \
      || warn "rosdep update 失败，可稍后手动执行"
  fi
}

patch_bashrc() {
  [[ "${NO_BASHRC}" -eq 1 ]] && return 0
  local bashrc="${REAL_HOME}/.bashrc"
  [[ -f "${bashrc}" ]] || touch "${bashrc}"
  local begin="# >>> ros2-jazzy-rpi4 >>>"
  local end="# <<< ros2-jazzy-rpi4 <<<"
  local block index
  index="$(rosdistro_index_url "${MIRROR}")"
  block="${begin}
# 由 scripts/install_ros2_jazzy_rpi4_ubuntu24.sh 写入
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  . /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi
"
  if [[ -n "${index}" ]]; then
    block+="export ROSDISTRO_INDEX_URL=\"${index}\"
"
  fi
  block+="${end}"

  if grep -qF "${begin}" "${bashrc}"; then
    sed -i '\|# >>> ros2-jazzy-rpi4 >>>|,|# <<< ros2-jazzy-rpi4 <<<|d' "${bashrc}"
  fi
  printf '\n%s\n' "${block}" >>"${bashrc}"
  chown "${REAL_USER}:${REAL_USER}" "${bashrc}" 2>/dev/null || true
  log "已写入 ${bashrc} （新开终端生效）"
}

install_docker_agent() {
  [[ "${WITH_DOCKER}" -eq 1 ]] || return 0
  log "安装 docker.io，并拉取 micro-ros-agent:humble（对本仓库 ESP32 Humble 固件）"
  apt-get install -y docker.io
  if id "${REAL_USER}" >/dev/null 2>&1; then
    usermod -aG docker "${REAL_USER}" || true
  fi
  systemctl enable --now docker || true
  if docker pull microros/micro-ros-agent:humble; then
    log "已拉取 microros/micro-ros-agent:humble"
  else
    warn "docker pull 失败。国内可稍后用："
    warn "  docker pull registry.cn-hangzhou.aliyuncs.com/fishros/micro-ros-agent:humble"
  fi
}

verify_ros() {
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  command -v ros2 >/dev/null || die "找不到 ros2 命令"
  if ros2 --help >/dev/null 2>&1; then
    log "验证: ros2 可用  ROS_DISTRO=${ROS_DISTRO}"
  else
    die "ros2 --help 失败"
  fi
}

print_next_steps() {
  cat <<EOF

安装完成。

  新开一个终端，或执行:
    source /opt/ros/${ROS_DISTRO}/setup.bash

  本机检查:
    ros2 topic list

  对本仓库 WiFi 遥测（ESP32 固件若仍是 Humble micro-ROS）:
    sudo docker run -it --rm --net=host --privileged \\
      microros/micro-ros-agent:humble udp4 --port 8888 -v6

  apt 旧配置备份在: ${BACKUP_DIR}

EOF
}

# ---------- main ----------
if [[ "${PRINT_SOURCES}" -eq 1 ]]; then
  MIRROR="$(pick_mirror)"
  ubuntu_sources_text "$(ubuntu_ports_uri "${MIRROR}")"
  exit 0
fi

need_root
check_os
MIRROR="$(pick_mirror)"
log "使用镜像: ${MIRROR}"

stamp_backup_dir
maybe_force_ipv4
disable_conflicting_apt_files
write_ubuntu_sources "$(ubuntu_ports_uri "${MIRROR}")"
apt_update_strict
ensure_bootstrap_pkgs

if [[ "${FIX_APT_ONLY}" -eq 1 ]]; then
  log "已按 --fix-apt-only 只修复 apt 源。再执行一次本脚本即可安装 ROS 2。"
  exit 0
fi

ensure_locale
ensure_swap
apt_full_upgrade
write_ros2_sources "$(ros2_uri "${MIRROR}")"
apt_update_strict
install_ros
setup_rosdep
patch_bashrc
install_docker_agent
verify_ros
print_next_steps
