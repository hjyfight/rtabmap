#!/usr/bin/env bash
# Install official Humble binaries and keep sources/configuration in Project.
set -euo pipefail
source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target="${1:-/home/jyfight/Project/rtabmap_slam}"
proxy="${RTABMAP_INSTALL_PROXY-http://127.0.0.1:7897}"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'ROS 2 Humble is required at /opt/ros/humble.' >&2
    exit 1
fi
mkdir -p "$target"
target="$(cd -- "$target" && pwd)"
if [[ "$source_dir" != "$target" ]]; then
    if [[ -f "$target/install.sh" ]]; then
        echo "Project already exists. Resume with: bash $target/install.sh $target" >&2
        exit 1
    fi
    cp -a -- "$source_dir/." "$target/"
fi
mkdir -p "$target/src" "$target/maps" "$target/logs"
exec > >(tee -a "$target/logs/install.log") 2>&1
apt_options=(-o Acquire::Retries=2 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)
git_options=()
if [[ -n "$proxy" ]]; then
    apt_options+=(-o "Acquire::http::Proxy=$proxy" -o "Acquire::https::Proxy=$proxy")
    git_options+=(-c "http.proxy=$proxy")
fi

refresh_apt() {
    sudo apt-get "${apt_options[@]}" update
}
install_rtabmap() {
    sudo apt-get "${apt_options[@]}" install -y ros-humble-rtabmap-ros
}

# Some ROS mirrors serve an index whose package files have already rotated
# away (404). Fall back to the official repository once in that case.
if ! refresh_apt || ! install_rtabmap; then
    echo 'ROS apt repository appears stale; retrying with official packages.ros.org' >&2
    ros_list=/etc/apt/sources.list.d/ros2.list
    if [[ -f "$ros_list" && ! -f "$ros_list.pre-rtabmap-install" ]]; then
        sudo cp -a "$ros_list" "$ros_list.pre-rtabmap-install"
    fi
    official_list="$(mktemp)"
    printf 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' > "$official_list"
    sudo cp "$official_list" "$ros_list"
    rm -f "$official_list"
    refresh_apt
    install_rtabmap
fi
dpkg-query -W -f='${Package}\t${Version}\n' 'ros-humble-rtabmap*' > "$target/logs/packages.tsv"
version="$(dpkg-query -W -f='${Version}' ros-humble-rtabmap-ros)"
version="${version%%-*}"
for repository in rtabmap rtabmap_ros; do
    url="https://github.com/introlab/$repository.git"
    if [[ "$repository" == rtabmap_ros ]]; then
        fallback=ros2
    else
        fallback=master
    fi
    branch="$fallback"
    candidates=("$version-humble" "v$version-humble" "$version" "v$version")
    for candidate in "${candidates[@]}"; do
        if [[ -n "$(git "${git_options[@]}" ls-remote --tags "$url" "refs/tags/$candidate")" ]]; then
            branch="$candidate"
            break
        fi
    done
    if [[ ! -f "$target/src/$repository/CMakeLists.txt" ]]; then
        git "${git_options[@]}" clone --depth 1 --branch "$branch" "$url" "$target/src/$repository"
    fi
    if [[ -e "$target/src/$repository/.git" ]]; then
        git -C "$target/src/$repository" log -1 --format='%H %s' > "$target/logs/${repository}_source_commit.txt"
    else
        cp "$target/docs/source_versions.md" "$target/logs/${repository}_source_commit.txt"
    fi
done
chmod +x "$target/install.sh" "$target/scripts/"*.sh
bash "$target/scripts/test_environment.sh"
echo "Installation and synthetic mapping test passed. Project: $target"
