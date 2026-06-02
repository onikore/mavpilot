#!/usr/bin/env bash
# Запускает 05_precision_land_gazebo.py с нужными переменными окружения:
#   LD_LIBRARY_PATH  — путь к libaruco.so.3.1
#   PYTHONPATH       — путь к aruco Python-пакету (из venv arucofractal)
#
# Использование:
#   source /opt/ros/jazzy/setup.bash
#   bash examples/run_gazebo.sh
#   bash examples/run_gazebo.sh --connection udp:127.0.0.1:14540

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LIBARUCO_DIR="$REPO_ROOT/arucofractal/third_party/aruco-3.1.12/install/lib"
ARUCO_SITE="$REPO_ROOT/arucofractal/venv/lib/python3.12/site-packages"

if [ ! -f "$LIBARUCO_DIR/libaruco.so" ]; then
    echo "Ошибка: libaruco.so не найден в $LIBARUCO_DIR"
    echo "Сначала соберите aruco:"
    echo "  cd $REPO_ROOT/arucofractal/third_party/aruco-3.1.12"
    echo "  cmake -B build -DCMAKE_INSTALL_PREFIX=install ."
    echo "  cmake --build build --target install -j\$(nproc)"
    exit 1
fi

export LD_LIBRARY_PATH="$LIBARUCO_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$ARUCO_SITE${PYTHONPATH:+:$PYTHONPATH}"

echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "PYTHONPATH (aruco): $ARUCO_SITE"

exec python3 "$SCRIPT_DIR/05_precision_land_gazebo.py" "$@"
