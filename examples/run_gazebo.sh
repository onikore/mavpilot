#!/usr/bin/env bash
# Запускает примеры Gazebo с нужными переменными окружения:
#   LD_LIBRARY_PATH  — путь к libaruco.so.3.1
#   PYTHONPATH       — путь к aruco Python-пакету (из venv arucofractal)
#
# Использование:
#   source /opt/ros/jazzy/setup.bash
#
#   # OFFBOARD точная посадка (05):
#   bash examples/run_gazebo.sh
#   bash examples/run_gazebo.sh --connection udp:127.0.0.1:14540
#
#   # Ручной полёт → OFFBOARD → точная посадка (06):
#   bash examples/run_gazebo.sh 06
#   bash examples/run_gazebo.sh 06 --connection udp:127.0.0.1:14540

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

# Выбор скрипта: первый аргумент "06" → landing_target_publisher, иначе → precision_land
EXAMPLE="${1:-05}"
shift 2>/dev/null || true

case "$EXAMPLE" in
    05|precision_land)
        SCRIPT="$SCRIPT_DIR/05_precision_land_gazebo.py"
        ;;
    06|manual)
        SCRIPT="$SCRIPT_DIR/06_precision_land_gazebo_manual.py"
        ;;
    *)
        echo "Использование: bash run_gazebo.sh [05|06] [args...]"
        echo "  05  — автономно: взлёт → точная посадка (default)"
        echo "  06  — ручной полёт → OFFBOARD → точная посадка"
        exit 1
        ;;
esac

echo "Запуск: $SCRIPT $*"
exec python3 "$SCRIPT" "$@"
