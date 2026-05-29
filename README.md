# mavpilot

> 🇬🇧 [English version](README_EN.md)

**Асинхронный контроллер PX4-дрона на Python** — последовательное автономное управление через MAVLink, встроенная 3D-визуализация в браузере и режим без железа.

[![CI](https://github.com/Onikore/mavpilot/actions/workflows/ci.yml/badge.svg)](https://github.com/Onikore/mavpilot/actions)
[![PyPI](https://img.shields.io/pypi/v/mavpilot)](https://pypi.org/project/mavpilot/)
[![Python](https://img.shields.io/pypi/pyversions/mavpilot)](https://pypi.org/project/mavpilot/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Возможности

| | |
|---|---|
| **Чистый asyncio API** | Пишите последовательную логику миссии через `await` — без колбэков и машин состояний |
| **PX4 OFFBOARD режим** | Стримит `SET_POSITION_TARGET_LOCAL_NED` на частоте 50 Гц |
| **Точная посадка** | Визуально-направляемый спуск через простой callback API |
| **Движение в теле дрона** | `goto_body_relative()` без ручного пересчёта NED/курс |
| **Ограничение скорости рыскания** | Плавные переходы курса (по умолчанию 15 °/с, настраивается) |
| **Визуализация в браузере** | Живая 3D-траектория + телеметрия через HTTP+SSE — без npm, без CDN |
| **Mock-режим** | Встроенный физический симулятор — тестируйте миссию без SITL и железа |
| **Потокобезопасность** | Heartbeat, receiver и streamer крутятся в фоновых потоках |

---

## Установка

```bash
pip install mavpilot
```

Или из исходников:

```bash
git clone https://github.com/Onikore/mavpilot
cd mavpilot
pip install -e ".[dev]"
```

**Зависимость времени выполнения:** [pymavlink](https://pypi.org/project/pymavlink/) (устанавливается автоматически).

---

## Быстрый старт — mock-режим

Дрон и SITL не нужны:

```bash
# Квадратная траектория
python -m mavpilot --mock

# Траектория в виде звезды
python -m mavpilot --mock --pattern star

# Демо точной посадки
python -m mavpilot --mock --precision-land
```

Откройте **http://localhost:8765** в браузере — увидите живую 3D-визуализацию.

---

## Использование как библиотека

```python
import asyncio
from mavpilot import DroneController

async def mission():
    # `async with` подключается на входе и завершает работу на выходе (aclose());
    # если блок выходит из-за исключения и дрон ещё в воздухе — сначала
    # вызывается emergency_land().
    async with DroneController(
        connection_string="udp:127.0.0.1:14540",  # SITL по умолчанию
        enable_viz=True,   # визуализация в браузере на :8765
    ) as drone:
        await drone.apply_safe_params()  # рекомендуемые параметры безопасности PX4
        await drone.wait_until_ready()   # ждём EKF / LOCAL_POSITION_NED

        await drone.takeoff(altitude_m=5.0)

        # Координаты NED (x=Север, y=Восток, z=Вниз)
        await drone.goto(x=10, y=0, z=-5)
        await drone.goto(x=10, y=10, z=-5, yaw_deg=90)
        await drone.goto_body_relative(forward_m=5, right_m=0, down_m=0)
        await drone.hover(duration_s=3.0)

        await drone.land()

asyncio.run(mission())
```

### Точная посадка

Передайте callback, возвращающий смещение маркера в **системе координат тела дрона (FRD)**:

```python
from mavpilot import DroneController, MarkerObservation

def get_marker() -> MarkerObservation | None:
    # подключите свой визуальный пайплайн
    # dx = смещение вперёд (м), dy = смещение вправо (м)
    return MarkerObservation(dx=0.3, dy=-0.1)

async def mission():
    async with DroneController(mock=True, enable_viz=False) as drone:
        await drone.takeoff(altitude_m=10.0)
        result = await drone.precision_land(
            get_marker_offset=get_marker,
            descent_rate_mps=0.3,
            final_altitude_m=0.5,
            horizontal_tolerance_m=0.15,
            min_altitude_floor_m=0.3,   # новый параметр в v0.2.0
        )
        if not result:
            # status ∈ {ABORTED_AT_FLOOR, MARKER_LOST, TIMEOUT}
            print(f"precision_land не приземлился: {result.status.value}")
            print(f"финальная позиция: {result.final_position}")
```

### Перевод пикселей камеры в смещение в теле дрона

```python
from mavpilot.utils import pixel_to_body_offset

dx, dy = pixel_to_body_offset(
    px_norm_x=0.1,            # нормализованные координаты [-1, 1]
    px_norm_y=-0.05,
    camera_hfov_deg=90.0,
    camera_vfov_deg=60.0,
    altitude_above_ground_m=drone.get_local_position().altitude,
    camera_mount_yaw_deg=0.0,
)
```

---

## CLI

```
python -m mavpilot [ОПЦИИ]

Опции:
  --connection STR      MAVLink endpoint  [по умолчанию: udp:127.0.0.1:14540]
  --mock                Симуляторный режим без железа
  --viz-port INT        Порт браузерной визуализации  [по умолчанию: 8765]
  --viz-host STR        Интерфейс визуализатора [по умолчанию: 127.0.0.1]
                        Используйте 0.0.0.0 для доступа из локальной сети
  --no-viz              Отключить браузерную визуализацию
  --precision-land      Точная посадка с симулированным маркером
  --pattern {square,star}  Паттерн полёта в демо  [по умолчанию: square]
```

### Поведение при ошибках и Ctrl-C

- **Ctrl-C** в любой момент миссии вызывает `emergency_land()`. Это включает: смену режима на `AUTO_LAND`, ожидание касания земли (до 10 с), отправку команды `MAV_CMD_NAV_LAND` если режим завис, и в крайнем случае `DO_FLIGHTTERMINATION` (мгновенное обесточивание моторов — дрон падает).
- **RTL не входит в `emergency_land()`**. Возврат на точку старта — это отдельная штатная операция (`drone.return_to_launch()`), не аварийная.
- Любое необработанное исключение в миссии (включая `KeyboardInterrupt`) также вызывает `emergency_land()`.

### Watchdog телеметрии и протокольная безопасность (v0.2.0)

- **Watchdog телеметрии** — `telemetry_watchdog_s` (по умолчанию 2 с). Если за это окно не приходит свежий `LOCAL_POSITION_NED`, стример выставляет флаг watchdog, и следующий вызов миссии (`takeoff`/`goto`/`set_yaw`/`land`/`return_to_launch`/`precision_land`) бросает `DroneError`. `emergency_land()` намеренно игнорирует флаг — это путь восстановления, ради которого watchdog и срабатывает.
- **Проверка здоровья EKF** — `wait_until_ready()` теперь проверяет ещё и здоровье EKF AHRS (`SYS_STATUS`, бит 5), а не только свежесть позиции.
- **`send_command_long()`** — даёт доступ к COMMAND_ACK через Future: ожидает финальный ACK по ключу `(cmd_id, target_sys, target_comp)`. `IN_PROGRESS` продлевает дедлайн; дубликат команды в полёте, таймаут или не-`ACCEPTED` результат бросают `DroneError`.
- **`get_yaw_deg()`** нормализован к `[-180, 180]`.

---

## Справочник API

### `DroneController(…)`

```python
DroneController(
    connection_string = "udp:127.0.0.1:14540",
    source_system     = 255,
    source_component  = MAV_COMP_ID_MISSIONPLANNER,
    loop_hz           = 50.0,       # частота стриминга сетпоинтов
    enable_viz        = True,       # запустить браузерную визуализацию
    viz_port          = 8765,
    mock              = False,      # симулятор без железа
    yaw_slew_rate_deg = 15.0,       # макс. скорость рыскания (°/с)
)
```

### Методы управления полётом

| Метод | Описание |
|---|---|
| `await connect(timeout_s)` | Открыть MAVLink и запустить фоновые потоки |
| `await apply_safe_params()` | Записать рекомендуемые параметры безопасности PX4 |
| `await wait_until_ready(timeout_s)` | Ждать пока EKF не выдаст LOCAL_POSITION_NED |
| `await takeoff(altitude_m, timeout_s)` | Арм, OFFBOARD режим, набор высоты |
| `await goto(x, y, z, yaw_deg, …)` | Лететь в точку NED |
| `await goto_relative(dx, dy, dz, …)` | Смещение от текущей позиции NED |
| `await goto_body_relative(fwd, right, down, …)` | Смещение в системе тела дрона |
| `await set_yaw(yaw_deg, timeout_s)` | Разворот на месте |
| `await hover(duration_s)` | Удерживать позицию |
| `await land(timeout_s)` | AUTO_LAND, ждать приземления |
| `await precision_land(callback, …)` | Визуально-направляемый спуск; возвращает `PrecisionLandResult` |
| `await return_to_launch(timeout_s)` | AUTO_RTL, ждать приземления |
| `await emergency_land()` | Цепочка: AUTO_LAND → NAV_LAND → DO_FLIGHTTERMINATION |
| `await aclose()` / `async with` | Остановить потоки и закрыть соединение (рекомендуется) |
| `close()` | Синхронное завершение (устарело; используйте `aclose()`) |

### Телеметрия

| Метод | Возвращает |
|---|---|
| `get_local_position()` | `Position(x, y, z)` в метрах NED |
| `get_yaw_rad()` / `get_yaw_deg()` | Текущий курс |
| `is_armed()` | `bool` |
| `is_offboard()` | `bool` |
| `landed_state()` | `int` (1 = на земле, 2 = в воздухе) |

### Датаклассы

```python
from mavpilot import Position, MarkerObservation

# Позиция в NED (x=Север, y=Восток, z=Вниз)
pos: Position       # pos.altitude == -pos.z

# Смещение маркера в системе тела дрона FRD
obs: MarkerObservation  # dx=вперёд, dy=вправо, dz=вниз (опционально)
```

---

## Система координат

mavpilot использует **NED-конвенцию PX4** из `LOCAL_POSITION_NED`:

| Ось | Направление | Примечание |
|---|---|---|
| x | Север (+) | |
| y | Восток (+) | |
| z | Вниз (+) | высота = `-z` |

Утилиты для преобразования координат:

```python
from mavpilot.utils import body_to_ned, ned_to_body, pixel_to_body_offset
```

---

## Визуализация

Лёгкий встроенный HTTP+SSE сервер раздаёт **3D-вид на Three.js** без сборки и пакетного менеджера. Откройте `http://localhost:8765` пока дрон работает.

Правая панель отображает:
- Статус арма и режим полёта
- Позицию, скорость, курс, заряд батареи в реальном времени
- Активный сетпоинт
- Лог команд (взлёт, goto, посадка, …)
- Сообщения PX4 STATUSTEXT

UI состоит из нативных ES-модулей, раздаваемых из `mavpilot/viz/static/` (`index.html` + `styles.css` + `main.js`/`scene.js`/`sse.js`/`telemetry.js`/`log.js`) — без сборщика, но нужен **современный браузер с поддержкой ES-модулей**. Параметр `max_clients` (по умолчанию 32) ограничивает число одновременных SSE-подключений; лишним клиентам возвращается HTTP 503.

---

## Архитектура

```
asyncio event loop  <-- ваш код миссии
        |
        v
 DroneController
        |
        +-- heartbeat_thread   (1 Гц MAVLink heartbeat)
        +-- receiver_thread    (разбор входящих MAVLink → self._tel)
        +-- streamer_thread    (публикация SET_POSITION_TARGET_LOCAL_NED @ 50 Гц)
        +-- viz_server         (опциональный HTTP+SSE → браузер)
```

Всё общее состояние защищено `_tel_lock` и `_setpoint_lock`. В коде миссии asyncio-примитивы не нужны.

### Структура модулей (v0.2.0)

```
mavpilot/
├── controller.py          # Фасад DroneController (корень композиции)
├── errors.py              # DroneError
├── types.py               # Position, MarkerObservation, PrecisionLand{Status,Result}
├── utils.py               # преобразования координат, pinhole, нормализация курса
├── constants.py           # биты режимов PX4, id MAV_CMD, type_mask
├── cli.py                 # точка входа argparse
├── core/                  # внутренние компоненты полётного стека
│   ├── connection.py      # MAVLinkConnection — pymavlink + лок I/O + heartbeat/receiver
│   ├── telemetry.py       # Telemetry — разбор входящих сообщений + кэш состояния
│   ├── commands.py        # CommandSender — COMMAND_LONG с маршрутизацией ACK через Future
│   ├── streamer.py        # OffboardStreamer — поток сетпоинтов + watchdog телеметрии
│   ├── mission.py         # MissionOps — takeoff/goto/hover/land/rtl/emergency_land
│   ├── precision_land.py  # PrecisionLand — визуальный спуск с нижним порогом высоты
│   ├── safety.py          # SafetyOps — wait_until_ready
│   └── mock.py            # MockMavConnection + встроенный симулятор
└── viz/                   # сервер браузерного UI (HTTP + SSE) + статические ES-модули
```

Каждый MAVLink send и recv проходит через `MAVLinkConnection`, владеющий единственным локом. Каждый подкомпонент получает зависимости через конструктор — легко мокать в тестах.

---

## Подключение к реальному железу

```python
# UART (Raspberry Pi <-> Pixhawk)
drone = DroneController(connection_string="/dev/ttyAMA0")

# UDP (SITL или мост компаньон-компьютер → GCS)
drone = DroneController(connection_string="udp:192.168.1.10:14540")

# TCP
drone = DroneController(connection_string="tcp:127.0.0.1:5760")
```

**Рекомендуемые параметры безопасности** (устанавливаются через `apply_safe_params()`):

| Параметр | Значение | Назначение |
|---|---|---|
| `COM_RCL_EXCEPT` | 7 | Нет failsafe в offboard / mission / hold |
| `COM_OBL_RC_ACT` | 4 | Потеря RC → hold, не RTL |
| `COM_OF_LOSS_T` | 2.0 с | Таймаут потери offboard |
| `COM_RC_IN_MODE` | 1 | RC не требуется |

---

## Разработка

```bash
# Установка в editable-режиме с dev-зависимостями
pip install -e ".[dev]"

# Тесты
pytest -q

# Линтер
ruff check mavpilot/

# Проверка типов
mypy mavpilot/
```

---

## Лицензия

[MIT](LICENSE)
