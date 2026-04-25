# lucy_config_generator

Python tool that reads the hardware mapping YAML from an **URDF** (`config/hardware/active.yaml`) and generates:

1. **Firmware** — one `config_<board_id>.c` per RP2040 board (filename equals the **`boards:`** key, e.g. `config_rp2040_left_arm.c`), suitable for `micro_ros_raspberrypi_pico_sdk` (`constant.h` / `INTERNAL_SERVO_*`, `SERVO_TYPE_*`, `DEG_TO_RAD`).
2. **ros2_control** — `inmoov_ros2_control.xacro` with one `<ros2_control>` block per board in **`boards:`** document order; `hardware_name` / `node_name` / `publisher_topic` are **derived and validated** from each board id and YAML fields (see `schema.py`). Shoulder Y joints stay on whichever board the YAML assigns them to (typically `rp2040_torso_head`).
3. **Controllers** — `controllers.yaml` with `joint_state_broadcaster` / trajectory controllers and **`extra_joints`** as *URDF joints that are not any YAML actuator* (passive / unmapped only; all actuator rows are trajectory-controlled regardless of `enabled`).

Schema validation lives in `lucy_config_generator/schema.py` for reuse by **`lucy_config_pipeline`**.

## CLI

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

generate_config \
  --input src/thais_urdf/config/hardware/active.yaml \
  --urdf src/thais_urdf/description/urdf/inmoov.urdf.xacro \
  --base-path src/thais_urdf/description \
  --controller-config src/thais_urdf/config/controllers.yaml \
  --output-dir /tmp/gen_out \
  --targets all
```

Options:

| Flag | Meaning |
|------|---------|
| `--targets` | `all` (default), `firmware`, `ros2_control`, or `controllers` |
| `--boards` | Optional comma-separated board ids (e.g. `rp2040_left_arm,rp2040_torso_head`) |

`--urdf`, `--base-path`, and `--controller-config` are passed to **`xacro`** the same way as **`thais_urdf`** tests so joint discovery matches the real robot.

## Templates

Jinja2 sources are under the Python package at `lucy_config_generator/lucy_config_generator/templates/`:

| Template | Output |
|----------|--------|
| `config_internal_only_board.c.j2` | `board_class: internal_servo_only` — single internal PWM `dump_config()` |
| `config_internal_i2c_board.c.j2` | `board_class: internal_servo_i2c_pwm` — internal PWM + I2C/PCA scaffold |
| `ros2_control.xacro.j2` | One `<ros2_control>` block per board (names derived from board id) |
| `controllers.yaml.j2` | `controller_manager` + per-board joint lists + `extra_joints` |

**Disabled actuators** (`enabled: false`): **omitted from firmware C** only (no `virtual_pin` row on the Pico for that joint). They **remain** in **`ros2_control`** and in the per-board **trajectory controller** joint list so the stack is uniform; firmware ignores commands at `virtual_pin` indices it does not own. **`extra_joints`** lists only URDF joints that are **not** any actuator row (typically passive links).

## Tests

```bash
colcon test --packages-select lucy_config_generator --event-handlers console_direct+
```

Golden files live in `test/fixtures/` alongside `test_mapping.yaml`.