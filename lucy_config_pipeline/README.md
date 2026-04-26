# lucy_config_pipeline

Service/action layer for the hardware config workflow.

- Config store operations on `config/hardware/` (`active.yaml`, `configs/*.yaml`, backups)
- Validation (schema + URDF cross-check)
- `ConfigurePipeline` action phases: validate, generate, build (RP2040 cmake/make), flash (stub)

## Services

- `config/list` (`lucy_msgs/srv/ListConfigs`)
- `config/get` (`lucy_msgs/srv/GetConfig`)
- `config/save` (`lucy_msgs/srv/SaveConfig`)
- `config/activate` (`lucy_msgs/srv/ActivateConfig`)
- `config/delete` (`lucy_msgs/srv/DeleteConfig`)

## Action

- `configure_pipeline` (`lucy_msgs/action/ConfigurePipeline`)

## Launch

```bash
ros2 launch lucy_config_pipeline config_pipeline.launch.py robot_package:=thais_urdf
```

Optional: pass `config_dir:=/abs/path/to/config/hardware` to override source-path detection.
