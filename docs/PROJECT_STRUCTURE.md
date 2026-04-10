# 项目结构说明

## 当前目录结构

```text
robot-arm project/
├─ main.py                      # 根启动入口，兼容 python main.py
├─ robot_simulator.py           # 根模拟器入口，兼容 python robot_simulator.py
├─ README.md
├─ requirements.txt
├─ docs/
│  ├─ PROJECT_STRUCTURE.md      # 当前这份结构说明
│  └─ STAGE3_测试清单.md         # Stage 3 测试清单
├─ logs/                        # 日志输出目录
├─ runtime_data/                # 运行期数据，如 transforms.json
├─ 数据文件/                     # 示例点位和标定测试数据
└─ polyworks_robot_arm/
   ├─ __init__.py
   ├─ main.py                   # Qt 桌面程序真实入口
   ├─ common/
   │  ├─ app_logger.py
   │  ├─ config.py
   │  ├─ data_parser.py
   │  ├─ exceptions.py
   │  └─ result_types.py
   ├─ integrations/
   │  └─ polyworks_com.py
   ├─ measurement/
   │  └─ services.py
   ├─ calibration/
   │  ├─ calibration_service.py
   │  ├─ transform_models.py
   │  ├─ transform_repository.py
   │  └─ transform_service.py
   ├─ robot/
   │  ├─ robot_protocol.py
   │  ├─ robot_server.py
   │  ├─ robot_session.py
   │  ├─ robot_simulator.py
   │  └─ task_router.py
   ├─ controllers/
   │  └─ measurement_controller.py
   └─ ui/
      ├─ ui_main_window.py
      ├─ ui_robot_link_tab.py
      └─ ui_calibration_tab.py
```

## 分层原则

- `common/`: 放所有跨模块复用的基础能力，例如配置、异常、日志、基础数据结构、文件解析。
- `integrations/`: 放和外部软件直接打交道的接口层，目前主要是 `PolyWorks COM`。
- `measurement/`: 放纯测量业务流程，不负责 UI，也不负责机器人协议。
- `calibration/`: 放 Stage 3 的标定、变换模型、变换持久化。
- `robot/`: 放机器人通讯协议、Socket 服务端、会话状态、模拟器。
- `controllers/`: 放总调度器，把 UI、测量、标定、机器人通信串起来。
- `ui/`: 放所有 Qt 页面和窗口。

## 推荐阅读顺序

1. `polyworks_robot_arm/common/config.py`
2. `polyworks_robot_arm/common/exceptions.py`
3. `polyworks_robot_arm/common/result_types.py`
4. `polyworks_robot_arm/integrations/polyworks_com.py`
5. `polyworks_robot_arm/measurement/services.py`
6. `polyworks_robot_arm/robot/robot_protocol.py`
7. `polyworks_robot_arm/robot/task_router.py`
8. `polyworks_robot_arm/calibration/transform_models.py`
9. `polyworks_robot_arm/calibration/calibration_service.py`
10. `polyworks_robot_arm/controllers/measurement_controller.py`
11. `polyworks_robot_arm/ui/ui_main_window.py`
12. `polyworks_robot_arm/ui/ui_robot_link_tab.py`
13. `polyworks_robot_arm/ui/ui_calibration_tab.py`

## 启动方式

桌面程序：

```bash
python main.py
```

机器人模拟器：

```bash
python robot_simulator.py
```

## 后续维护建议

- 新增公共常量时，优先放到 `common/config.py`。
- 新增业务异常时，优先放到 `common/exceptions.py`。
- 新增机器人协议任务时，优先修改 `robot/task_router.py` 和 `robot/robot_protocol.py`。
- 新增坐标变换或标定流程时，优先放到 `calibration/`，不要直接塞进 UI。
- UI 页面如果继续增多，建议后续再把 `ui/` 细分为 `tabs/` 和 `widgets/`。
