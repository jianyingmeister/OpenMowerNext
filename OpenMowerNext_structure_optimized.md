# 🧭 OpenMowerNext 优化后目录结构（推荐）

以下是经整理、分层优化后的目录树，符合 **ROS2 Humble 工作空间标准结构**，并剔除了冗余与临时文件。

---

```bash
OpenMowerNext/
├── 📁 src/                        # ROS2 源码主目录
│   └── 📁 open_mower_next/
│       ├── 📁 action/                # 自定义 Action 定义
│       ├── 📁 msg/                   # 自定义消息
│       ├── 📁 srv/                   # 自定义服务
│       ├── 📁 description/           # 机器人模型 (URDF/Xacro)
│       ├── 📁 launch/                # Launch 文件
│       ├── 📁 rviz/                  # RViz 配置
│       ├── 📁 maps/                  # 地图与环境文件
│       ├── 📁 sim/                   # Gazebo 仿真节点
│       ├── 📁 docking_helper/        # 停靠辅助模块
│       ├── 📁 map_recorder/          # 地图记录模块
│       ├── 📁 map_server/            # 地图服务模块
│       ├── 📁 ntrip_client/          # RTK/NTRIP 客户端
│       ├── 📁 ublox_f9p/             # GPS 驱动
│       ├── 📁 vesc/                  # 电机控制模块
│       ├── 📁 foxglove/              # Foxglove Studio 配置
│       ├── 📄 CMakeLists.txt
│       └── 📄 package.xml
│
├── 📁 config/                     # 全局参数配置
│   ├── ⚙ nav2_params.yaml
│   ├── ⚙ robot_localization.yaml
│   ├── ⚙ controllers.yaml
│   ├── ⚙ twist_mux.yaml
│   ├── ⚙ gps.yaml
│   ├── 📁 hardware/
│   │   ├── ⚙ openmower.yaml
│   │   ├── ⚙ vesc.yaml
│   │   └── ⚙ yardforce500.yaml
│   └── 🎨 view_bot.rviz
│
├── 📁 cmake/                      # 自定义 CMake 脚本
│   ├── docking_helper.cmake
│   ├── map_recorder.cmake
│   ├── map_server.cmake
│   └── sim.cmake
│
├── 📁 docs/                       # 开发文档与架构说明
│   ├── 📁 architecture/
│   │   ├── docking-helper.md
│   │   ├── localization.md
│   │   ├── map-recorder.md
│   │   ├── map-server.md
│   │   ├── omros2-firmware.md
│   │   ├── ros-workspace.md
│   │   └── sim-node.md
│   ├── 📁 assets/
│   │   ├── gazebo.jpg
│   │   ├── geojson-schema.json
│   │   ├── logo.png
│   │   └── ros.png
│   ├── simulator.md
│   ├── roadmap.md
│   └── visualisation.md
│
├── 📁 utils/                      # 构建与辅助工具
│   ├── docker-entrypoint.sh
│   ├── install-custom-deps.sh
│   └── xesc2tcp.sh
│
├── 📁 worlds/                     # Gazebo 世界文件
│   ├── solar_farm.world
│   └── empty.sdf
│
├── 📁 .devcontainer/
│   ├── Dockerfile
│   ├── devcontainer.json
│   └── default.env
│
├── 📁 .github/workflows/
│   ├── build.yml
│   ├── devcontainer.yml
│   └── docs.yml
│
├── 🧩 Dockerfile
├── ⚙ docker-compose.yaml
├── ⚙ custom_deps.yaml
├── 📄 Makefile
├── 📄 README.md
├── 📄 LICENSE
└── 📄 NOTES.md
```

---

## 🧹 清理建议

执行以下命令清除冗余文件：

```bash
# 删除 Windows 附带的元数据
find . -name "*.Zone.Identifier" -delete

# 删除临时 / 副本 / 备份文件
find . -regex ".*\(副本\|backup\|save\).*" -delete

# 删除构建缓存与空目录
rm -rf build install log
find . -type d -empty -delete
```

---

## ⚙ 一键优化脚本（clean_and_optimize.sh）

```bash
#!/bin/bash
set -e
echo "🧹 Cleaning temporary and redundant files..."
find . -name "*.Zone.Identifier" -delete
find . -regex ".*\(副本\|backup\|save\).*" -delete
find . -type d -empty -delete
rm -rf build install log

echo "📦 Ensuring proper ROS2 src layout..."
mkdir -p src/open_mower_next
mv */CMakeLists.txt */package.xml src/open_mower_next/ 2>/dev/null || true

echo "✅ Done. You can now run:"
echo "   colcon build --symlink-install"
```

---

### 📘 说明
- 本结构遵循 ROS2 `ament_cmake` 与 `colcon` 构建规范。  
- 支持 **Gazebo + Nav2 + RTK 驱动集成**。  
- 目录清晰分层，便于扩展和团队协作。
