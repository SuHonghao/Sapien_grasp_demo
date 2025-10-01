明白 ✅
你是想在 **使用方法** 部分补充说明：

* `bag` 表示运行整个类别（类别下所有子任务都会跑一遍）
* `bag.001` 表示运行该类别下的某一个具体物体

我帮你把 **README.md** 更新好（Markdown 格式，直接可复制）：

```markdown
# Grasp Simulation with SAPIEN

本项目基于 [SAPIEN](https://sapien.ucsd.edu/) 物理仿真引擎，实现了 **Panda Gripper + 力反馈** 的抓取动作验证。  
程序会读取 **isaac_grasp 格式的抓取 proposals**，逐一在仿真环境中重建场景并检测抓取的成功与否，最终输出结果到 `.yml` 文件。

---

## 📂 项目结构
```

grasp/
│── panda/               # Panda 手爪 URDF 模型
│── task/                # 任务配置（task.yml + 每个物体子目录）
│   ├── bag/
│   │   ├── bag.yml      # 抓取 proposals 配置
│   │   └── bag.glb      # 物体 3D 模型
│   ├── teapot/
│   │   ├── teapot.yml
│   │   └── teapot.glb
│   └── task.yml         # 总任务入口
│
│── test_main.py         # 主程序入口
│── load_glb.py          # GLB 模型加载工具
│── math_utils.py        # 数学/坐标变换工具
│── float_utils.py       # 物体悬浮处理
│── world.py             # 仿真环境初始化
│── physx_utils.py       # 物理引擎参数设置
│── gripper_demo.py      # Gripper 控制逻辑

````

---

## 🔧 环境依赖

建议使用 **conda** 管理 Python 环境。

### 1. 创建环境
```bash
conda create -n sapien_grasp python=3.10
conda activate sapien_grasp
````

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

如果没有 `requirements.txt`，可以运行：

```bash
pip freeze > requirements.txt
```

---

## ▶️ 使用方法

### 单个任务

```bash
python test_main.py --task bag.001 --viewer
```

* `bag.001` 表示运行 **bag 类别下的一个具体物体**。
* `--viewer` 是否启用 GUI 渲染。

### 整个类别

```bash
python test_main.py --task bag --viewer
```

* `bag` 表示运行 **bag 类别下的所有物体**。

### 全部任务

```bash
python test_main.py --all --viewer
```

* 依次运行 `task.yml` 中配置的所有类别与物体。

### 自定义配置

```bash
python test_main.py --cfg grasp/task/teapot/teapot.yml --glb grasp/task/teapot/teapot.glb --viewer
```

* 手动指定 proposal 配置文件 (`.yml`) 和物体模型 (`.glb`)。

---

## 📤 输出结果

运行后会在任务目录下生成结果文件，例如：

```
grasp/task/teapot/batch_res_teapot.001.yml
```

结果文件格式如下：

```yaml
grasp_01:
  best_key: grasp_01
  status: success
  tcp_in_obj: [0.01, 0.02, 0.03]
  gripper_quat_in_obj: [1.0, 0.0, 0.0, 0.0]
```

---

## 📝 备注

* 当前仅支持 **isaac_grasp 格式** 输入文件。
* 抓取稳定性通过 **TCP 与物体质心距离变化** 检测，超过阈值则判定为失败。
* 阈值可在 `test_main.py` 中调整（默认 `0.005 m`）。

---

## 🤝 致谢

* [SAPIEN 团队](https://sapien.ucsd.edu/)
* [IsaacGrasp 数据集](https://isaac-grasp.github.io/)

```

---

要不要我帮你再画一个 **任务选择逻辑图**（`bag` vs `bag.001` vs `--all`）放到 README 里，更直观？
```
