

---

# 🦾 Grasp Test Framework

基于 **SAPIEN** 的抓取 proposal 测试与验证框架。支持运行单个/多个任务、指定 proposal、保存修正结果并进行稳定性验证。

---

## 📂 项目结构

```
grasp/
│── task/                # 存放任务配置
│   ├── task.yml         # 总任务索引
│   ├── bag/001/...
│   ├── cup/002/...
│   └── ...
│
│── panda/               # Panda 手爪 URDF
│── test_main.py         # 主程序 (测试入口)
```

* **`task.yml`**：记录所有任务类别与编号，指向对应的 `.yml` proposals 与 `.glb` 模型文件
* **`test_main.py`**：测试入口，支持运行单个/多个任务，并输出修正结果 `batch_res_xxx.yml`

---

## 🚀 使用方法

### 1️⃣ 运行单个任务

```bash
python test_main.py --task bag --id 001 --viewer
```

* `--task`：任务类别 (`bag`, `cup`, `knife` 等)
* `--id`：任务编号 (`001`, `002`)
* `--viewer`：启用可视化

---

### 2️⃣ 运行指定 proposal

```bash
python test_main.py --task bag --id 001 --proposal grasp_12 grasp_34
```

* 只会测试 `grasp_12` 和 `grasp_34`

---

### 3️⃣ 运行某一类别的所有任务

```bash
python test_main.py --task bag
```

* 会运行 `bag/001`, `bag/002`, `bag/003`

---

### 4️⃣ 运行所有任务

```bash
python test_main.py --all
```

---

### 5️⃣ 自定义输入

```bash
python test_main.py --cfg grasp/task/bag/001/graspgen_proposals_topk.yml \
                    --glb grasp/task/bag/001/bag_scaled.glb
```

* 可直接指定 `.yml` 和 `.glb` 文件路径，不依赖 `task.yml`

---

## ⚙️ 参数说明

| 参数           | 说明                           |
| ------------ | ---------------------------- |
| `--cfg`      | 指定抓取配置文件路径                   |
| `--glb`      | 指定 GLB 模型路径                  |
| `--task`     | 任务类别 (`bag`, `cup`, `knife`) |
| `--id`       | 任务编号 (`001`, `002`)          |
| `--proposal` | 指定要运行的 proposal（支持多个）        |
| `--all`      | 运行所有任务                       |
| `--viewer`   | 是否启用可视化                      |

---

## 📌 注意事项

1. `task.yml` 中 `config` 统一使用 `graspgen_proposals_topk.yml`
2. 模型文件统一为 `xxx_scaled.glb`
3. 输出结果保存为 `batch_res_{task_name}.yml`，会覆盖已有文件，请注意备份

---

## 🔗 依赖

安装依赖：

```bash
conda create -n hongaho_sapien python=3.10 -y

pip install sapien

pip install -r requirements.txt
```

---

## 🎬 演示流程

以 `bag/001` 为例，验证修正前后抓取的稳定性。

### 第一次运行（原始 proposals）

```bash
python grasp/test_main.py --task bag --id 001 
```

输出示例：

```
[INFO] ▶️ 开始测试 proposal grasp_12
[INFO] Proposal grasp_12 ✅ 初步成功
[INFO] Motion 1: Δ=0.012345
[INFO] Motion 2: Δ=0.003210
[INFO] Motion 3: Δ=0.004321
[INFO] Proposal grasp_12 ✅ 最终成功
```

保存结果：

```
grasp/task/bag/001/batch_res_bag.001.yml
```

---

### 第二次运行（修正 proposals）

```bash
python grasp/test_main.py --cfg grasp/task/bag/001/batch_res_bag.001.yml --glb grasp/task/bag/001/bag_scaled.glb 
```

输出示例：

```
[INFO] ▶️ 开始测试 proposal grasp_12
[INFO] Proposal grasp_12 ✅ 初步成功
[INFO] Motion 1: Δ=0.000842
[INFO] Motion 2: Δ=0.001004
[INFO] Motion 3: Δ=0.000992
[INFO] Proposal grasp_12 ✅ 最终成功
```

---

### 对比结果

* **第一次运行**：Motion Δ 偏大 → 抓取姿态不稳定
* **第二次运行**：Motion Δ 显著减小 → 抓取更稳定

通过这种方式，可以直观验证 **proposal 修正前后的效果差异**。

---
