# input_utils.py

from __future__ import annotations

import yaml

TASK_FILE = "grasp/task/task.yml"

def load_task(task_name: str, task_file: str = TASK_FILE):
    """Read a task entry from the central task.yml and return (config_path, model_path)."""
    with open(task_file, "r", encoding="utf-8") as f:
        tasks = yaml.safe_load(f).get("tasks", {})
    if task_name not in tasks:
        raise ValueError(f"Task {task_name} not found in {task_file}")
    return tasks[task_name]["config"], tasks[task_name]["model"]
