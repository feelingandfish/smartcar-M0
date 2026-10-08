import argparse
import yaml
import json
import time
import random
import sys

# ---------- 彩色输出 ----------
def setup_colors():
    if sys.stdout.isatty():
        return {"GREEN": "\033[92m", "RED": "\033[91m",
                "YELLOW": "\033[93m", "BLUE": "\033[94m", "RESET": "\033[0m"}
    return {"GREEN": "", "RED": "", "YELLOW": "", "BLUE": "", "RESET": ""}

C = setup_colors()

# ---------- 读命令行参数 ----------
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--report", default="report.json")
    parser.add_argument("--seed", type=int, default=None)
    return parser.parse_args()

# ---------- 读配置文件 ----------
def load_config(path):
    with open(path) as f:
        if path.endswith(".json"):
            return json.load(f)
        return yaml.safe_load(f)

# ---------- 建任务字典 ----------
def build_tasks_dict(config):
    d = {}
    for t in config["tasks"]:
        d[t["name"]] = t
    return d

# ---------- 拓扑排序 ----------
def topological_sort(tasks_dict):
    visited, order = {}, []
    def dfs(name):
        if visited.get(name) == 2:
            return
        if visited.get(name) == 1:
            raise ValueError("依赖成环：" + name)
        visited[name] = 1
        for dep in tasks_dict[name].get("dependencies", []):
            if dep not in tasks_dict:
                raise ValueError("依赖不存在：" + dep)
            dfs(dep)
        visited[name] = 2
        order.append(name)
    for name in tasks_dict:
        dfs(name)
    return order

# ---------- 校验配置 ----------
def validate_tasks(config):
    if not config.get("tasks"):
        raise ValueError("任务列表为空")
    for t in config["tasks"]:
        if not (0 <= t["success_rate"] <= 1):
            raise ValueError(f"success_rate 必须在0到1之间，任务 {t['name']}")

# ---------- 执行单个任务 ----------
def run_task(name, t, tasks_dict, results, start_time, timeout, C):
    started = time.time()
    # 检查前置是否失败/跳过
    for dep in t.get("dependencies", []):
        if results[dep]["status"] in ["SKIPPED", "FAILED", "TIMEOUT"]:
            print(f"{C['BLUE']}  {name} 跳过（依赖失败）{C['RESET']}")
            results[name] = {"name": name, "status": "SKIPPED", "attempts": 0,
                             "duration": 0, "started_at": started, "ended_at": started}
            return
    # 执行前检查超时
    if time.time() - start_time > timeout:
        results[name] = {"name": name, "status": "TIMEOUT", "attempts": 0,
                         "duration": 0, "started_at": started, "ended_at": started}
        return
    # 重试最多3次
    for attempt in range(3):
        print(f"{C['YELLOW']}正在执行：{name}（第{attempt+1}次）{C['RESET']}")
        time.sleep(t["duration"])
        if time.time() - start_time > timeout:
            print(f"{C['RED']}  {name} 超时{C['RESET']}")
            results[name] = {"name": name, "status": "TIMEOUT", "attempts": attempt+1,
                             "duration": t["duration"], "started_at": started, "ended_at": time.time()}
            return
        if random.random() < t["success_rate"]:
            print(f"{C['GREEN']}  {name} 成功{C['RESET']}")
            results[name] = {"name": name, "status": "SUCCESS", "attempts": attempt+1,
                             "duration": t["duration"], "started_at": started, "ended_at": time.time()}
            return
        print(f"{C['RED']}  {name} 失败{C['RESET']}")
    print(f"{C['BLUE']}  {name} 3次都失败，跳过{C['RESET']}")
    results[name] = {"name": name, "status": "SKIPPED", "attempts": 3,
                     "duration": t["duration"], "started_at": started, "ended_at": time.time()}

# ---------- 生成报告 ----------
def generate_report(results, order, start_time, timeout, report_path):
    total_duration = time.time() - start_time
    report = {
        "timeout": total_duration > timeout,
        "total_duration": round(total_duration, 2),
        "tasks": [results[name] for name in order]
    }
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"报告已保存到 {report_path}")

# ---------- 主入口 ----------
def main():
    args = parse_args()
    try:
        config = load_config(args.config)
        validate_tasks(config)
        print("配置读取成功")
        print("任务列表：", [t["name"] for t in config["tasks"]])
        tasks_dict = build_tasks_dict(config)
        order = topological_sort(tasks_dict)
        print("执行顺序：", order)
    except FileNotFoundError:
        print("错误：找不到文件", args.config); exit(1)
    except yaml.YAMLError:
        print("错误：YAML文件格式错误"); exit(1)
    except json.JSONDecodeError:
        print("错误：JSON文件格式错误"); exit(1)
    except ValueError as e:
        print("错误：", e); exit(1)

    if args.seed:
        random.seed(args.seed)

    results = {}
    start_time = time.time()
    timeout = args.timeout if args.timeout else config.get("timeout", 9999)

    for name in order:
        run_task(name, tasks_dict[name], tasks_dict, results, start_time, timeout, C)

    generate_report(results, order, start_time, timeout, args.report)

if __name__ == "__main__":
    main()
