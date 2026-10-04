import argparse
import yaml
import json
import time
import random

# 彩色输出
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"

# 读命令行参数
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=True)
parser.add_argument("--timeout", type=float, default=None)
parser.add_argument("--report", default="report.json")
parser.add_argument("--seed", type=int, default=None)
args = parser.parse_args()

# 读配置文件
with open(args.config) as f:
    if args.config.endswith(".json"):
        config = json.load(f)
    else:
        config = yaml.safe_load(f)

print("配置读取成功")
print("任务列表：", [t["name"] for t in config["tasks"]])

# 建任务字典
tasks_dict = {}
for t in config["tasks"]:
    tasks_dict[t["name"]] = t

# 拓扑排序
visited = {}
order = []

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

for t in config["tasks"]:
    dfs(t["name"])

print("执行顺序：", order)
#设置随机数种子
if args.seed:
    random.seed(args.seed)
#执行任务
results=[]
for name in order:
    t=tasks_dict[name]
    print(f"正在执行:{name}")
    time.sleep(t["duration"])
#随机判定成功失败
results = {}
start_time = time.time()
timeout = args.timeout if args.timeout else config.get("timeout", 9999)

for name in order:
    t = tasks_dict[name]
    
    # 检查前置是否失败/跳过
    skipped = False
    for dep in t.get("dependencies", []):
        if results[dep]["status"] in ["SKIPPED", "FAILED", "TIMEOUT"]:
            skipped = True
            break
    if skipped:
        print(f"  {name} 跳过（依赖失败）")
        results[name] = {"name": name, "status": "SKIPPED", "attempts": 0}
        continue
    
    # 执行前检查超时
    elapsed = time.time() - start_time
    if elapsed > timeout:
        results[name] = {"name": name, "status": "TIMEOUT", "attempts": 0}
        continue
    
    # 重试最多3次
    success = False
    for attempt in range(3):
        print(f"{YELLOW}正在执行：{name}（第{attempt+1}次）{RESET}")
        time.sleep(t["duration"])
        
        # sleep后检查超时
        elapsed = time.time() - start_time
        if elapsed > timeout:
            print(f"{RED}  {name} 超时{RESET}")
            results[name] = {"name": name, "status": "TIMEOUT", "attempts": attempt+1}
            success = False
            break
        
        if random.random() < t["success_rate"]:
            print(f"{GREEN}  {name} 成功{RESET}")
            results[name] = {"name": name, "status": "SUCCESS", "attempts": attempt+1}
            success = True
            break
        else:
            print(f"{RED}  {name} 失败{RESET}")

    
    if not success:
        if name not in results:
            print(f"{BLUE}  {name} 3次都失败，跳过{RESET}")
            results[name] = {"name": name, "status": "SKIPPED", "attempts": 3}

print("执行完毕")
for name in order:
    status = results[name]["status"]
    if status == "SUCCESS":
        print(f"{GREEN}  {name}: {status}{RESET}")
    elif status == "SKIPPED":
        print(f"{BLUE}  {name}: {status}{RESET}")
    else:
        print(f"{RED}  {name}: {status}{RESET}")

