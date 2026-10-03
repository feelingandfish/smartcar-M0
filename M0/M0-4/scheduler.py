import argparse
import yaml
import json
import time
import random

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
    if random.random()<t["success_rate"]:
      print(f"{name}成功")
      results.append({"name":name,"status":"SUCCESS"})
    else:
     print(f"{name}失败")
     results.append({"name":name,"status":"FAILED"})
print("执行完毕",results)
