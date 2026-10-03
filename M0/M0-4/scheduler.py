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
