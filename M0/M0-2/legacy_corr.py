import yaml
import csv
import math

CONFIG_PATH = "config.yaml"

try:
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
except FileNotFoundError:
    print(f"错误：找不到配置文件 {CONFIG_PATH}")
    exit(1)

csv_path = cfg["input_csv"]
col_x = cfg["columns"]["x"]
col_y = cfg["columns"]["y"]

xs = []
ys = []

try:
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row[col_x]))
            ys.append(float(row[col_y]))
    n = len(xs)
except FileNotFoundError:
    print(f"错误：找不到数据文件 {csv_path}")
    exit(1)
except KeyError:
    print(f"错误：csv 里没有列名 {col_x} 或 {col_y}")
    exit(1)



sum_x = 0.0
sum_y = 0.0
for i in range(n):
    sum_x = sum_x + xs[i]
    sum_y = sum_y + ys[i]

mean_x = sum_x / n
mean_y = sum_y / n

dx = 0.0
dy = 0.0
prod = 0.0
for i in range(n):
    a = xs[i] - mean_x
    b = ys[i] - mean_y
    dx = dx + a * a
    dy = dy + b * b
    prod = prod + a * b

denom = (dx**0.5) *(dy**0.5) 
r = prod / denom

print("n =", n)
print("mean_x =", mean_x)
print("mean_y =", mean_y)
print("r =", r)
