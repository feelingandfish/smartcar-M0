import argparse
import yaml
import csv


def calc_corr(xs, ys):
    n = len(xs)
    sum_x = sum_y = 0.0
    for i in range(n):
        sum_x += xs[i]
        sum_y += ys[i]
    mean_x = sum_x / n
    mean_y = sum_y / n

    dx = dy = prod = 0.0
    for i in range(n):
        a = xs[i] - mean_x
        b = ys[i] - mean_y
        dx += a * a
        dy += b * b
        prod += a * b

    denom = (dx ** 0.5) * (dy ** 0.5)
    if denom == 0:
        raise ValueError("某一列数据全部相同，无法计算相关系数")
    r = prod / denom
    return n, mean_x, mean_y, r


def main():
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
    except FileNotFoundError:
        print(f"错误：找不到数据文件 {csv_path}")
        exit(1)
    except KeyError:
        print(f"错误：csv 里没有列名 {col_x} 或 {col_y}")
        exit(1)

    if len(xs) == 0:
        print("错误：csv 文件是空的")
        exit(1)

    try:
        n, mean_x, mean_y, r = calc_corr(xs, ys)
    except ValueError as e:
        print("错误：", e)
        exit(1)

    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)


if __name__ == "__main__":
    main()
