# 导入需要的工具库
import argparse    # 处理命令行参数
import yaml        # 读yaml配置文件
import csv         # 读csv表格


def calc_corr(xs, ys):
    """计算皮尔逊相关系数"""
    n = len(xs)        # 数据个数

    sum_x = sum_y = 0.0   # 初始化两个求和变量
    for i in range(n):     # 循环每一行
        sum_x += xs[i]     # 累加x列
        sum_y += ys[i]     # 累加y列
    mean_x = sum_x / n     # x的平均值
    mean_y = sum_y / n     # y的平均值

    dx = dy = prod = 0.0   # 初始化三个求和变量
    for i in range(n):     # 再循环每一行
        a = xs[i] - mean_x    # x离平均值的差
        b = ys[i] - mean_y    # y离平均值的差
        dx += a * a           # x偏差平方累加
        dy += b * b           # y偏差平方累加
        prod += a * b         # 两偏差乘积累加

    # 相关系数公式：分子/分母，分母要开根号
    denom = (dx ** 0.5) * (dy ** 0.5)   # 分母
    if denom == 0:                       # 某列全相同则分母为0
        raise ValueError("某一列数据全部相同，无法计算相关系数")
    r = prod / denom                     # 相关系数
    return n, mean_x, mean_y, r          # 返回结果


def main():
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser()
    # 定义 --config 参数，默认值是 config.yaml
    parser.add_argument("--config", default="config.yaml")
    # 读取命令行输入
    args = parser.parse_args()

    # 打开配置文件
    try:
        with open(args.config) as f:
            cfg = yaml.safe_load(f)    # 读成字典
    except FileNotFoundError:          # 文件不存在时
        print(f"错误：找不到配置文件 {args.config}")
        exit(1)

    # 从配置字典里取值
    csv_path = cfg["input_csv"]          # csv文件路径
    col_x = cfg["columns"]["x"]          # x列列名
    col_y = cfg["columns"]["y"]          # y列列名

    xs = []    # 存x列数据
    ys = []    # 存y列数据

    # 打开csv文件逐行读
    try:
        with open(csv_path) as f:
            reader = csv.DictReader(f)    # 按列名读
            for row in reader:            # 逐行循环
                xs.append(float(row[col_x]))   # 取x列转数字存入
                ys.append(float(row[col_y]))   # 取y列转数字存入
    except FileNotFoundError:
        print(f"错误：找不到数据文件 {csv_path}")
        exit(1)
    except KeyError:    # 列名不存在时
        print(f"错误：csv 里没有列名 {col_x} 或 {col_y}")
        exit(1)

    # 空文件检查
    if len(xs) == 0:
        print("错误：csv 文件是空的")
        exit(1)

    # 调计算函数
    try:
        n, mean_x, mean_y, r = calc_corr(xs, ys)
    except ValueError as e:
        print("错误：", e)
        exit(1)

    # 打印结果
    print("n =", n)
    print("mean_x =", mean_x)
    print("mean_y =", mean_y)
    print("r =", r)


# 程序入口
if __name__ == "__main__":
    main()
