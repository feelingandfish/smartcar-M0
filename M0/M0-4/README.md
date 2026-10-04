# M0-4 TaskScheduler 任务调度器

## 虚拟环境配置

uv venv
source .venv/bin/activate
uv pip install pyyaml

## 参数说明

- --config：指定任务配置文件路径，必须传
- --timeout：总超时时间，单位秒，不传就用配置文件里的
- --report：报告写到哪个文件，不传就叫report.json
- --seed：随机种子，传了每次跑结果一样，方便调试

## 运行示例

1. 基本运行
python3 scheduler.py --config tasks_demo.yaml --seed 42

2. 指定超时时间
python3 scheduler.py --config tasks_demo.yaml --seed 42 --timeout 10

3. 指定报告路径
python3 scheduler.py --config tasks_demo.yaml --seed 42 --report my_report.json

## 开发过程

1. 读取配置文件和命令行参数
   - 用argparse读命令行参数，用yaml/json读配置文件
   - AI使用：豆包给出代码

2. 拓扑排序确定执行顺序
   - 用DFS按依赖关系算出执行顺序，检测依赖成环
   - AI使用：豆包给出思路和代码

3. 按顺序执行任务并模拟成功失败
   - 按顺序执行每个任务，sleep模拟耗时，随机判定成功失败
   - AI使用：豆包给出基本结构

4. 失败重试和超时控制
   - 失败最多重试3次，依赖失败的任务标记SKIPPED，总超时停止
   - AI使用：豆包给出逻辑和代码

5. 修复timeout和skipped状态错误
   - 问题：超时后被覆盖成SKIPPED，3次都失败时KeyError
   - 解决：加if name not in results判断
   - AI使用：豆包指出原因代码

6. 彩色输出
   - 成功绿色、失败红色、重试黄色、跳过蓝色
   - 加了TTY判断，输出到文件时不显示颜色
   - AI使用：豆包给出颜色代码

7. 生成report.json
   - 记录每个任务状态、总耗时、是否超时
   - AI使用：豆包给出json结构

8. 异常处理
   - 文件不存在、YAML格式错误、空任务列表、依赖成环都友好报错
   - AI使用：豆包给出try-except模板
