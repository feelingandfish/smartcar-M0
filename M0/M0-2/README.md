# M0-2 首个代码工具

## 如何运行

python3 legacy_corr.py --config config.yaml

## 开发过程

1. fix: 修改相关系数公式
   - 豆包指出原代码分母 dx*dy 缺少开根号
   - 改为 (dx**0.5)*(dy**0.5)，自己跑了 python3 legacy_corr.py 验证，r 从 0.000156 变为 0.5635

2. feat: 增加读取文件的异常处理
   - 豆包给出 try-except 模板，自己逐行对照代码加了 FileNotFoundError 和 KeyError 处理
   - 验证：故意把 config.yaml 改成不存在的文件名，程序正确报错退出

3. refactor: 封装函数
   - 豆包给出函数结构，自己把原来的全局计算代码搬进 calc_corr 函数
   - 验证：封装后重新跑 python3 legacy_corr.py，结果仍然是 r=0.5635

4. feat: --config 参数设置
   - 豆包给出 argparse 用法，自己改了 CONFIG_PATH 的读取方式
   - 验证：分别跑 python3 legacy_corr.py 和 python3 legacy_corr.py --config config.yaml，结果都正确

5. docs: 增加注释
   - 豆包给出逐行注释版本，自己对照代码逐行确认注释是否准确
   - 验证：注释加完后重新跑程序，结果不变

6. test: 单元测试
   - 豆包给出 test.py 模板，自己新建 test.py 文件并运行
   - 验证：python3 test.py 三个测试全部通过

## 输入文件格式

### config.yaml
- input_csv：数据文件路径
- columns.x：第一列列名
- columns.y：第二列列名

### csv 文件
- 第一行是列名
- 必须包含 columns.x 和 columns.y 指定的两列
- 数据必须是数字

## 输出结果
- n：数据行数
- mean_x：x列平均值
- mean_y：y列平均值
- r：0.5635
- 在text中进行测试三次测试全部通过：完全正相关，完全负相关，除0报错
