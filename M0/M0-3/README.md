# M0-3 代码排查问题报告



## 修复的8个缺陷

| 序号 | 位置      | 现象            | 原因                 | 修复       | AI使用 |
| 1 | value变量名 | 变量名拼写不一致 | 原代码变量名大小写混乱 | 统一为value |
| 2 | OUTPUT_DIR="out" | 运行报PermissionError | out文件夹不存在 | 改为直接用OUTPUT_FILE | 豆包指出out目录问题并给出修改意见|
| 3 | acc += (v - mean) | std=-0.0000 | 偏差没平方正负抵消 | 改为(v-mean)**2 | 
| 4 | std = acc/len(data) | std值偏大 | 这是方差不是标准差 | 加上**0.5开根号 
| 5 | if v > mean+2*std | 只删大值没删小值 | 没判断偏小离群值 | 改为abs(v-mean)>2*std |
| 6 | for v in data: data.remove(v) | cleaned为空，输出cleaned的长度为0 | 边遍历边删且cleaned没赋值 | 改为往cleaned里append | 豆包指出并修改建议 |
| 7 | writer.writerow([v]) | csv只有一列 | 只写value没写time | 改为writerow([t,v]) | 豆包指出，给出修改建议 |
| 8 | 读文件没有try-except | 文件不存在崩Traceback | 没有异常处理 | 加try-except友好退出 | 豆包给了模板 |

另外：增加了显式的输入输出