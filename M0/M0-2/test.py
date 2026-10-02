from legacy_corr import calc_corr

# 测试1：完全正相关
n, mx, my, r = calc_corr([1,2,3], [2,4,6])
assert abs(r - 1.0) < 1e-6, f"期望1.0，得到{r}"
print("测试1通过：完全正相关 r=1.0")

# 测试2：完全负相关
n, mx, my, r = calc_corr([1,2,3], [6,4,2])
assert abs(r - (-1.0)) < 1e-6, f"期望-1.0，得到{r}"
print("测试2通过：完全负相关 r=-1.0")

# 测试3：除零异常
try:
    calc_corr([5,5,5], [1,2,3])
    print("测试3失败：应该报错")
except ValueError:
    print("测试3通过：除零正确报错")
