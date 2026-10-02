# M0-1 环境搭建记录

## 第1步：安装 WSL
- 装了什么：WSL 2.7.14
- 怎么装：从 GitHub microsoft/WSL releases 下载 wsl.2.7.14.0.x64.msi，双击安装
- 参考资料：b站有博主教学https://www.bilibili.com/video/BV1y8VJ6hEAs/?spm_id_from=333.1391.0.0&vd_source=3e422c768083ef6f3eb1b22349226e74
GitHub上面找安装包：https://github.com/microsoft/WSL/releases

## 第2步：安装 Ubuntu 22.04
- 装了什么：Ubuntu 22.04.5
- 怎么装：从 Ubuntu 官方镜像站下载 ubuntu-22.04.5-wsl-amd64.wsl，双击安装
- 参考资料：b站同上面的博主
   在清华镜像网站下载安装包
   豆包给的命令
- 遇到的问题：首次创建用户时输入密码看不见，以为卡死了；原因是 Linux 密码输入就是不显示的，盲打回车即可

## 第3步：换清华源
- 做了什么：把 apt 源换成清华镜像，加速下载
- 参考资料：豆包给的 sed 命令
- 遇到的问题：最开始改 /etc/apt/sources.list.d/ubuntu.sources 报错文件不存在；Ubuntu 22.04 的源文件在 /etc/apt/sources.list，改对路径后成功

## 第4步：装基础工具
- 装了什么：git、build-essential（gcc/g++）、cmake、python3-pip、curl
- 怎么装：sudo apt install

## 第5步：配置 git
- 做了什么：设置 user.name=feelingandfish，user.email=193863997@qq.com
- 参考资料：豆包给的 git config 命令

## 第6步：安装 uv
- 装了什么：uv 0.12.21
- 怎么装：curl -LsSf https://astral.sh/uv/install.sh | sh
- 参考资料：豆包给的命令
- 遇到的问题：复制命令后没反应，等了一会才开始下载；装完需要 source ~/.local/bin/env 才能用

## 第7步：安装 ROS2 Humble
- 装了什么：ROS2 Humble
- 怎么装：设置 locale → 添加 universe 源 → 换清华 ROS2 镜像 → sudo apt install ros-humble-desktop → source setup.bash 写入 .bashrc
- 参考资料：豆包给的命令 + 清华镜像源教程
- 遇到的问题：下载量大（几个 G），花了比较久；无报错

## 第8步：安装 VS Code 和扩展
- 装了什么：VS Code Windows 版 + WSL/Python/Pylance/Remote-SSH 扩展
- 参考资料：豆包给的步骤
- 遇到的问题：
  - code . 命令找不到：wsl --shutdown 重启后恢复
  - C/C++ 扩展（269MB）下载极慢：WSL NAT 模式走不到 Windows 加速器代理，暂时放弃，不影响基本使用

## 第9步：配置 SSH 密钥
- 做了什么：ssh-keygen 生成 ed25519 密钥，公钥贴到 GitHub，ssh -T git@github.com 验证通过
- 参考资料：豆包给的步骤

