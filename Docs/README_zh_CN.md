# 🦉 CAW-FOC

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/H2H3PQZVW)

## 🎮 项目地址

### 驱动器硬件

https://github.com/fake-rick/caw-drive

### FOC 控制算法

https://github.com/fake-rick/caw-foc

### 基于 Python & CAN 的总线控制

https://github.com/fake-rick/caw-bus-python

## 🎥 效果展示

[自制 FOC 驱动器 Python & CAN 总线控制](https://www.bilibili.com/video/BV1684y197R8)

## ⏰ TODO

| 功能             | 状态 |
| ---------------- | ---- |
| 开环速度控制     | ✔    |
| 闭环电流力矩控制 | ✔    |
| 闭环速度控制     | ✔    |
| 闭环位置控制     | ✔    |
| CAN 总线通讯     | ✔    |

## 🎮 配置 & 编译项目

### 安装 arm-none-eabi-gcc

**Windows**

- 下载 arm-none-eabi-gcc

  https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

- 设置环境变量

  将`gcc-arm-none-eabi\bin`的绝对路径添加进系统变量的 Path 中

**Mac**

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
```

### 编译

切换到项目根目录然后运行: `mingw32-make`，即可在项目根目录中的 build 目录中找到编译生成的文件[CawFOC.elf | CawFOC.bin | CawFOC.hex]。

# ☕ 支持

### 💰 微信 & 支付宝：

<img title="" src="https://github.com/fake-rick/caw-foc/blob/master/Docs/imgs/wxpay.jpg?raw=true" alt="https://github.com/fake-rick/caw-drive/blob/master/Docs/imgs/wxpay.jpg?raw=true" height="200"> <img src="https://github.com/fake-rick/caw-foc/blob/master/Docs/imgs/alipay.jpg?raw=true" title="" alt="https://github.com/fake-rick/caw-drive/blob/master/Docs/imgs/alipay.jpg?raw=true" height="200">

## 📚 参考资料

Park 变换：https://zhuanlan.zhihu.com/p/614244367

Clark 变换：https://zhuanlan.zhihu.com/p/613996592

SimpleFOC：https://github.com/simplefoc/Arduino-FOC
