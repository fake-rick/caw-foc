# 🦉 CAW-FOC

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/H2H3PQZVW)

## 🎮 Project URL

### Driver hardware

https://github.com/fake-rick/caw-drive

### FOC control algorithm

https://github.com/fake-rick/caw-foc

### Bus Control Based on Python & CAN

https://github.com/fake-rick/caw-bus-python

## 🎥 Video

[Homemade FOC (Field-Oriented Control) Driver: Python & CAN Bus Control](https://www.bilibili.com/video/BV1684y197R8)

## ⏰ TODO

| Functionality                      | Status |
| ---------------------------------- | ------ |
| Open-loop speed control            | ✔      |
| Closed-loop current/torque control | ✔      |
| Closed-loop speed control          | ✔      |
| Closed-loop position control       | ✔      |
| CAN bus communication              | ✔      |

## 🎮 Configure & Compile Project

### Install arm-none-eabi-gcc

**Windows**

- Download arm-none-eabi-gcc

  https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

- Set environment variables

  Add the absolute path of `gcc-arm-none-eabi\bin` to the Path variable in the system's environment variables.

**Mac**

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
```

### Compile

Switch to the project's root directory and run `mingw32-make`. The [CawFOC.elf | CawFOC.bin | CawFOC.hex] files will appear in the build directory.

# ☕ Support

### 💰 WeChat Pay & Alipay：

<img title="" src="https://github.com/fake-rick/caw-foc/blob/master/Docs/imgs/wxpay.jpg?raw=true" alt="https://github.com/fake-rick/caw-drive/blob/master/Docs/imgs/wxpay.jpg?raw=true" height="200"> <img src="https://github.com/fake-rick/caw-foc/blob/master/Docs/imgs/alipay.jpg?raw=true" title="" alt="https://github.com/fake-rick/caw-drive/blob/master/Docs/imgs/alipay.jpg?raw=true" height="200">

## 📚 References

Park Transformation：https://zhuanlan.zhihu.com/p/614244367

Clark Transformation：https://zhuanlan.zhihu.com/p/613996592

SimpleFOC：https://github.com/simplefoc/Arduino-FOC
