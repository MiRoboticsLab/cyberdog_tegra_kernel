# Tegra Linux Kernel for CyberDog

[![License](https://img.shields.io/badge/License-GPL%202.0-brightgreen)](https://choosealicense.com/licenses/gpl-2.0/)

---
## **[简介]**

本项目为L4T的内核代码项目(适用于小米定制的英伟达应用板)。目前版本为Jetpack 4.5，内核版本为4.9.201。

## **[编译方法]**

cd到内核代码所在目录后并执行以下命令：

```
git clone https://github.com/MiRoboticsLab/cyberdog_tegra_kernel.git
cd cyberdog_tegra_kernel/kernel/kernel-4.9
./build.sh
```

build.sh中make的jobs数量(-jn)可根据个人电脑实际情况进行修改。
