# DA270

## 简介

本软件包是为 MiraMEMS DA270 加速度传感器提供的通用传感器驱动包。通过使用此软件包，开发者可以快速的利用 RT-Thread 将此传感器驱动起来。

本篇文档主要内容如下：

- 传感器介绍
- 支持情况
- 使用说明

## 传感器介绍

DA270 是 MiraMEMS（苏州明皜）公司专为可穿戴设备和 IOT 市场开发的一款超低功耗加速度传感器，尺寸小巧。它的功耗只需要 13μA，数据输出速率 1-1000HZ 可调。

## 支持情况

| 包含设备         | 加速度计 |
| ---------------- | -------- |
| **通讯接口**     |          |
| IIC              | √        |
| SPI              |          |
| **工作模式**     |          |
| 轮询             | √        |
| 中断             |          |
| FIFO             |          |
| **电源模式**     |          |
| 掉电             | √        |
| 低功耗           | √        |
| 普通             | √        |
| 高功耗           |          |
| **数据输出速率** | √        |
| **测量范围**     | √        |
| **自检**         |          |
| **多实例**       |          |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：DA270 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动框架支持；
- PIN 驱动：用于处理设备中断引脚；

### 获取软件包

使用 DA270 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] DA270: DA270 sensor driver package, support: accelerometer.
                [*]   Enable DA270 acce
                    Version (latest)  --->
```

**Enable DA270 acce**： 配置开启加速度计功能

**Version**：软件包版本选择

### 使用软件包

DA270 软件包初始化函数如下所示：

```
int rt_hw_da270_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 DA270 设备的注册；

#### 初始化示例

```
#include "sensor_mira_DA270.h"

int da270_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)DA270_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_da270_init("da270", &cfg);
    return 0;
}
INIT_APP_EXPORT(da270_port);
```

## 注意事项

暂无

## 联系人信息

维护人:

- [Zhou Dinghua](https://github.com/baozi119) 
