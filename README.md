# A133-副板 文档

## 基本功能:
- **RFID卡号识别**
- **M1卡号识别**
- **RF433呼叫铃识别**
- **温度采集**

---

## 简介
A133-副板 由Honokahqh设计,使用芯岭XL32F003 MCU

---

## 分区表

0x08000000-0x08010000为使用芯岭XL32F003的Flash范围，每128B为一页  
| Name        | Type | Offset | Size   |
|-------------|------|--------|--------|
| fac         | boot | 0x0000 | 0x8000 |
| app         | app  | 0x8000 | 0x7000 |
| flashdata   | data | 0xFC00 | 0x400  |

--- 

编译fac时,选择option->Target->IROM1 ,设置FLASH START = 0x08000000, SIZE = 0x8000,打开APP.h #define IsAPP 0  
编译fac时,选择option->Target->IROM1 ,设置FLASH START = 0x08008000, SIZE = 0x8000,打开APP.h #define IsAPP 1  
均编译并烧录后即可正常运行  

## 通讯格式

 - 帧头(1):0x88
 - 命令(1):指令参考cmd_process.h
 - 命令序列(1):对命令的扩充解释,例如1为发,2为ACK
 - 长度(1):仅数据部分
 - 数据(x):应小于133
 - CRC(2):modbus crc

## 版本更新
 
### 版本 1.0.0 (2024.4.5)