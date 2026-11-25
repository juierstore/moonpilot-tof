# NLink 协议与解析规则

本文记录 `tof_sense_ms` 实际使用的 NLink 帧格式、单位换算、校验和与容错
策略。协议来源：

- <https://support.nooploop.com/tofsense/protocol/>
- <https://support.nooploop.com/tofsense/example-code/>

默认串口配置 **921600 8N1**，模块主动上报，小端（little endian）。

---

## 1. 单点系列（TOFSense / -F / -S）

功能字 `0x00`，固定 **16 字节**：

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | `0x57` | 帧头 |
| 1 | 1 | `0x00` | 功能字（单点） |
| 2 | 1 | `0xFF` | 固定 |
| 3 | 1 | `id` | 模块 ID |
| 4 | 4 | `system_time` | 上电后毫秒 |
| 8 | 3 | `distance` | **有符号 24 位，单位 mm**，可为负（无效） |
| 11 | 1 | `dis_status` | `0` = 有效 |
| 12 | 2 | `signal_strength` | 信号强度 |
| 14 | 1 | `range_precision` | 精度（预留） |
| 15 | 1 | `checksum` | 校验和 |

---

## 2. 面阵系列（TOFSense-M / -MS）

功能字 `0x01`，帧长随区数变化：

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | `0x57` | 帧头 |
| 1 | 1 | `0x01` | 功能字（面阵） |
| 2 | 1 | `0xFF` | 固定 |
| 3 | 1 | `id` | 模块 ID |
| 4 | 4 | `system_time` | 上电后毫秒 |
| 8 | 1 | `zone_map` | 区数：`64` = 8×8，`16` = 4×4 |
| `9 + 6·i` | 3 | 第 i 区 `distance` | **无符号 24 位，单位 µm**（`492000` → 492 mm） |
| `12 + 6·i` | 1 | 第 i 区 `dis_status` | `0` = 有效，`0xFF` = 无效 |
| `13 + 6·i` | 2 | 第 i 区 `signal` | 信号强度 |
| … | 0 或 6 | 尾部填充 | 8×8 型号常见 6 字节 `0xFF` 尾部 |
| 末字节 | 1 | `checksum` | 校验和 |

长度关系：

```
无尾部: 9 + 6·zone_map + 1
有尾部: 9 + 6·zone_map + 6 + 1
8×8:    394 / 400 字节
4×4:    106 / 112 字节
```

解析器在 `[9 + 6·zone_map + 1, + 6]` 区间内逐字节尝试校验和，因此对
「有/无 6 字节尾部」两种固件都兼容。区域数量不在 `1..64` 时整帧丢弃并重新
同步。

---

## 3. 查询模式

模块可以配置成被动上报。查询帧 8 字节：

```
57 10 FF FF <id> FF FF <checksum>
```

当前驱动只接收主动上报，不实现查询模式。如需要，在 `Run()` 中定时
`write()` 该 8 字节即可。

---

## 4. 校验和

```
checksum = (sum of all bytes before the checksum) & 0xFF
```

单点帧校验前 15 字节，面阵帧校验到帧长 - 1。驱动对每帧都做校验，失败时
计入 `checksum errors` 并丢弃该帧。

---

## 5. 容错与重同步

解析器是一个四状态流式状态机：

```
Sync ──0x57──> Function ──0x00──> Payload(16)
                        └─0x01──> Header ──zone_map──> Payload(变长)
```

- **任意位置丢字节**：状态机在下一帧 `0x57` 处重新同步，不需要整帧对齐；
- **校验失败**：丢弃当前缓冲，若当前字节正好是 `0x57` 则立即作为新帧头；
- **截断帧**：驱动 1 s 收不到有效帧时计一次 `timeout`，并 `reset()` 解析器，
  避免半帧粘到下一帧前面；
- **缓冲越界**：写入前检查 400 字节上限，超限计 `length error` 并重同步。

错误分类统计（`status` 命令）：

| 计数 | 触发条件 |
|---|---|
| `header/length errors` | `0x57` 后功能字非法、`zone_map` 越界、缓冲越界 |
| `checksum errors` | 帧长已到但校验和不匹配 |
| `timeouts` | 超过 1 s 没有有效帧 |

---

## 6. 单位与过滤

| 帧类型 | 原始单位 | 解析器输出 |
|---|---|---|
| 单点 | mm，有符号 24 位 | `distance_mm`，可能为负 |
| 面阵 | µm，无符号 24 位 | `distance_mm`（内部 `/1000`） |

驱动聚合前先过滤：

1. `status == 0`；
2. `current_distance` 在 `[SENS_TOFMS_MIN, SENS_TOFMS_MAX]` 内；
3. 单点系列的盲区额外按 `max(SENS_TOFMS_MIN, 0.30 m)` 处理。

一帧中没有任何可用区时**不发布** `distance_sensor`，只累加对应计数器
（`no-valid-zone` / `out-of-range`），让 EKF2 自然超时，避免脏数据污染高度。
