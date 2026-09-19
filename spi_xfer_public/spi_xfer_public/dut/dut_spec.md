# spi_xfer_public DUT 规格书

## 1. 概述

`spi_xfer_public` 是一个 **SPI / SSP 主机串行传输控制器**，通过简单的字寻址寄存器总线进行配置。
- 支持三种帧格式：**SPI0**、**SPI1**、**SSP**。
- 内置 TX/RX FIFO（8×32bit）、波特率分频器、传输控制状态机、收发移位寄存器。
- 配置完成后，只要满足启动条件，控制器**自动开始传输**，无需额外握手。
- 部分配置常量由评测端注入，不可通过寄存器修改（见 §4）。

## 2. 接口定义

### 2.1 顶层端口

| 信号 | 方向 | 位宽 | 说明 |
|------|------|------|------|
| `clk` | in | 1 | 系统时钟 |
| `rst_n` | in | 1 | 低有效异步复位 |
| `reg_we` | in | 1 | 寄存器写使能 |
| `reg_addr` | in | 4 | 寄存器字地址（见 §3 寄存器映射） |
| `reg_wdata` | in | 32 | 寄存器写数据 |
| `reg_re` | in | 1 | 寄存器读使能 |
| `rxd` | in | 1 | 串行接收数据输入 |
| `ss_in_n` | in | 1 | 从片选输入（传输期间必须保持正确极性，见 §5.3） |
| `reg_rdata` | out | 32 | 寄存器读数据 |
| `sclk_out` | out | 1 | 串行时钟输出 |
| `txd` | out | 1 | 串行发送数据输出 |
| `ss_n` | out | 4 | 从片选输出（SPI 低有效 / SSP 高有效） |
| `ssi_busy` | out | 1 | 传输忙指示（1=正在传输） |

### 2.2 覆盖观测总线（Verilator public）

见 §8。所有 `cov_*` 信号仅供覆盖采样，不影响功能。

## 3. 寄存器映射（字地址）

| 地址 | 名称 | 读写 | 位域 | 说明 |
|------|------|------|------|------|
| 0x0 | CTRLR0 | RW | [4:0]=DFS, [6:5]=FRF, [7]=SCPH, [8]=SCPOL, [10:9]=TMOD, [11]=SRL_TEST, [12]=SSTE | 控制寄存器 0 |
| 0x1 | CTRLR1 | RW | [16:0]=NDF | 数据帧数 |
| 0x2 | SSIENR | RW | [0]=SSI_EN | SSI 使能 |
| 0x3 | SER | RW | [3:0]=从机使能 | 从片选位图 |
| 0x4 | BAUDR | RW | [15:0]=波特率分频 | sclk 分频系数 |
| 0x5 | TXFTLR | RW | [3:0]=TX 阈值 | TX FIFO 水位阈值 |
| 0x6 | TXFLR | RO | [3:0]=TX FIFO 深度 | 0~8 |
| 0x7 | RXFLR | RO | [3:0]=RX FIFO 深度 | 0~8 |
| 0x8 | SR | RO | [0]=busy, [1]=TXF_NOT_FULL, [2]=TXE, [3]=RXF_NOT_EMPTY, [4]=RXF_FULL | 状态寄存器 |
| 0x9 | DR | RW | [31:0] | 写=TX FIFO 压入，读=RX FIFO 弹出 |

### 3.1 关键字段

- **DFS**：数据帧长编码，字段值 = **帧长 - 1**（写 `DFS=7` → 8 位帧，`DFS=31` → 32 位帧）。
- **FRF**：帧格式，0=SPI（摩托罗拉），1=SSP（德州仪器）。
- **SCPH**：SPI 时钟相位，0=SPI0，1=SPI1。
- **SCPOL**：SPI 时钟极性。**sclk 空闲电平 = ~SCPOL**（SCPOL=0 → 空闲高，SCPOL=1 → 空闲低）。
- **TMOD**：传输模式，0/1/2/3（行为差异见 §5.4）。2=仅发送，3=发送并接收。
- **SRL_TEST**：回环使能，1=txd 直接环回作为接收数据。
- **SSTE**：片选切换使能（帧间片选翻转）。
- **NDF**：数据帧数，用于 TMOD=2 或（TMOD=3 且 FRF=0）的帧数计数。

### 3.2 寄存器访问规则

- 配置寄存器（CTRLR0/CTRLR1/SER/BAUDR/TXFTLR）**仅在 SSIENR=0 时可写**。
- SSIENR 与 DR 始终可访问。
- DR 写 = TX FIFO 压入（写入的数据即待发送数据）；DR 读 = RX FIFO 弹出。
- TX FIFO 满时写被忽略；RX FIFO 空时读返回无效数据。

## 4. 隐藏配置（评测端注入）

### 4.1 注入方式

`SPEC_DFS_MIN`、`SPEC_HOLD_SS` 为顶层常量输入，由评测平台在综合/仿真时注入，**不可通过寄存器修改**。本地默认值与评测一致，保证本地可达性与评测一致。

### 4.2 隐藏参数说明

| 参数 | 位宽 | 说明 |
|------|------|------|
| `SPEC_DFS_MIN` | 4 | 帧长相关的隐藏配置。 |
| `SPEC_HOLD_SS` | 4 | 传输结束时序相关的隐藏配置。 |

## 5. 传输行为

### 5.1 传输启动条件（全部满足时在第一个 sclk 边沿启动）

1. `SSIENR=1`；
2. `SER != 0`（至少一个从机使能）；
3. TX FIFO 水位超过 `TXFTLR` 阈值（`TMOD=2` 时仅需非空）；
4. `ss_in_n` 极性正确（SPI 模式 `ss_in_n=1`，SSP 模式 `ss_in_n=0`）。

### 5.2 传输过程

配置满足 §5.1 后传输自动开始，无需额外握手。传输期间：

- 选中从机的片选信号保持有效；
- 数据按所选帧格式逐位移出，同时逐位移入接收数据；
- 每帧结束自动衔接下一帧，直至本次传输结束（结束方式随传输模式与配置而异）；
- 帧末接收结果是否捕获并压入 RX FIFO 由传输模式决定（见 §5.4）。

传输结束后 `ssi_busy` 拉低，片选释放。

### 5.3 从片选与 ss_in_n

- **SPI0/SPI1**：`ss_n` 输出**低有效**（选中位输出 0）；`ss_in_n` 需保持高。
- **SSP**：`ss_n` 输出**高有效**；`ss_in_n` 需保持低。
- `SSTE=1` 使能帧间片选翻转（见 §3.1）。

### 5.4 传输模式（TMOD）差异

| TMOD | 名称 | 行为差异 |
|------|------|----------|
| 0 | EepromRead | 与 3 基本一致，接收使能 |
| 1 | EepromWrite | **接收禁止**（不捕获/压入接收数据） |
| 2 | TxOnly | 仅发送，按 NDF 帧数计数结束 |
| 3 | TxAndRx | 发送并接收，帧数由 FIFO 数据量决定（FRF=0 时按 NDF 计数） |

### 5.5 波特率

- `BAUDR` 为分频系数，sclk 频率 = clk/BAUDR。
- `BAUDR=0` 不产生有效时钟，无法启动传输。

### 5.6 传输结束

- 传输完成后 `ssi_busy` 拉低，片选释放。

## 6. 动作空间（推理接口，12 维）

```
action = [reg_we, reg_addr, reg_wdata, reg_re, rxd, ss_in_n, rst_n,
          pad0, pad1, pad2, pad3, pad4]
```

| 字段 | 说明 |
|------|------|
| `reg_we` | 寄存器写使能（1=写） |
| `reg_addr` | 寄存器字地址（0~9） |
| `reg_wdata` | 32 位写数据 |
| `reg_re` | 寄存器读使能（addr=9 时弹出 RX FIFO） |
| `rxd` | 串行接收输入位 |
| `ss_in_n` | 从片选输入（SPI=1，SSP=0，见 §5.3） |
| `rst_n` | 复位（1=运行，0=复位） |
| `pad0..4` | 占位 |

寄存器访问规则见 §3.2，传输启动条件见 §5.1，帧数结束判定见 §5.6。

## 7. 本地可见性分界

| 阶段 | 覆盖观测可见性 |
|------|----------------|
| 本地开发 | `cov_*` 全部可见，隐藏参数为本地默认值（与评测一致） |
| 评测阶段 | 推理镜像仅获得 bin 命中反馈（覆盖率向量），不暴露原始信号 |

## 8. 覆盖观测信号（cov_*）

| 信号 | 位宽 |
|------|------|
| `cov_fsm_state` / `cov_fsm_state_id` | 11/4 |
| `cov_sclk_re` / `cov_sclk_fe` | 1 |
| `cov_sclk_out` / `cov_sclk_mask` | 1 |
| `cov_bit_cnt` | 6 |
| `cov_frame_cnt` | 17 |
| `cov_x_done` | 1 |
| `cov_last_frame` | 1 |
| `cov_tx_shift_en` / `cov_rx_shift_en` | 1 |
| `cov_load_tx_shift` / `cov_load_rx_buf` | 1 |
| `cov_ss_active` | 1 |
| `cov_xfer_complete` | 1 |
| `cov_txd` | 1 |
| `cov_dfs` / `cov_eff_dfs` | 5/6 |
| `cov_baudr` / `cov_baud2` | 16/1 |
| `cov_ndf` | 17 |
| `cov_tmod` / `cov_frf` / `cov_scph` / `cov_scpol` | 2/2/1/1 |
| `cov_srl_test` / `cov_ss_tgl_en` | 1 |
| `cov_ser` | 4 |
| `cov_tx_level` / `cov_rx_level` | 4 |
| `cov_tx_empty` / `cov_tx_full` / `cov_rx_empty` / `cov_rx_full` | 1 |
| `cov_hold_ss_cnt` | 4 |
| `cov_s0` / `cov_s1` / `cov_s2` | 1 |
| `cov_rx_data` | 32 |
| `cov_ss_in_n` / `cov_start_xfer` | 1 |

## 9. 覆盖点概览（86 bins）

覆盖报告按覆盖率向量返回每个 bin 的命中情况（1=已命中，0=未命中），共 86 个 bin。bin 语义覆盖以下类别，具体命中条件需结合 §3~§5 的传输行为推断：

- 基本功能
- 数据边界
- 条件组合
- 时序窗口（含状态保持/跨阶段序列）
- 交叉

## 10. Verilator 兼容

- 覆盖观测总线标记 `/* verilator public */`。
- `verilator --cc --public` 编译；隐藏参数通过 `-GSPEC_DFS_MIN=... -GSPEC_HOLD_SS=...` 注入。
- 提供 `verilator_harness/cross_check.py` 逐 cycle 对拍（local_sim vs Verilator）。

---

**版本**：v1.1
**状态**：待评审
