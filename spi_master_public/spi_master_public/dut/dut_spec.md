# spi_master_public DUT 规格书

## 1. 概述

`spi_master_public` 是一个 **全功能 SPI / SSP / Microwire 主机串行传输控制器**，通过 6-bit 字寻址寄存器总线进行配置。
- 支持四种帧格式：**SPI0**、**SPI1**、**SSP**、**Microwire**。
- 内置 TX/RX FIFO（8×32bit）、波特率分频器、传输控制状态机、收发移位寄存器、6 个中断源。
- Microwire 模式具备独立的控制帧（地址/命令帧）与数据帧分离的时序，并支持从机就绪握手。
- 配置完成后，只要满足启动条件，控制器**自动开始传输**，无需额外握手。
- 部分配置常量由评测端注入，不可通过寄存器修改（见 §4）。

## 2. 接口定义

### 2.1 顶层端口

| 信号 | 方向 | 位宽 | 说明 |
|------|------|------|------|
| `clk` | in | 1 | 系统时钟 |
| `rst_n` | in | 1 | 低有效复位 |
| `reg_we` | in | 1 | 寄存器写使能 |
| `reg_addr` | in | 6 | 寄存器字地址（见 §3 寄存器映射） |
| `reg_wdata` | in | 32 | 寄存器写数据 |
| `reg_re` | in | 1 | 寄存器读使能 |
| `rxd` | in | 1 | 串行接收数据输入 |
| `ss_in_n` | in | 1 | 从片选输入（传输期间必须保持正确极性，见 §5.3） |
| `reg_rdata` | out | 32 | 寄存器读数据 |
| `sclk_out` | out | 1 | 串行时钟输出 |
| `txd` | out | 1 | 串行发送数据输出 |
| `ss_n` | out | 4 | 从片选输出（SPI 低有效 / SSP 高有效） |
| `ssi_busy` | out | 1 | 传输忙指示（1=正在传输） |
| `ssi_txe_intr` ~ `ssi_mst_intr` | out | 6 | 中断输出（低有效） |

### 2.2 覆盖观测总线（Verilator public）

见 §8。所有 `cov_*` 信号仅供覆盖采样，不影响功能。

## 3. 寄存器映射（6-bit 字地址）

| 地址 | 名称 | 读写 | 位域 | 说明 |
|------|------|------|------|------|
| 0x00 | CTRLR0 | RW | [4:0]=DFS, [7:6]=FRF, [8]=SCPH, [9]=SCPOL, [11:10]=TMOD, [12]=SLV_OE, [13]=SRL_TEST, [14]=SS_TGL_EN, [19:16]=CFS | 控制寄存器 0 |
| 0x01 | CTRLR1 | RW | [15:0]=帧数-1 | 数据帧数 NDF = CTRLR1+1 |
| 0x02 | SSIENR | RW | [0]=SSI_EN | SSI 使能 |
| 0x03 | MWCR | RW | [2:0]=Microwire 控制（见 §3.1） | Microwire 控制寄存器 |
| 0x04 | SER | RW | [3:0]=从机使能 | 从片选位图 |
| 0x05 | BAUDR | RW | [15:0]=波特率分频 | sclk 分频系数（LSB 恒 0） |
| 0x06 | TXFTLR | RW | [3:0]=TX 阈值 | TX FIFO 水位阈值（复位值见 §4） |
| 0x07 | RXFTLR | RW | [3:0]=RX 阈值 | RX FIFO 水位阈值 |
| 0x08 | TXFLR | RO | [3:0]=TX FIFO 深度 | 0~8 |
| 0x09 | RXFLR | RO | [3:0]=RX FIFO 深度 | 0~8 |
| 0x0A | SR | RO | [0]=busy, [1]=TXF_NOT_FULL, [2]=TXE, [3]=RXF_NOT_EMPTY, [4]=RXF_FULL, [6]=DCOL | 状态寄存器 |
| 0x0B | IMR | RW | [5:0]=中断掩码 | 复位全 1（全部使能） |
| 0x0C | ISR | RO | [5:0]=中断状态（屏蔽后） | 读中断状态 |
| 0x0D | RISR | RO | [5:0]=中断原始状态 | 只读（清除方式见 §3.3） |
| 0x0E | TXOICR | RO | [0]=ISR[1] | 读该寄存器清除 TXO |
| 0x0F | RXOICR | RO | [0]=ISR[3] | 读该寄存器清除 RXO |
| 0x10 | RXUICR | RO | [0]=ISR[2] | 读该寄存器清除 RXU |
| 0x11 | MSTICR | RO | [0]=ISR[5] | 读该寄存器清除 MST |
| 0x12 | ICR | RO | [0]=ISR[5:3]\|ISR[2]\|ISR[1] | 读该寄存器清除 TXO/RXO/RXU/MST |
| 0x16 | IDR | RO | [31:0] | 模块标识（只读常量） |
| 0x17 | VIDR | RO | [31:0] | 版本号（只读常量） |
| 0x18~0x3B | DR | RW | [31:0] | 写=TX FIFO 压入，读=RX FIFO 弹出 |

### 3.1 关键字段

- **DFS**：数据帧长编码，字段值 = **帧长 - 1**（写 `DFS=7` → 8 位帧，`DFS=31` → 32 位帧）。有效帧长 4~32 位。
- **FRF**：帧格式，0=SPI（摩托罗拉），1=SSP（德州仪器），2=Microwire。
- **SCPH**：SPI 时钟相位，0=SPI0，1=SPI1。
- **SCPOL**：SPI 时钟极性。**SPI 模式下 sclk 空闲电平 = SCPOL**（SCPOL=0 → 空闲低，SCPOL=1 → 空闲高）。SSP/Microwire 模式下 sclk 空闲恒为低。
- **TMOD**：传输模式，0/1/2/3（行为差异见 §5.4）。
- **SRL_TEST**：回环使能，1=txd 直接环回作为接收数据。
- **SS_TGL_EN**：片选切换使能（帧间片选翻转）。
- **CFS**：**Microwire 控制帧长**，字段值 = 帧长 - 1。
- **MWCR**（Microwire 控制）：
  - `[0]=MW_MOD`：控制帧模式，0=连续数据模式，1=控制帧模式。
  - `[1]=MW_HS`：握手使能，1=控制帧与数据帧之间插入**从机就绪握手**（见 §5.5）。
  - `[2]=MW_DIR`：方向，0=写（数据从主机到从机），1=读（数据从从机到主机）。

### 3.2 寄存器访问规则

- 配置寄存器（CTRLR0/CTRLR1/MWCR/BAUDR）**仅在 SSIENR=0 时可写**。
- SER/TXFTLR/RXFTLR/IMR/SSIENR/DR 始终可访问。
- CTRLR0 写时拒绝 `FRF=11`（保留格式）与 `DFS<3` 的写入。
- BAUDR 写入 LSB 恒为 0（`wdata[7:1]→[7:1]`，`[0]=0`），故 BAUDR 只能为偶数。
- TXFTLR/RXFTLR 仅接受 ≤7 的值。
- DR 写 = TX FIFO 压入；DR 读 = RX FIFO 弹出（弹出数据在 `reg_rdata`）。
- TX FIFO 满时写被忽略；RX FIFO 空时读返回无效数据。

### 3.3 中断

| bit | 助记符 | 名称 | 触发条件 |
|-----|--------|------|----------|
| 0 | TXE | TX FIFO 空 | TX FIFO 变空 |
| 1 | TXO | TX FIFO 溢出 | TX FIFO 满时写入 |
| 2 | RXU | RX FIFO 下溢 | RX FIFO 空时读出 |
| 3 | RXO | RX FIFO 溢出 | RX FIFO 满时接收压入 |
| 4 | RXF | RX FIFO 满 | RX FIFO 变满 |
| 5 | MST | 主机竞争 | 检测到片选输入被外部拉低的竞争条件 |

- `ISR = RISR & IMR`。TXE/RXF 随 FIFO 状态硬件自动清除；TXO/RXO/RXU/MST 通过**读**对应 OICR/ICR 寄存器清除（ICR 一次清除 TXO/RXO/RXU/MST）。

## 4. 隐藏配置（评测端注入）

### 4.1 注入方式

`SPEC_CFS_MIN`、`SPEC_HOLD_SS`、`SPEC_TXFTLR_DFLT` 为顶层常量输入，由评测平台在综合/仿真时注入，**不可通过寄存器修改**。本地默认值与评测一致，保证本地可达性与评测一致。

### 4.2 隐藏参数说明

| 参数 | 位宽 | 说明 |
|------|------|------|
| `SPEC_CFS_MIN` | 4 | Microwire 控制帧长相关的隐藏配置。 |
| `SPEC_HOLD_SS` | 4 | 传输末段时序相关的隐藏配置。 |
| `SPEC_TXFTLR_DFLT` | 4 | TXFTLR 复位相关的隐藏配置。 |

## 5. 传输行为

### 5.1 传输启动条件（全部满足时在第一个 sclk 边沿启动）

1. `SSIENR=1`；
2. `SER != 0`（至少一个从机使能）；
3. TX FIFO 水位满足启动条件：`TMOD=2` 时仅需非空；其余模式需水位 `> TXFTLR` 阈值；
4. `ss_in_n` 极性正确（SPI/Microwire 模式 `ss_in_n=1`，SSP 模式 `ss_in_n=0`）。

**启动机制**：`SSIENR` 写 1 使串行核退出睡眠并重启，写 0 使其返回睡眠；`SSIENR` 边沿与上电复位一致，会同时清空 TX/RX FIFO。

### 5.2 传输阶段概览

一次传输由内部状态机调度，分为若干阶段：

- **SPI0/SPI1/SSP** 数据帧：片选选通 → 数据装载 → 逐位移位 → 结果捕获 → 帧间片选翻转 / 结束处理（时钟掩蔽、片选保持）→ 片选释放 → 空闲或睡眠。
- **Microwire**：先移出控制/地址字（`CFS` 位），再移出数据帧（`DFS` 位）；控制帧与数据帧之间的衔接方式、是否插入就绪握手，由 MWCR 决定（见 §5.5）。

### 5.3 从片选与 ss_in_n 约束

- **SPI0/SPI1**：`ss_n` 输出**低有效**（选中位输出 0）；传输期间必须保持 `ss_in_n=1`。
- **SSP**：`ss_n` 输出**高有效**；传输期间必须保持 `ss_in_n=0`。
- **Microwire**：`ss_n` 输出低有效；`ss_in_n` 保持 1。
- `SS_TGL_EN=1` 时帧间片选翻转。
- 数据帧结束后片选保持数个 sclk 半周期，随后释放。

### 5.4 传输模式（TMOD）差异

| TMOD | 名称 | 行为差异 |
|------|------|----------|
| 0 | EepromRead | 与 3 基本一致，接收使能 |
| 1 | EepromWrite | **接收禁止**（不捕获/压入接收数据） |
| 2 | TxOnly | 仅发送，按 NDF 帧数计数结束（仅非 Microwire 模式） |
| 3 | TxAndRx | 发送并接收，帧数由 FIFO 数据量决定 |

### 5.5 Microwire 控制帧与握手

- **控制帧**：每次 Microwire 传输先移出 `CFS` 位的控制/地址字，随后移出 `DFS` 位的数据帧。
- **MW_MOD=0（连续模式）**：控制字之后直接连续发送数据帧，数据帧间片选翻转；帧数由 FIFO 数据量或 NDF 决定。
- **MW_MOD=1（控制帧模式）**：控制字与数据帧分离，数据帧按 NDF 计数结束（MW_HS=0 时）。
- **MW_HS=1（握手模式）**：控制字发完后控制器按 `rxd` 的从机就绪信号衔接后续时序，再进入数据帧或下一控制帧。
- 控制帧长度由 CFS 字段解码得到。

### 5.6 波特率

- `BAUDR` 为分频系数，sclk 频率 = clk/BAUDR，sclk 周期 = BAUDR 个时钟周期。
- `BAUDR=0` 不产生有效时钟，无法启动传输。

### 5.7 传输结束

- 按配置计数发满帧数后结束传输（帧数来源随配置而异）。
- 传输完成后 `ssi_busy` 拉低，片选释放（经片选保持后）。

## 6. 动作空间（推理接口，12 维）

```
action = [reg_we, reg_addr, reg_wdata, reg_re, rxd, ss_in_n, rst_n,
          pad0, pad1, pad2, pad3, pad4]
```

| 字段 | 说明 |
|------|------|
| `reg_we` | 寄存器写使能（1=写） |
| `reg_addr` | 寄存器字地址（0~0x3B） |
| `reg_wdata` | 32 位写数据 |
| `reg_re` | 寄存器读使能（addr=DR 时弹出 RX FIFO） |
| `rxd` | 串行接收输入位（Microwire 握手时驱动从机就绪） |
| `ss_in_n` | 从片选输入（SPI/Microwire=1，SSP=0，见 §5.3） |
| `rst_n` | 复位（1=运行，0=复位） |
| `pad0..4` | 占位 |

寄存器访问规则见 §3.2，传输启动条件见 §5.1，传输结束见 §5.7。

## 7. 本地可见性分界

| 阶段 | 覆盖观测可见性 |
|------|----------------|
| 本地开发 | `cov_*` 全部可见，隐藏参数为本地默认值（与评测一致） |
| 评测阶段 | 推理镜像仅获得 bin 命中反馈（覆盖率向量），不暴露原始信号 |

## 8. 覆盖观测信号（cov_*）

| 信号 | 位宽 |
|------|------|
| `cov_fsm_state` / `cov_fsm_state_id` | 15/4 |
| `cov_sclk_re` / `cov_sclk_fe` / `cov_sclk_out` / `cov_sclk_mask` | 1 |
| `cov_bit_cnt` / `cov_ctrl_cnt` / `cov_frame_cnt` | 6/4/17 |
| `cov_f_done` / `cov_c_done` / `cov_x_done` / `cov_last_frame` | 1 |
| `cov_tx_shift_en` / `cov_rx_shift_en` / `cov_load_tx_shift` / `cov_s3` | 1 |
| `cov_txd` | 1 |
| `cov_ss_active` / `cov_ss_in_n` | 1 |
| `cov_dfs` / `cov_eff_dfs` | 5 |
| `cov_cfs` / `cov_eff_cfs` | 4 |
| `cov_baudr` / `cov_baud2` | 16/1 |
| `cov_ndf` | 17 |
| `cov_tmod` / `cov_frf` / `cov_scph` / `cov_scpol` | 2/2/1/1 |
| `cov_srl_test` / `cov_ss_tgl_en` | 1 |
| `cov_ser` | 4 |
| `cov_tx_level` / `cov_rx_level` | 4 |
| `cov_tx_full` / `cov_tx_empty` / `cov_rx_full` / `cov_rx_empty` | 1 |
| `cov_tx_overflow` / `cov_rx_overflow` / `cov_rx_underflow` | 1 |
| `cov_risr` / `cov_isr` / `cov_imr` | 6 |
| `cov_ssi_txe_intr` / `cov_ssi_mst_intr` | 1 |
| `cov_s0` / `cov_s1` / `cov_s2` | 1 |
| `cov_hold_ss_cnt` | 4 |
| `cov_start_xfer` / `cov_fsm_busy` / `cov_fsm_sleep` | 1 |
| `cov_ssi_busy` / `cov_ssi_sleep` | 1 |
| `cov_rx_push` / `cov_tx_pop` | 1 |
| `cov_rx_data` | 32 |
| `cov_mwcr` | 3 |
| `cov_txftlr` | 4 |
| `cov_load_tx_buffer` / `cov_load_rx_buf` / `cov_slv_oe` / `cov_multi_mst_sync` / `cov_spi0_control` / `cov_spi1_control` | 1 |

## 9. 覆盖点概览（120 bins）

覆盖报告按覆盖率向量返回每个 bin 的命中情况（1=已命中，0=未命中），共 120 个 bin。bin 语义覆盖以下类别，具体命中条件需结合 §3~§5 的传输行为推断：

- 基本功能
- 数据边界
- 条件组合
- 时序窗口
- 交叉

## 10. Verilator 兼容

- 覆盖观测总线标记 `/* verilator public */`。
- `verilator --cc --public` 编译；隐藏参数通过 `-GSPEC_CFS_MIN=... -GSPEC_HOLD_SS=... -GSPEC_TXFTLR_DFLT=...` 注入。
- 提供 `verilator_harness/cross_check.py` 逐 cycle 对拍（local_sim vs Verilator）。

---

**版本**：v1.1
**状态**：待评审
