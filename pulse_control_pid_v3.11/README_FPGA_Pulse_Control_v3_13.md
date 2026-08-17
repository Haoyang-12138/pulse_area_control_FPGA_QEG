# Red Pitaya FPGA Pulse-Control Project — v3.13 External Trigger + Q31 Ki Candidate

> **Status:** source package generated from the physically verified v3.12 baseline on 2026-08-12.  
> **Important:** this v3.13 source has **not** yet been synthesized/implemented in Vivado or verified on the Red Pitaya. Do not call it timing-clean or hardware-verified until the regression sequence below passes.  
> The design intentionally preserves the v3.12 ADC measurement, FIFO formats, DAC2 physical output pipeline, safe `/dev/mem` 32-bit MMIO access, and multi-cycle PID architecture.

## A. Exactly what changes in v3.13

### A1. External DIO0_P group trigger

Normal group source is now:

```text
external experimental trigger -> DIO0_P
        ↓
200 MHz 2-FF synchronizer
        ↓
programmable delay, 5 ns/cycle
        ↓
toggle event
        ↓
125 MHz 2-FF event CDC
        ↓
group aggregator
```

`DIO1_P` remains the verified programmable 200 MHz GPIO output and `DIO2_P` remains the debug/latency marker. The old **internal GPIO-period -> ADC-domain group-boundary CDC is removed from the active grouping path**. The GPIO generator itself is otherwise unchanged.

Group-source register semantics at `0x038` are now:

```text
0 = external DIO0_P trigger
1 = DAC2 period boundary (debug/fallback)
2 = manual (safe idle/configuration)
```

`0x03C` is repurposed from the old 125 MHz `GROUP_BOUNDARY_DELAY_SAMPLES` to:

```text
EXTERNAL_TRIGGER_DELAY_CYCLES
clock = 200 MHz
resolution = 5 ns
```

The first delayed external trigger **starts group 0**. It does not close/log an implicit pre-trigger group. Every following delayed trigger closes the current group and starts the next group.

The ADC pulse detector itself remains continuously enabled during an acquisition session. It is **not** toggled on/off for every trigger, because doing so would clear/re-prime the rolling baseline and can lose the first pulse. Python primes the detector for 1 ms while group source is manual, then arms the external source.

The trigger front-end is enabled only when:

```text
measurement_enable = 1
AND
group_source = external
```

so triggers seen during idle/configuration cannot leave a delayed event pending for the next session.

The trigger delay front-end has one pending delay counter. A new trigger arriving before the previous delay expires is dropped and sets a sticky overrun flag. The supplied Python configuration therefore enforces:

```text
trigger_delay < one expected trigger period
```

At 1 MHz this means `EXTERNAL_TRIGGER_DELAY_NS < 1000 ns`.

The programmable component has 5 ns resolution, but the absolute DIO0_P -> group-boundary latency also contains fixed asynchronous synchronizer/event-CDC latency and the 125 MHz group logic. Calibrate the required delay on the scope; do not interpret the parameter as the entire physical path latency.

### A2. Ki numerical fix

Kp and Kd stay in the verified Q16.16 implementation. Ki at register `0x054` changes semantics to **non-negative Q1.31 per valid group update**:

```text
ki_raw = round(KI_PER_S * dt_s * 2^31)
```

The register-map version is therefore bumped from:

```text
PC03 = 0x50433033
```

to:

```text
PC04 = 0x50433034
```

The old implementation did:

```text
error * Ki(Q16.16)
        ↓
>>> 16 on every update
        ↓
integer integral accumulator
```

which discarded fractional increments before accumulation and produced the observed positive/negative asymmetry for small Ki.

The new implementation does:

```text
error * Ki(Q1.31)
        ↓
NO shift
        ↓
64-bit Q31 integral accumulator
        ↓
clamp in Q31 domain
        ↓
symmetric round-to-nearest
        ↓
32-bit integer I area term
        ↓
existing P + I + D / area-to-DAC / DAC2 path
```

The anti-windup decision is preserved. If the candidate correction exceeds physical DAC bounds, the old Q31 integral state is kept exactly as the old integer integrator was kept.

To protect timing closure, the change adds explicit pipeline states rather than combining more arithmetic in one 125 MHz cycle:

```text
PI_INTEGRAL  -> full Q31 wide add
PI_I_RANGE   -> guard-bit overflow classification
PI_I_CLAMP   -> 64-bit clamp compare
PI_I_ROUND   -> registered integer I term
PI_SUM       -> existing P + I + D path
```

No attempt is made to collapse the PID back into one cycle.

### A3. Registers added for trigger diagnostics

```text
0x388 EXTERNAL_TRIGGER_COUNT
0x38C EXTERNAL_TRIGGER_ACTIVE_DELAY_CYCLES
STATUS bit 24 = external_trigger_overrun_seen
```

The trigger count is a diagnostic counter of delayed trigger events reaching the 125 MHz domain. The sticky overrun flag is cleared by the existing `clear-flags` command.

## B. Files that are intentionally unchanged from the verified v3.12 source

These are copied byte-for-byte from the uploaded working set:

```text
gpio_sequence_generator_v3.sv
pulse_debug_marker_v3.sv
pulse_measurement_v3.sv
pulse_sync_fifo_v3.sv
dac2_sequence_generator_v3.sv
```

In particular, **do not casually edit `dac2_sequence_generator_v3.sv`**: its registered waveform arithmetic and period-boundary correction application are part of the v3.12 physical OUT2 fix.

## C. Files changed in v3.13

```text
external_group_trigger_v3.sv        NEW
pulse_area_pid_controller_v3.sv     Ki Q1.31 + fractional accumulator
pulse_group_aggregator_v3.sv        first external trigger starts group 0
pulse_control_regs_v3.sv            PC04 + trigger registers + Ki semantics
red_pitaya_top.sv                    DIO0_P trigger path; remove GPIO group CDC
pulse_control_cdc_v3.xdc             async DIO0_P + existing clock groups
pulse_control.py                     PC04, safe trigger config, Q1.31 Ki
run_pulse_control.py                 external trigger defaults; DAC2=1 MHz
```

## D. Required Vivado/hardware regression before calling this version working

1. **Compile/source completeness**: add all `.sv` files in this directory, including the new `external_group_trigger_v3.sv` and `dac2_sequence_generator_v3.sv`.
2. **Timing**: post-route `WNS >= 0`, `TNS = 0`, hold clean. Do not accept a new timing violation merely because PID latency is still small.
3. **PC04 MMIO**: `show`, `clear-state`, `clear-flags`, `clear-fifos`; no Bus error. Keep single aligned 32-bit `/dev/mem` transactions.
4. **DAC2 physical regression**: run `dac2-dc` and `dac2-square`; verify OUT2 on the scope before testing PI. This checks that the v3.12 physical path was not disturbed.
5. **Idle FIFO**: measurement OFF, feedback OFF, group source manual; FIFOs remain at zero even if external DIO0_P triggers continue.
6. **External trigger**: inject known DIO0_P triggers; verify `external_trigger_count` increments and DIO2/group behavior follows the delayed trigger. Test at delay 0, 5 ns, 20 ns, etc.
7. **Trigger overrun**: intentionally configure delay >= trigger period only in a controlled test (hardware-side/manual register write, because Python blocks it) and verify status bit 24 catches the dropped-trigger condition.
8. **First-trigger semantics**: clear state, arm external source, confirm the first trigger starts group 0 and does not produce an empty pre-trigger group.
9. **Measurement regression**: baseline, duration, area, peak, invalid reasons match v3.12 behavior.
10. **P-only**: `KI_PER_S=0`, `KD=0`; verify the previously working correction sign/direction.
11. **Ki symmetry simulation/hardware test**: symmetric `+E, -E, +E, -E...` must show no systematic I drift.
12. **Ki constant-error test**: constant `+E` and `-E` must produce equal-magnitude opposite I slopes.
13. **Small Ki**: verify e.g. `KI_PER_S = 1, 2, 5 /s` are non-zero and accumulate correctly at `controller_dt_s = 1 us`.
14. **Full PI**: only after all above pass.

## E. Default high-level configuration in this package

```text
EXTERNAL_TRIGGER_FREQUENCY_HZ = 1 MHz
EXTERNAL_TRIGGER_DELAY_NS      = 0 ns
GROUP_SOURCE                   = external
DAC2_FREQUENCY_HZ              = 1 MHz
KI_PER_S                       = 0 initially
```

The external trigger frequency is not generated by the FPGA. It is supplied to Python so Ki/Kd can use the correct `dt` and so the trigger-delay overlap guard can be checked.

**Important phase note:** setting the external trigger and DAC2 both to 1 MHz aligns their nominal repetition rates but does **not** phase-lock them. `dac2_sequence_generator_v3.sv` intentionally keeps the verified v3.12 behaviour in which `correction_pending` becomes physically active only at a DAC2 period boundary. On first hardware bring-up, scope DIO0_P, DIO2_P, OUT2 and the photodiode together and verify the relative phase does not create an unwanted one-period correction ambiguity. Do not modify the verified DAC2 pipeline unless this test demonstrates a real problem.

**Trigger pulse-width note:** DIO0_P is asynchronously sampled at 200 MHz. Use a clean external trigger HIGH width of at least 10 ns unless a separate pulse-stretcher is deliberately added and verified.

---

# Historical v3.12 debugging/design log (preserved from the uploaded working set)

# Red Pitaya FPGA Pulse-Control Project — Debugging & Design History

> 项目：Quantum-dot optical pulse real-time measurement / feedback  
> 平台：Red Pitaya STEMlab 125-14 / Zynq-7010，Vivado 2025.1  
> 当前稳定 FPGA 版本：**v3.12**  
> 当前软件接口：Python 直接 `/dev/mem` 访问 AXI-Lite registers  
> 主要数据单位：FPGA 内部使用 **ADC counts × samples**；Python/CSV 层再换算到 V、µs、V·µs

---

# 1. 项目最终目标

这个 FPGA 系统的目标不是简单生成波形，而是形成完整的 optical-pulse control chain：

```text
GPIO / experimental sequence
        ↓
ADC IN1 measures photodiode signal
        ↓
detect individual optical pulses
        ↓
extract:
    baseline
    duration
    pulse height
    peak height
    integrated pulse area
        ↓
group pulses according to experimental sequence
        ↓
compare measured group pulse area with target
        ↓
P / PI / PID feedback
        ↓
modify DAC2 output
        ↓
actuator changes optical pulse amplitude
```

当前实验里：

- GPIO sequence 可以运行到约 1 MHz repetition rate；
- ADC 采样率为 125 MS/s；
- 1 µs optical pulse 对应约 125 ADC samples；
- FPGA 内部 measurement / feedback 可以每一个 pulse/group 实时运行；
- Python 不需要记录所有 1 MHz pulse；
- 当前 logging 需求是例如 **每 500 ms 抽取一个 pulse snapshot 写入 CSV**。

---

# 2. FPGA 架构概览

## 2.1 Clock domains

主要 clock：

```text
ADC clock       = 125 MHz
PS FCLK0        = 125 MHz
PS FCLK3        = 200 MHz
DAC clock       = 2 × 250 MHz style Red Pitaya DAC interface
```

当前设计中：

- GPIO sequence generator：200 MHz
- ADC measurement / group / PID：125 MHz
- DAC2 sequence / actuator path：125 MHz logic → Red Pitaya DAC output chain

---

## 2.2 I/O

当前 custom firmware：

```text
IN1      → ADC pulse measurement
DIO1_P   → fast GPIO experimental/group marker output
DIO2_P   → debug latency marker
OUT2     → programmable DAC waveform / PID actuator
OUT1     → unused / zero
```

---

# 3. 第一阶段：bitstream 生成过程中遇到的 timing 问题

这是整个项目最早、也是最严重的一类问题。

最初的设计把 pulse measurement、PI/PID arithmetic、GPIO、DAC 等逻辑都加入到 Red Pitaya design 后，Vivado implementation 出现严重 timing violation。

早期典型结果：

```text
WNS ≈ -7.150 ns
TNS ≈ -31336 ns
failing endpoints ≈ 18685
```

另外一次早期版本也出现过：

```text
WNS ≈ -5.610 ns
WHS ≈ +0.015 ns
WPWS ≈ +0.750 ns
```

这说明问题主要不是 hold timing，而是 **setup timing**。

---

## 3.1 为什么 timing 会失败

最开始一个核心错误是：

> 尝试在一个 125 MHz clock cycle 内完成过多 pulse-area / PID arithmetic。

125 MHz clock period：

```text
8 ns
```

如果一个 cycle 内同时做：

```text
error calculation
× Kp
× Ki
× Kd
integrator
correction scaling
limit
DAC conversion
```

乘法器、加法器、比较器和组合逻辑串起来以后，critical path 会远远超过 8 ns。

尤其 fixed-point multiplication 和宽位宽 accumulation 很容易形成长 combinational path。

---

## 3.2 解决方法：multi-cycle FSM / pipelining

后来设计原则固定为：

> **不能为了“低 latency”把全部 arithmetic 塞到一个 cycle。**

正确方法是将 feedback calculation 拆成多级 pipeline，例如：

```text
cycle 1:
    latch measured area
    calculate error

cycle 2:
    P multiply

cycle 3:
    I multiply / accumulator preparation

cycle 4:
    D term

cycle 5:
    sum P + I + D

cycle 6:
    area-to-DAC scaling

cycle 7:
    correction sign / limit

...
```

最终 v3.10 左右的 PID pipeline latency 约：

```text
~14 ADC cycles
≈ 14 × 8 ns
≈ 112 ns
```

对于实验目标 `< 1 ms` 来说，这个 latency 已经远远足够快，因此没有必要为了少几十 ns 去牺牲 timing closure。

---

## 3.3 删除不需要的 stock Red Pitaya logic

为了降低 routing congestion 和 timing pressure：

### v3.4
移除了 stock ASG（arbitrary signal generator）。

### v3.10
进一步移除了 stock scope path。

原因是 custom firmware 已经用于专门的 pulse-control task，不再需要完整保留 Red Pitaya 原厂 oscilloscope / ASG architecture。

这样可以：

- 减少 LUT / FF / routing usage；
- 缩短 critical paths；
- 降低 clock-domain complexity；
- 增加 timing closure 概率。

---

## 3.4 最终 timing closure

v3.10 在 post-route 后执行：

```tcl
phys_opt_design -directive AggressiveExplore
```

最终得到：

```text
WNS = +0.005 ns
TNS = 0
failing endpoints = 0
```

hold timing 也 clean。

这个 margin 非常薄，但已经 technically timing clean。

后续 v3.11 因 DAC output 修复再次引入 timing regression：

```text
WNS ≈ -0.512 ns
```

执行 `phys_opt_design` 后改善到：

```text
WNS ≈ -0.231 ns
```

仍未 closure。

于是 v3.12 又增加 DAC path register/pipeline，最终再次通过 timing。

---

# 4. 第二阶段：bitstream 生成后 OUT2 没有真正输出

这是 timing closure 之后遇到的第二个主要硬件问题。

## 4.1 症状

软件 register readback 显示：

```text
requested DAC high value 改变
effective high value 改变
PID correction register 改变
```

例如：

```text
3277 counts → 5734 counts
```

看起来数字逻辑完全正常。

但是示波器上：

```text
OUT2 ≈ 固定在中点电压
```

没有按照 requested DAC waveform 变化。

用户的 Red Pitaya fast output 做过 0–2 V style modification，因此：

```text
digital zero
≈ physical 1 V
```

所以示波器看到约 1 V 并不代表 DAC command 正确，而可能只是 DAC 数字输入一直停在 zero-code。

---

## 4.2 首先排除硬件故障

为了判断到底是 FPGA 还是 Red Pitaya analog output hardware：

重新加载 stock Red Pitaya bitstream：

```bash
/opt/redpitaya/fpga/fpga_0.94.bit
```

然后用 stock signal generator：

```bash
generate 2 0.2 1000 sine
generate 2 0.8 1000 sine
```

OUT2 能正常改变。

因此确认：

> **DAC hardware / analog front-end 本身没有坏。**

问题一定在 custom FPGA DAC path。

---

## 4.3 还排除了 digital loopback

检查 housekeeping digital loopback register：

```bash
monitor 0x4000000C
```

结果：

```text
0x00000000
```

因此不是 Red Pitaya internal loopback / alternate source 抢占 DAC output。

---

## 4.4 根本原因与修复

问题最终定位在：

> custom DAC2 logic 虽然内部 requested/effective values 正常，但没有可靠地进入最终 Red Pitaya DAC output register/interface。

v3.11/v3.12 重新整理了 DAC2 output pipeline：

```text
DAC sequence/PID logic
        ↓
registered DAC command
        ↓
Red Pitaya DAC interface
        ↓
OUT2
```

并增加足够 pipeline 避免 timing regression。

---

## 4.5 v3.12 的验证

加载 v3.12 后进行直接 DAC test：

```bash
python3 run_pulse_control.py dac2-dc --counts 0
python3 run_pulse_control.py dac2-dc --counts 4096
```

以及 square-wave test。

示波器上 OUT2 能正确响应，因此确定：

```text
v3.12 DAC2 physical output path = FIXED
```

从这一步之后，如果没有新的证据，不应再修改 DAC HDL。

---

# 5. 第三阶段：FIFO 问题

FIFO 是整个项目后来最容易混淆的一部分，因为其实出现过 **两种不同性质的问题**：

1. FIFO 在 measurement OFF 时仍不断填满；
2. Python 读取 FIFO 时的 MMIO 方法导致 Bus error。

---

# 6. FIFO 问题 A：measurement OFF，但 group FIFO 仍然增长

## 6.1 症状

即使：

```text
measurement = OFF
feedback = OFF
```

运行一段时间后：

```text
group FIFO level rapidly increases
```

最终 overflow。

最初容易误以为：

> measurement module 关闭后 FIFO 应该完全停止产生数据。

但后来发现这并不正确。

---

## 6.2 根本原因：group boundary 和 measurement enable 是两件不同的事

当时配置：

```text
GROUP_SOURCE = dac2
```

而 DAC2 waveform generator 即使 measurement OFF，仍然持续运行。

因此每次 DAC2 period boundary 都会：

```text
close current group
→ generate group record
→ push group FIFO
```

即使这个 group 内：

```text
valid pulse count = 0
```

仍然会产生 empty group。

所以：

```text
measurement OFF
≠
group generation OFF
```

这是非常关键的架构认识。

---

## 6.3 实验验证

手动设置：

```bash
monitor 0x40700038 0x2
```

其中 group source：

```text
0 = GPIO
1 = DAC2
2 = manual
```

设置成：

```text
GROUP_SOURCE = manual
```

之后：

- clear FIFO；
- clear flags；
- 等待数秒；

观察到：

```text
group FIFO no longer increases
```

这直接证明：

> FIFO 空转填满的根本原因是 autonomous group boundary source，而不是 measurement detector。

---

## 6.4 最终 software policy

最终 Python runtime 固定成：

### idle 状态

```text
measurement OFF
feedback OFF
group source = manual
```

因此 FPGA 不会自己产生 group record。

### active measurement

运行：

```bash
python3 run_pulse_control.py sample ...
```

或其他 active acquisition command 时：

```text
group source → configured source (normally GPIO)
measurement → ON
optional feedback → ON
```

运行结束后自动：

```text
feedback → OFF
measurement → OFF
group source → manual
```

这样不会因为用户忘记关 group source 导致 FIFO overnight overflow。

---

# 7. Group source 的最终定义

真实实验中：

```text
GROUP_SOURCE = GPIO
```

而不是 DAC2。

因为 experimental sequence 的 repetition period 是由 GPIO sequence 定义的。

当前实验例子：

```text
GPIO frequency = 1 MHz
period = 1 µs
```

因此：

```text
1 GPIO period = 1 experimental group
```

注意：

> group period 和 optical pulse duration 是不同概念。

当前实验可能恰好也是 µs scale，但逻辑上必须区分。

---

# 8. FIFO 问题 B：Python `/dev/mem` 读取出现 Bus error

## 8.1 症状

一些 Python 版本运行：

```text
feedback-off
clear-fifos
clear-flags
```

这样非常简单的 register command 都可能直接出现：

```text
Bus error
```

因此问题不是 FIFO data volume，而是 MMIO 访问方式。

---

## 8.2 错误做法

曾尝试直接对 `mmap` 使用：

```python
struct.unpack_from(...)
struct.pack_into(...)
```

或者：

```text
一次 bulk read 多个 32-bit words
```

这些方法对普通 RAM 没问题，但 `/dev/mem` 映射的 AXI-Lite MMIO region 并不是普通内存。

结果导致 SIGBUS / Bus error。

---

## 8.3 最终安全 MMIO primitive

目前确定最稳定的方法是：

```python
mmap.seek(offset)
data = mmap.read(4)
```

写：

```python
mmap.seek(offset)
mmap.write(struct.pack("<I", value))
```

即：

> **每一次 MMIO transaction 只访问一个 aligned 32-bit register。**

FIFO record 也不要一次 bulk read，而是：

```text
read word 0
read word 1
read word 2
...
```

全部使用 safe 4-byte transaction。

这是当前 canonical low-level access 方法。

---

# 9. FIFO throughput 与最终 logging 策略

最开始曾考虑：

> 是否需要实时把 1 MHz 下所有 pulse 都写到 CSV？

后来明确真实需求并不是这样。

当前目标：

```text
FPGA:
    每个 1 µs group 都实时 measurement / feedback

Python:
    例如每 500 ms 抽取一个 pulse snapshot
    写入 CSV
```

因此：

```text
FPGA realtime rate = 1 MHz
CSV logging rate   = 2 Hz
```

Python 不需要持续搬运全部 pulse data。

这避免：

- 1 MHz Python MMIO bottleneck；
- 巨量 CSV；
- DMA / AXI Stream 复杂化。

如果未来真的需要 continuous 1 MHz raw recording，则应该改成：

```text
FPGA → AXI Stream → DMA → DDR
```

而不是继续扩大 register FIFO。

---

# 10. 当前 snapshot logging 的注意事项

当前 CSV 中：

```text
pulse_area
pulse_duration
baseline
peak
...
```

来自 pulse FIFO record，可以认为是同一个 pulse 的 coherent data。

但：

```text
live_error
live_P
live_I
live_D
```

是 Python 之后读取 live registers 得到的。

由于 FPGA 1 MHz 更新，而 ARM `/dev/mem` 是逐 register 读取：

```text
error
P
I
D
```

这些 live values **不保证严格来自同一个 PID update**。

因此：

> CSV snapshot 很适合看长期趋势，但不能仅根据某一行 `live_error` 与 `live_I` 的瞬时关系判断一个 clock 内 PID arithmetic 的符号。

---

# 11. 当前反馈控制：P-only 工作正常

实验已经验证：

```text
P-only feedback works
```

也就是说：

```text
error sign             correct
DAC correction sign    correct
plant response          correct direction
P fixed-point scaling   usable
```

例如：

```text
area > target
→ P correction makes DAC move in direction that decreases area
```

因此当前 feedback chain 的基本 sign convention 没问题。

---

# 12. 当前最大新问题：Ki / integral implementation

这是目前最新发现、也是下一版 FPGA 最值得修复的问题。

## 12.1 当前 FPGA pulse-area 单位其实已经正确

FPGA 内部 pulse area 不是 V·µs。

它实际计算：

```text
pulse area = Σ(ADC_sample - baseline)
```

所以单位是：

```text
ADC counts × ADC samples
```

由于 ADC：

```text
125 MHz
→ 1 sample = 8 ns
```

Python 最后才换算：

```text
counts × samples
→ V × µs
```

因此：

> **问题不是 FPGA 使用了 V·µs。**

FPGA 用整数 counts×samples 本身是正确设计。

---

# 13. Ki 的真正数值问题：过早截断 fractional contribution

当前 integral implementation 的概念大致是：

```systemverilog
i_product = error * ki_q16_per_update;

integral_accumulator <=
    integral_accumulator +
    (i_product >>> 16);
```

这里的问题是：

> 每一个 1 µs update 都先 `>>>16`，把 fractional part 丢掉，然后才累加。

---

## 13.1 为什么 P 没问题

假设：

```text
error = +2831 counts×samples
Kp = 0.1
```

Q16.16：

```text
Kp ≈ 6554
```

则：

```text
2831 × 6554 >> 16
≈ 283
```

P term resolution 足够大，因此正常。

---

## 13.2 为什么 Ki 特别容易坏

group update rate：

```text
1 MHz
dt = 1 µs
```

假设：

```text
KI_PER_S = 15.2588 /s
```

则：

```text
Ki_per_update = 15.2588 × 1e-6
≈ 1.52588e-5
```

Q16.16 后：

```text
ki_q16_per_update ≈ 1
```

那么对于：

```text
error = +2831
```

：

```text
+2831 × 1 >> 16
= 0
```

但对于：

```text
error = -2831
```

signed arithmetic shift：

```text
-2831 >> 16
= -1
```

因此出现非常严重的量化不对称：

```text
positive small error → ΔI = 0
negative small error → ΔI = -1
```

即使真实 error 完全对称：

```text
+2000
-2000
+2000
-2000
...
```

理论平均：

```text
0
```

当前 integrator 却可能：

```text
0
-1
0
-1
...
```

结果：

```text
I accumulator drifts negative
```

这与实验中观察到：

```text
P-only 正常
加 I 后 I term 持续朝一个方向累积
甚至把工作点推离 target
```

高度一致。

---

# 14. 另一个 Ki 问题：Q16.16 在 1 MHz 下 resolution 太低

当前：

```text
Ki_per_update = KI_PER_S × 1e-6
```

Q16.16 最小正数：

```text
1 / 65536
```

因此最小非零：

```text
KI_PER_S
≈ 1 / (65536 × 1e-6)
≈ 15.26 /s
```

也就是说当前硬件实际只能很好表示类似：

```text
0
15.26
30.52
45.78
61.04
...
```

而如果希望 I 专门补偿：

```text
10 ms
50 ms
100 ms
```

这种 slow drift，通常希望能够设置更小、更精细的 Ki。

例如：

```text
Kp = 0.1
```

integral time：

```text
Ti = Kp / Ki
```

则：

```text
Ti = 100 ms → Ki ≈ 1 /s
Ti =  50 ms → Ki ≈ 2 /s
Ti =  20 ms → Ki ≈ 5 /s
Ti =  10 ms → Ki ≈ 10 /s
```

这些都低于当前 Q16.16 @ 1 MHz 的最小有效 15.26 /s。

所以：

> 当前 Ki representation 天生不适合非常慢的 drift compensation。

---

# 15. 当前 I accumulator 还有一个行为需要注意

当前 integrator 是有 memory 的。

如果已经积出：

```text
I = negative bias
```

进入 deadband 后，I 不一定自动 decay 回零。

因此：

```text
P error 已经很小
```

并不代表：

```text
I contribution 自动消失
```

这可能形成 persistent DAC offset。

另外 integral clamp 曾设置得相对较宽，例如：

```text
MAX_INTEGRAL_AREA_TERM_FRACTION = 0.25
```

相当于允许 I term 达到 target 的约 ±25%。

对于只想补偿 percent-level slow drift 来说，这个范围可能过大，I 很容易压过 P。

---

# 16. 下一版 Ki 的推荐修复

最重要的修改不是“调一个新的 Ki 数”，而是修 fixed-point implementation。

## 16.1 保留 fractional integral accumulator

不要：

```systemverilog
integral <= integral + ((error * ki) >>> 16);
```

应该：

```systemverilog
integral_fp <= integral_fp + error * ki;
```

其中：

```text
integral_fp
```

是更宽的 fixed-point accumulator。

只有在需要产生最终 I term 时：

```systemverilog
i_term_integer = integral_fp >>> FRACTION_BITS;
```

这样 fractional increment 会跨 pulse 保留下来。

例如理论每 pulse：

```text
ΔI = +0.043
```

正确实现：

```text
0.043
0.086
0.129
...
0.989
1.032
```

最终自然累积出 +1。

而不是：

```text
0
0
0
0
...
```

---

# 17. 推荐将 Ki 改成更高 fractional precision

例如：

```text
Q0.32 per-update
```

Python：

```python
ki_per_update = KI_PER_S * 1.0e-6
ki_reg = round(ki_per_update * 2**32)
```

例如：

```text
KI_PER_S = 1 /s
```

得到：

```text
ki_reg ≈ 4295
```

完全可以准确表示。

于是可以自然设置：

```text
Kp = 0.1

Ki = 1 /s  → Ti ≈ 100 ms
Ki = 2 /s  → Ti ≈  50 ms
Ki = 5 /s  → Ti ≈  20 ms
Ki = 10/s  → Ti ≈  10 ms
```

这更符合真实实验希望：

```text
P → fast pulse-to-pulse correction
I → ms–100 ms slow drift correction
```

---

# 18. 另一个可选的未来方案：slow / decimated I

如果希望进一步减少 I 对 high-frequency pulse noise 的敏感度，可以做：

```text
P:
    every group
    1 MHz

I:
    average N groups
    update once every N groups
```

例如：

```text
N = 1024
```

则：

```text
I update interval
≈ 1.024 ms
```

I 可以积分：

```text
mean error over last 1024 pulses
```

而不是 single-pulse raw error。

这个结构会更接近：

```text
fast P loop
+
slow drift integrator
```

但目前最优先修复仍然是：

> **fractional accumulator + higher precision Ki。**

---

# 19. 当前推荐的 safe operating state

在 Ki 修复 bitstream 生成前：

```python
KP = 已验证稳定的值
KI_PER_S = 0.0
KD = 0.0
```

先使用 P-only。

当前已经确认：

```text
P-only = physically working
```

因此没有必要在存在 numerical integral issue 的情况下继续强行调 Ki。

---

# 20. 当前 canonical firmware / software 状态

## FPGA

当前推荐：

```text
v3.12
```

状态：

```text
timing clean                 YES
ADC pulse measurement        working
GPIO sequence/group source   working
DAC2 physical output         working
P feedback                   working
FIFO idle handling           understood/fixed in software
Ki implementation            needs next revision
```

---

## Software

使用：

```text
direct /dev/mem
```

而不是 SCPI。

需要避免启动：

```bash
systemctl start redpitaya_scpi
```

因为 stock Red Pitaya service 可能重新加载：

```text
/opt/redpitaya/fpga/fpga_0.94.bit
```

覆盖 custom bitstream。

---

## Safe MMIO rule

只使用：

```python
mmap.seek(offset)
mmap.read(4)

mmap.seek(offset)
mmap.write(4_bytes)
```

不要重新使用：

```text
bulk mmap read
struct.unpack_from(mmap)
struct.pack_into(mmap)
```

否则可能再次出现 Bus error。

---

# 21. 最重要的 debugging lessons

## Lesson 1 — timing closure 比“少几个 cycles latency”重要

112 ns feedback latency 对 `<1 ms` 实验目标完全足够。

不要再尝试把所有 arithmetic 塞回单 cycle。

---

## Lesson 2 — software register 正确，不代表 physical DAC output 正确

必须同时验证：

```text
internal register
+
scope physical output
```

v3.10 的 DAC bug 就是典型例子。

---

## Lesson 3 — measurement enable 和 group generation 不是同一个 enable

FIFO empty-group bug 的关键：

```text
measurement OFF
```

并不能阻止 autonomous：

```text
DAC2 / GPIO period boundary
```

继续生成 group records。

idle 应强制：

```text
group source = manual
```

---

## Lesson 4 — AXI-Lite MMIO 不是 normal RAM

`/dev/mem` mapping 必须保守地进行 aligned 32-bit accesses。

---

## Lesson 5 — feedback arithmetic 的单位尽量留在 integer hardware domain

FPGA 内：

```text
ADC counts
ADC samples
counts × samples
```

Python/CSV 才转换：

```text
V
µs
V·µs
```

这样最容易保证 deterministic fixed-point behavior。

---

## Lesson 6 — fixed-point integrator 不能每一拍先截断再累加

对于非常小的 per-update Ki：

```text
truncate first
then accumulate
```

会产生严重 quantization bias。

正确顺序：

```text
multiply
accumulate with fractional bits preserved
convert only when output is needed
```

---

# 22. 下一步优先级

当前不需要大范围重构。

下一版 bitstream 最值得做的修改：

1. 保持 v3.12 其余 architecture 不动；
2. 保持 ADC / GPIO / FIFO / DAC path 不动；
3. 保持已经 timing-clean 的 PID pipeline 结构；
4. 只修改 I fixed-point implementation：
   - wider fractional accumulator；
   - higher precision Ki；
   - symmetric rounding；
   - sensible integral clamp；
5. 重新检查 timing；
6. 用 synthetic positive / negative error simulation 验证：
   ```text
   +e → I increases
   -e → I decreases
   symmetric ±e → no long-term drift
   ```
7. 上硬件后先验证：
   ```text
   P-only
   ```
   再：
   ```text
   very small Ki
   ```
8. 最后测：
   ```text
   open-loop drift
   vs
   P-only
   vs
   PI
   ```

---

# 23. Recommended regression tests for every future bitstream

以后每生成一个新 bit，建议固定跑以下最小 regression sequence。

### A. Timing

```text
WNS >= 0
TNS = 0
hold clean
```

### B. Register access

```text
show
feedback-off
clear-state
clear-flags
clear-fifos
```

不能有 Bus error。

### C. DAC2 physical output

```text
dac2-dc --counts 0
dac2-dc --counts 4096
dac2-square ...
```

示波器必须看到真实变化。

### D. Idle FIFO

```text
measurement OFF
feedback OFF
group source manual
```

等待数秒：

```text
pulse FIFO = 0
group FIFO = 0
```

### E. Active group source

切到：

```text
group source GPIO
```

确认 group count 按正确 frequency 增长。

### F. Pulse detector

确认：

```text
valid pulse
baseline
duration
area
peak
```

全部合理。

### G. P feedback

人工制造一个小 positive / negative error：

```text
+error → correction toward lower area
-error → correction toward higher area
```

### H. I feedback — after numerical fix

用对称 synthetic error：

```text
+E, -E, +E, -E ...
```

检查：

```text
integral mean drift ≈ 0
```

这是下一版最关键的新 regression test。

---

# 24. 当前一句话总结

目前系统已经从：

```text
严重 timing failure
→ timing closure
→ DAC register变化但 physical OUT2不动
→ DAC physical output修复
→ FIFO idle overflow
→ group-source逻辑修复
→ /dev/mem Bus error
→ safe MMIO访问
→ 1 MHz realtime FPGA + sparse Python snapshot logging
→ P feedback working
→ 发现 Ki fixed-point quantization / truncation problem
```

演进到一个已经可以进行真实 pulse measurement 和 P feedback 的系统。

当前剩下最明确、最值得下一版 bitstream 修复的问题是：

> **I term 的 fixed-point 数值实现：需要保留 fractional accumulation，并提高 Ki resolution，使其真正适合 ms–100 ms drift compensation。**
