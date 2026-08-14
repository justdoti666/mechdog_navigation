# 开发规范 Checklist（工程实践）

> 适用：mechdog_navigation / mechdog_navigation_ros 的任何代码改动。
> 原则：方案靠推演，落地靠纪律。本清单是"落地纪律"部分。

---

## 一、标定闭环（阈值/常量不许"猜完就忘"）

所有标定类参数在 `config.h` 中用注释标注来源与状态，未标定的**必须**写明"待标定"并关联标定文档。

### 当前待标定参数清单

| 参数 | 位置 | 当前值 | 状态 | 标定方法 |
|---|---|---|---|---|
| `IR_MAX_REF` | `sensor_ir.h` | 20000 | 待标定 | 见 `docs/IR_CALIBRATION.md` §1 |
| `IrConfig::ir_indoor_max/ir_outdoor_min` | `config.h` | 0.10 / 0.40 | 待标定 | 见 `docs/IR_CALIBRATION.md` §1 |
| `PlannerConfig::max_linear/angular_velocity` | `config.h` | 0.20 / 0.60 | 与底盘限幅一致 | 底盘实测后复核 |
| bottom 悬崖阈值 30cm | `sensor_ultrasonic.cpp` | 30.0 | 待标定 | 按底部安装高度实测 |

### 标定流程模板（每个参数照此走）

1. **测量**：按标定文档步骤采集真机数据（≥3 个典型场景，记录原始值）
2. **计算**：归一化/换算，确认数值落在预期量级
3. **回填**：改 `config.h` / `sensor_ir.h`，**同步更新注释**（写清实测范围与依据）
4. **验证**：模拟模式测试全绿（`ctest`）→ 真机场景复测三档判定正确
5. **归档**：标定记录（日期、环境、数据）追加到 `docs/IR_CALIBRATION.md`

> 规则：**没有标定文档的"待标定"参数不许出现**——要么写文档，要么删参数。

---

## 二、测试先行（TDD 节奏）

已有测试框架：`tests/test_fusion.cpp`（轻量断言，无第三方依赖，619+ 断言）。

### 每次新增/修改逻辑的标准循环

```bash
# 1. 先写/改测试（描述期望行为，含边界与无效输入）
# 2. 跑测试确认"红"（证明测试真的覆盖了目标逻辑）
cd /mnt/c/Users/老w/mechdog_fix/mechdog_navigation
g++ -std=c++17 tests/test_fusion.cpp sensor_ultrasonic.cpp sensor_astra.cpp \
    sensor_ir.cpp sensor_fusion.cpp path_planner.cpp -lpthread -o /tmp/tf_new \
    2>/tmp/tf_err.log && /tmp/tf_new | tail -1
# 期望: failed >= 1  (红)
# 3. 写最小实现
# 4. 重跑测试确认"绿" (failed = 0)
# 5. 回归验证: 临时回退实现, 测试必须变红 —— 证明测试能守护该逻辑
```

### 必须写测试的场景（缺一不可）

- 融合边界（L0/L1/L2/L3 分层取值）
- 无效传感器读数（超时 -1.0 / valid=false）：不得产生负距离、假急停、误标 source
- 置信度计算（含超声无效时的 Astra 单独可信度）
- 环境判定三档覆盖（模拟模式）
- 悬崖/跌落判定（无效读数必须判风险）

### 测试访问私有方法

用 `SensorFusionTestAccess` friend 访问器（**必须与类同命名空间**，见 `tests/test_fusion.cpp`）。
测试必须调用**真函数**，禁止用 lambda 复刻业务逻辑（复刻=假测试，逻辑变了它也不会失败）。

---

## 三、文档同步纪律（代码合入 = 文档合入）

1. 改行为（传感器、协议、参数、构建选项）→ 同步改 `README.md` 对应章节
2. 改接口/话题/消息 → 同步改 README 的话题表
3. 删除文件 → 同步删 README 目录树中的行 + 检查 CMakeLists/launch 引用
4. 提交信息规范：`fix(FIX-x): 描述` / `feat: 描述` / `docs: 描述` / `refactor: 描述`
5. 提交信息正文写清：问题 → 改了什么 → 验证方式（三行式）

---

## 四、提交前检查（60 秒清单）

```bash
# 1. 暂存区 diff 规模是否合理？（全文件几百行变化 = 行尾问题）
git diff --cached --stat
#    存储层行尾查询: git show HEAD:<file> | xxd | head -1  (0d0a=CRLF, 0a=LF)
#    存储 LF → 工作区转 LF; 存储 CRLF → 工作区转 CRLF + git -c core.autocrlf=false add

# 2. 残留检查（旧命名/死代码/过时引用）
grep -rn "OpenNI2\|USE_OPENNI2\|10Hz" --include="*.cpp" --include="*.h" --include="*.md" .

# 3. 编译 + 全量测试
#    Windows: build_real.bat (MSVC, USE_ASTRA_SDK=ON) + Release/test_fusion.exe
#    WSL:     colcon build --packages-select mechdog_navigation_ros
#    ctest:   cmake --build build --target test_fusion && ctest --output-on-failure

# 4. 死代码检查: 新函数/新常量是否真的被引用？
grep -rn "<新符号>" --include="*.cpp" --include="*.h" .
```

---

## 五、红线提醒

- 硬件相关路径/参数（SDK 路径、串口、CAN）不得硬编码进源码——用 CMake 选项或参数
- 提交前跑一遍 `git diff --cached` 通读自己的改动，假装是别人写的
