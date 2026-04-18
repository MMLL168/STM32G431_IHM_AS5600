# ST_FOC_G431_IMH 調適 SOP

## 1. 文件目的

本文件定義本專案之後固定遵守的正式調適流程，目標是把「馬達內層映射確認」「外層控制收斂」「旋鈕 normal follow 配合」「高速手推受擾驗證」整理成一套可重現、可交接、可回溯的 SOP。

這份文件要解決的不是單點參數記憶，而是流程失控：

1. 內層 FOC 還沒確認，就直接調外層 follow gain
2. 把高速失步、fault、重同步瞬態誤判成方向結論
3. 同一輪同時改 sign、trim、gain、slew，最後不知道是哪個變數有效
4. fixed target 都還沒穩，就直接回 normal follow 用旋鈕追
5. 旋鈕已接回後，又把 normal follow 的症狀拿去回推成 sensored 映射問題

本 SOP 的核心原則：

1. 先分層，後調參
2. 先固定目標，後接回旋鈕
3. 先低速，後高速
4. 一次只改一個維度

---

## 2. 適用範圍

本 SOP 適用於以下情境：

1. 目前這顆 STM32G431 + MCSDK + 雙 AS5600 + sensored FOC + outer follow 專案
2. 同機構或同感測器架構下的重新調適
3. 換馬達、換磁鐵、換幾何後的重新驗證
4. 後續要把 normal follow 調到可用時的正式流程

本 SOP 不適用於以下行為：

1. 只靠單次主觀感受就改很多參數
2. 跳過 fixed target / step sequence，直接拿 normal follow 猜問題
3. 沒有 livewatch 或 MCP snapshot 就直接下結論

---

## 3. 本專案目前正式基線

截至 2026-04-18，目前已驗證且應作為正式起點的基線如下：

### 3.1 Sensored 映射正式基線

1. `USER_SENSORED_DIRECTION_SIGN = -1`
2. `USER_SENSORED_ELEC_TRIM_DEG10 = 1800`
3. 這組結果對應先前 phase probe 收斂出的 phase 6

### 3.2 Outer follow 正式基線

1. `test_mode = normal_follow`
2. `source_direction_sign = 1`
3. `follower_direction_sign = 1`
4. `position_current_ma_num / den = 30 / 100`
5. `damping_current_ma_per_rpm = 2`
6. `hold_enter_deg10 / hold_exit_deg10 = 20 / 35`
7. `settle_zone_deg10 = 480`
8. `settle_min_current_ma / settle_max_current_ma / max_current_ma = 35 / 120 / 120`
9. `current_zero_window_ma = 20`
10. `breakaway_current_ma = 20`

### 3.3 已經成立的控制語意

1. normal follow 開機後採絕對角度跟隨，不保留舊的 target-follower 偏移
2. current zero window 只應在 near-target 區域生效，不應讓整段 normal follow 被提早歸零
3. 快速受擾時系統允許進入 urgent brake，不再強迫 every case 都先走 current slew

這代表後續若只是修 normal follow 的手感或收斂，不應再從 sign 或 trim 重新亂猜。

---

## 4. 工具與觀測面

### 4.1 Build / Flash / Debug

正式流程：

1. 以 Debug build 產生最新 ELF
2. 用 VS Code Debug 下載並執行
3. 除非明確要求，不另外跑獨立 flash 指令

### 4.2 `tools/follow_tuning_tool.py`

用途：

1. 完整 profile 編輯
2. preset 載入
3. 產生 `tuning_config.h`
4. Save + Build Debug
5. MCP over ASPEP live profile 套用

適用時機：

1. 要調 compile-time 基線
2. 要保存 preset
3. 要比對完整 profile 欄位

### 4.3 `tools/follow_control_app.py`

用途：

1. 操作型 GUI
2. 透過 MCP 做 scenario 切換
3. 連線後即時看 target / actual / error / speed / current
4. 快速調整少量常用參數

適用時機：

1. 已有正式基線，要做日常操作或 runtime 驗證
2. 要快速切 Normal Follow / Hold / Step / Probe
3. 要做旋鈕配合與快推受擾測試

### 4.4 Live Watch / Snapshot 核心欄位

最低必看：

1. `g_debug_monitor.follow_angle_error_deg10`
2. `g_debug_monitor.follow_target_rpm`
3. `g_debug_monitor.follow_filtered_speed_rpm`
4. `g_debug_monitor.follow_current_command_ma`
5. `g_debug_monitor.follow_reserved`
6. `g_debug_monitor.follow_test_mode`
7. `g_debug_monitor.knob_angle_deg10`
8. `g_debug_monitor.follower_angle_deg10`
9. `g_debug_monitor.iq_ref_a`
10. `g_debug_monitor.iq_a`
11. `g_debug_monitor.sensored_runtime_direction_sign`
12. `g_debug_monitor.sensored_runtime_electrical_trim_deg10`

`follow_reserved` 的正式判讀：

1. `0` = tracking
2. `1` = settle zone
3. `2` = urgent brake

direction probe 時：

1. `follow_probe_phase_index` 才代表 probe phase
2. 不要把 normal mode 的 `follow_reserved` 誤讀成 probe phase

---

## 5. 調適前置條件

每次正式調適前，先確認以下事項：

1. `devlog.md` 已記錄最新變更
2. `tools/follow_tuning_profile.json` 與 `Inc/tuning_config.h` 一致
3. 板上已燒錄最新 Debug 韌體
4. 旋鈕與 follower 機構可自由轉動，手推時不會被外部夾治具卡住
5. 電源、ST-LINK、UART/COM 連線穩定
6. 若要走 MCP GUI，先確認 COM 埠存在且 pyserial 正常

禁止事項：

1. 還沒做完 Step 1~Step 4 就直接用旋鈕追 normal follow
2. 還沒做完 fixed target / step 驗證就直接下「手感不好」結論
3. 看到 fault 或重同步瞬態就立刻翻 sign / trim

---

## 6. 正式調適流程

下面是之後必須固定遵守的正式順序。若硬體未變且基線未壞，可依條件跳過某些步驟；但判斷是否能跳過，也必須按這個順序做。

### Step 0. 建立可追溯基線

目的：

1. 確保這一輪測試有明確起點

動作：

1. 確認目前 active profile 與 preset 名稱
2. 記錄目前 commit 或目前修改點
3. 重新 Build Debug
4. 用 VS Code Debug 下載執行

觀察：

1. build 是否成功
2. 啟動後是否有立即 fault / runaway

通過條件：

1. 最新韌體已上板
2. 有明確基線可回頭比對

若不通過：

1. 先修 build / flash / debug 環境
2. 先不要開始調參

### Step 1. 確認感測器層健康

目的：

1. 先排除角度資料源頭異常

模式：

1. 可在 fixed_target 或 normal_follow 靜止狀態下觀察

動作：

1. 看 `knob_online`、`follower_online`
2. 看 `follower_magnet_detected`、`follower_magnet_too_weak`、`follower_magnet_too_strong`
3. 靜止時觀察 `knob_angle_deg10`、`follower_angle_deg10`
4. 觀察 `sensored_mech_angle_deg10` 是否與 follower 角度合理連動

通過條件：

1. knob / follower online 穩定為 1
2. 靜止時角度不應大幅跳動
3. follower magnet 狀態可接受

若不通過，先懷疑：

1. 磁鐵距離或高度
2. I2C 訊號品質
3. 感測器安裝幾何

這一步不通過時：

1. 不要調 sign / trim
2. 不要調 outer gain

### Step 2. 只在必要時重做 direction probe

目的：

1. 驗證 sensored 映射是否仍正確

何時需要做：

1. 換馬達
2. 換磁鐵或感測器幾何
3. 出現 `iq_ref_a`、`iq_a` 正常但馬達只震不轉
4. 小角度命令就持續單向 runaway
5. 明顯像在錯誤 torque axis 上工作

何時可跳過：

1. 硬體完全沒變
2. 基線本來可正常 normal follow
3. 現在只是要收斂手感或調高速受擾

模式 / preset：

1. `direction_probe`
2. `00_direction_probe`

動作：

1. 先用低速 probe，不用大電流高速暴衝
2. 只掃 sign / trim，不改 outer follow gain
3. 觀察每個 phase 是否能形成穩定低速扭矩

核心判讀：

1. `iq_ref_a` 與 `iq_a` 是否貼近
2. `sensored_speed_rpm` 是否穩定非零
3. 馬達是否真的產生單方向低速扭矩，而不是震動或鎖位

通過條件：

1. 有一組 sign / trim 可穩定低速轉動
2. 行為可重現

若不通過，先懷疑：

1. sensored sign / trim 錯
2. 極對數或機構前提已改變
3. 感測器角度品質不足

### Step 3. 固化內層結論，不讓 probe 混入正式行為

目的：

1. 把已找到的 sign / trim 變成正式基線

動作：

1. 將有效映射寫回正式設定
2. 關閉 probe mode
3. 回到正常 follow 測試鏈

本專案目前正式答案：

1. `USER_SENSORED_DIRECTION_SIGN = -1`
2. `USER_SENSORED_ELEC_TRIM_DEG10 = 1800`

原則：

1. probe 是診斷工具，不是產品模式
2. 找到正解後，不要長期把 probe 留在正式流程裡

### Step 4. 先做 `fixed_target` 零偏移靜態回位

目的：

1. 把旋鈕 noise 拿掉，只看 outer hold 是否成立

模式 / preset：

1. `fixed_target`
2. `01_fixed_target_hold`

動作：

1. 上板後不要先動旋鈕
2. 用手慢慢推 follower，確認能回原位
3. 再用手快速推 follower，觀察是否還能收斂

核心觀察欄位：

1. `follow_angle_error_deg10`
2. `follow_filtered_speed_rpm`
3. `follow_current_command_ma`
4. `follow_reserved`

通過條件：

1. 慢推能回原位
2. 快推後不應無限固定振幅擺動
3. `follow_angle_error_deg10` 的振幅應逐次縮小

若不通過，優先檢查：

1. `hold_enter_deg10 / hold_exit_deg10` 是否讓控制器介入太晚
2. outer actual filter 是否落後太多
3. damping 是否不足
4. near-target envelope 是否有跳變

這一步的原則：

1. 連零偏移 hold 都不穩時，不要接回旋鈕

### Step 5. 再做 `fixed_target` 小偏移保持

目的：

1. 確認不是只有原地回零可用，而是真的能保持偏移角

模式 / preset：

1. `fixed_target`
2. `02_fixed_target_offset_150`

動作：

1. 讓系統鎖在相對於當下中心角的 +15 度附近
2. 慢推與快推 follower
3. 觀察是否可回到偏移目標，而不是被吸回原中心或 runaway

通過條件：

1. 能維持偏移目標
2. 受擾後能回偏移目標附近

若不通過，優先檢查：

1. 外層角度符號
2. near-target current envelope
3. hold / release 邏輯是否過度依賴零速條件

### Step 6. `step_sequence` 小步階，專注 near-target 收斂

目的：

1. 調 near-target 收斂，不讓它卡在中小誤差或尾端抖動

模式 / preset：

1. `step_sequence`
2. `03_step_small_120`

建議範圍：

1. step amplitude 約 `100 ~ 150 deg10`
2. dwell 約 `1200 ~ 1800 ms`

核心觀察：

1. overshoot 次數
2. 振幅是否逐次縮小
3. `follow_current_command_ma` 在接近目標時是否太早掉到很低

通過條件：

1. 可以像欠阻尼鐘擺一樣縮小
2. 不會停在 `100 deg10` 左右不再收

若不通過，優先懷疑與處理順序：

1. 若 `filtered_speed_rpm ≈ 0` 且 `current_command_ma` 只有十幾到二十幾 mA，優先提高 `position_current_ma_num` 與 `settle_min_current_ma`
2. 若尾端像 `0 mA / 20 mA` 一直跳，優先檢查 breakaway ramp 與 current zero window
3. 若已跨過目標但還一直追，優先檢查 outer actual filter 與阻尼相位

### Step 7. `step_sequence` 中步階，專注大角度能量與煞車

目的：

1. 驗證較大動能時能不能煞住並走最短路徑

模式 / preset：

1. `step_sequence`
2. `04_step_medium_360`

建議範圍：

1. step amplitude 約 `300 ~ 600 deg10`

核心觀察：

1. 第一個 overshoot 大小
2. 收斂總時間
3. 是否持續來回擺盪
4. `follow_reserved` 是否在大動能時進到 `2 = urgent brake`

通過條件：

1. 大角度時能先煞車再收斂
2. 不會變成固定振幅擺動

若不通過，優先懷疑：

1. 阻尼不足
2. urgent brake 未進場
3. current slew 把反向煞車建立拖太慢
4. max current / settle current limit 太保守

### Step 8. 接回 `normal_follow`，驗證旋鈕絕對角跟隨

目的：

1. 確認 fixed target / step 都成立後，真正接回 knob

模式 / preset：

1. `normal_follow`
2. `05_normal_follow`

動作：

1. 上板後先不動旋鈕，確認靜止穩定
2. 小角度慢轉旋鈕，觀察 follower 是否線性跟上
3. 中角度轉動旋鈕，觀察是否能縮回誤差
4. 再做較大角度與快速轉動

核心觀察欄位：

1. `knob_angle_deg10`
2. `follow_target_angle_deg10`
3. `follow_actual_angle_deg10`
4. `follow_angle_error_deg10`
5. `follow_current_command_ma`
6. `follow_filtered_speed_rpm`

通過條件：

1. 開機後以絕對角度跟隨 knob
2. 小角度動作不應明顯顆粒或抖動
3. 中角度誤差可以回到小範圍

若不通過，先用症狀分類：

1. 若方向就錯，回 Step 2/3
2. 若方向對但殘差大，優先查 `position_current_ma_num`、`settle_min_current_ma`
3. 若尾端抖，優先查 breakaway ramp、current zero window、current slew
4. 若大角度來回擺，優先查 damping、outer filter、urgent brake

### Step 9. 專門做「快推 follower」受擾驗證

目的：

1. 驗證真實手動擾動下能否收斂，而不是只會慢速回位

這一步必做，因為慢推可回位不代表高速擾動可收斂。

動作：

1. 在 `fixed_target` 與 `normal_follow` 各做一次
2. 先慢推，再快推
3. 快推時看 `follow_reserved` 是否出現 `2`

正式判讀：

1. 慢推可回位但快推落入固定約 45 度來回擺動
	- 這不是靜態 restoring torque 問題
	- 優先懷疑高速煞車建立太慢
2. 若快推時 `follow_reserved = 2` 仍持續固定擺動
	- 優先補 `damping_current_ma_per_rpm`
3. 若快推時幾乎沒進 urgent brake
	- 優先查條件、outer angle lag、或輸出限制

通過條件：

1. 快推後振幅仍能縮小
2. 不再卡在固定約 45 度的極限環

### Step 10. 最後才優化手感與應用 GUI 操作

目的：

1. 在收斂已成立後，再調操作感與應用流程

推薦工具分工：

1. 若要改完整 profile、存 preset、Build，用 `follow_tuning_tool.py`
2. 若要做 runtime scenario 切換、看即時狀態、快速調少量參數，用 `follow_control_app.py`

應用 GUI 驗證內容：

1. Connect / Disconnect 穩定
2. Apply Preset 正常
3. Scenario 切換後 board state 與 snapshot 正確
4. `Normal Follow`、`Hold Here`、`Step 12 deg`、`Step 36 deg` 行為與預期一致

這一步的原則：

1. GUI 只是操作面，不應拿來掩蓋控制層本身還沒調好

---

## 7. 參數調整順序

之後若要調參，固定遵守下列順序，不要亂跳：

### 7.1 先處理收斂，再處理手感

順序：

1. sign / trim
2. outer angle sign 與絕對角語意
3. hold / release 條件
4. position spring 與 damping
5. near-target envelope
6. breakaway
7. target slew / current slew

### 7.2 每類症狀優先動的參數

#### A. 殘差太大，停在 `100 deg10` 左右不再收

先動：

1. `position_current_ma_num`
2. `settle_min_current_ma`

#### B. 快推後固定大幅擺動

先動：

1. `damping_current_ma_per_rpm`
2. urgent brake 相關行為
3. outer angle filter lag

#### C. 尾端抖、顆粒、像 0mA/固定 mA 一直跳

先動：

1. breakaway ramp
2. `current_zero_window_ma`
3. current slew

#### D. 小角度太黏、動作不跟手

先動：

1. `target_angle_slew_min_deg10_per_tick`
2. `target_angle_slew_max_deg10_per_tick`
3. `target_angle_slew_divisor`

#### E. 大角度很衝、第一下過頭太多

先動：

1. `damping_current_ma_per_rpm`
2. `max_current_ma`
3. `settle_max_current_ma`

---

## 8. 症狀對應表

### 現象 A：`iq_ref_a` 與 `iq_a` 很接近，但馬達只震不轉

優先嫌疑：

1. sensored sign / trim 錯
2. torque axis 偏 90 或 180 度
3. 感測器磁鐵品質不足

### 現象 B：小角度就單向 runaway

優先嫌疑：

1. outer angle sign 錯
2. 映射雖有扭矩，但控制回授方向錯

### 現象 C：`normal_follow` 基本可用，但殘差一直偏大

優先嫌疑：

1. `position_current_ma_num` 太低
2. `settle_min_current_ma` 太低
3. current zero window 過早壓掉 restoring torque

### 現象 D：收斂末段像抖抖抖地到目標

優先嫌疑：

1. breakaway floor 跳變
2. current slew 不夠平順
3. near-target envelope 不連續

### 現象 E：慢推可回目標，快推卻落入約 45 度來回擺動

優先嫌疑：

1. 高速煞車建立太慢
2. outer actual angle 有相位落後
3. 阻尼不足
4. urgent brake 未真正生效或 current slew 仍拖住輸出

### 現象 F：大角度時 target rpm 方向對，但實機還是越跑越遠

優先嫌疑：

1. 方向號誌仍有符號不一致
2. 感測角度或 mapped follower angle 定義錯

---

## 9. 何時可以說「旋鈕配合已完成」

只有同時滿足以下條件，才能說這顆系統的 knob-follow 已調到可用：

1. `fixed_target` 零偏移可回位
2. `fixed_target` 小偏移可保持
3. `step_sequence` 小步階可逐次縮小振幅
4. `step_sequence` 中步階不會固定大幅擺動
5. `normal_follow` 開機後以絕對角度跟隨 knob
6. 小角度旋鈕動作不明顯抖動
7. 快推 follower 後仍能回到目標，且振幅縮小

只要少一項，都不能只用「基本上會動」當結論。

---

## 10. 固定操作原則

1. 任何程式或設定變更都必須同步更新 `devlog.md`
2. 使用者按 Debug 就會下載與執行，除非明確要求，不再另外做獨立 flash
3. 不在同一輪同時改 sign、trim、外層 gain、slew
4. 不用高速瞬態判方向
5. phase 只用來診斷，找到正解就固化成正式參數
6. 所有結論都要以可重現的低速穩態與 livewatch / MCP snapshot 為主

---

## 11. 換馬達時如何套用本 SOP

### 11.1 同型號、同機構直替

可以：

1. 先沿用目前正式基線
2. 但至少重做 Step 1、Step 4、Step 8、Step 9

若出現下列任一項，就退回 Step 2：

1. 只震不轉
2. 小角度就單向發散
3. 正反方向有一邊完全不成立
4. `iq_ref_a` 與 `iq_a` 正常，但實機方向不對

### 11.2 不同型號，但同感測器架構

可以沿用：

1. 流程
2. 觀測欄位
3. 判讀方法

不能直接照抄：

1. sign / trim 正式值
2. 電流上限
3. 外層 gain
4. slew 與 envelope

建議：

1. 從 Step 0 開始完整走到 Step 4

### 11.3 不同馬達且感測器幾何改變

視為新專案：

1. 完整重走 Step 0 到 Step 10
2. 不要把目前 phase 6 當成預設正解

原因：

1. 只要磁鐵與感測器相對幾何改變，mechanical angle 到 electrical angle 的有效映射就可能整體改掉

### 11.4 換馬達時的最快決策表

如果只是想快速決定下一步，照下面走：

1. 同型號、同安裝、同磁鐵：先用目前正式基線，失敗再退回 Step 2 probe
2. 不同型號、同機構：直接從 Step 0 開始，但預期要重找 sign / trim
3. 不同型號且磁鐵或 AS5600 幾何也變：直接視為新映射問題，先 probe，不要直接 follow

### 11.5 換馬達時最常犯的錯

1. 看到同樣三相無刷就直接沿用上一顆的 sign / trim
2. 馬達不轉時先去調 follow gain
3. 跳過低速 probe，直接看高速方向
4. 沒更新極對數與最大速度就開始驗證
5. 把 fault 後的再起轉方向當成最終方向結論

---

## 12. 建議的現場調試記錄格式

每次測試至少記錄以下內容：

1. 測試日期時間
2. 韌體版本或對應的 devlog 項次
3. 使用的 mode / preset
4. 若是 probe mode，phase 是多少
5. knob 是否動作、方向、幅度
6. 慢推與快推 follower 的結果
7. `follow_angle_error_deg10`
8. `follow_target_rpm`
9. `follow_filtered_speed_rpm`
10. `follow_current_command_ma`
11. `follow_reserved`
12. `iq_ref_a`
13. `iq_a`
14. `current_faults`
15. `occurred_faults`

只有這樣，後續比較不同版本時才有完整上下文。

---

## 13. 總結

這份 SOP 的真正目的，是把調適從「憑感覺猜」改成「按層次驗證」。

固定順序如下：

1. 先感測器
2. 再內層 sensored 映射
3. 再 fixed target 與 step
4. 再接回 normal follow 與 knob
5. 最後才調手感與應用 GUI

只要照這個順序做，之後即使換馬達、改磁鐵位置、或重新對位，也能快速知道問題落在哪一層，而不是回到從頭亂猜參數。