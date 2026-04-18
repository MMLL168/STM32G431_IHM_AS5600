# ST_FOC_G431_IMH 調適 SOP

## 1. 文件目的

本文件的目的不是記錄零散的調參心得，而是定義一套之後必須固定遵守的調適流程，避免再次出現以下問題：

1. 同時改太多變數，最後無法判斷是哪一個改動有效
2. 內層 FOC 映射尚未驗證時，就直接調外層 follow gain
3. 把高速失步、重新同步、保護動作誤判成方向結論
4. 用單次偶發現象代替可重現的低速穩態驗證

本 SOP 的核心原則只有一句話：

先分層，後調參；先低速，後高速；一次只改一個維度。

---

## 2. 本專案目前已驗證的正式結論

截至 2026-04-18，本專案已透過 phase probe 完成 sensored 電角映射辨識，正式結論如下：

1. 正式 sensored direction sign = -1
2. 正式 sensored electrical trim = 1800 electrical degree deg10
3. 上述結果已固化於 `Inc/user_sensored.h`
4. probe 模式已關閉，系統已恢復正常 follow loop

對應檔案：

1. `Inc/user_sensored.h`
2. `Src/follow_control.c`
3. `devlog.md`

這代表之後若要繼續調適 follow 手感，預設基線應以這組映射為準，不要再回頭從 `direction sign = 1` 或 `trim = 0` 開始猜。

---

## 3. 系統分層觀念

本專案至少分成 4 層，調適時不能把它們混在一起看。

### 3.1 感測器層

負責提供 knob 與 follower 的機械角度。

1. knob AS5600 提供目標角
2. follower AS5600 提供實際角與 FOC feedback 基礎資料

如果這一層不穩，後面所有結論都不可靠。

### 3.2 sensored 映射層

負責把 follower 的 mechanical angle 轉成 electrical angle。

概念公式如下：

theta_e = (theta_m - theta_align) * pole_pairs * direction_sign + lock_angle + trim

如果這一層錯，最典型的現象就是：

1. 電流打得進去，但馬達只震不轉
2. 同樣的 q 軸電流在某些位置像鎖位，不像扭矩
3. 外層命令看起來合理，但實機方向完全不對

### 3.3 FOC 電流輸出層

負責把 q 軸命令轉成實際扭矩。

驗證這層時要看：

1. `iq_ref_a`
2. `iq_a`

如果 `iq_ref_a` 與 `iq_a` 貼得很近，但馬達仍不轉，問題通常不在電流 PI，而在電角映射或感測品質。

### 3.4 外層 follow 控制層

負責把 knob 與 follower 的角度誤差轉成目標轉速與目標電流。

只有在前 3 層已經成立時，調這一層才有意義。

---

## 4. phase 的真正意思

本次調適曾使用 phase probe。phase 不是產品模式，而是用來做內層電角映射診斷的索引。

phase table 定義在 `Src/follow_control.c`，其本質是在掃兩個維度：

1. runtime direction sign
2. runtime electrical trim

### 4.1 phase 對照表

| phase | runtime direction sign | runtime trim |
| --- | --- | --- |
| 0 | +1 | 0 |
| 1 | +1 | +900 |
| 2 | +1 | +1800 |
| 3 | +1 | -900 |
| 4 | -1 | 0 |
| 5 | -1 | +900 |
| 6 | -1 | +1800 |
| 7 | -1 | -900 |

### 4.2 phase 的意義

phase 並不代表「第幾種馬達模式」，而是代表：

目前我正在假設 electrical angle 的斜率方向與常數偏移是這一組。

所以 phase probe 的用途只有一個：

找出哪一組 `(direction_sign, trim)` 能讓固定 q 軸命令形成穩定、可重現的實際扭矩。

### 4.3 這次為什麼最後收斂到 phase 6

實測結果顯示：

1. phase 4、6、7 在 sweep 中都曾出現有效旋轉
2. phase 7 配負 q 軸可穩定低速逆時針
3. phase 6 配正 q 軸可穩定低速順時針
4. phase 6 是後續最適合回到正常 follow 的正式候選

因此最後把 phase 6 的結果固化為正式 compile-time 設定：

1. `USER_SENSORED_DIRECTION_SIGN = -1`
2. `USER_SENSORED_ELEC_TRIM_DEG10 = 1800`

注意：

probe 裡的 phase 6 是 runtime 組合；固化後則是把同樣的結果直接寫成 compile-time 基線，之後 runtime override 回到中性值。

---

## 5. Live Watch 核心欄位與判讀

之後調適時，優先看下列欄位，不要一開始就盯所有變數。

### 5.1 感測器健康

1. `g_debug_monitor.follower_online`
2. `g_debug_monitor.follower_magnet_too_weak`
3. `g_debug_monitor.follower_angle_deg10`
4. `g_debug_monitor.sensored_mech_angle_deg10`

判讀：

1. `follower_online` 必須穩定為 1
2. `follower_magnet_too_weak = 1` 代表磁鐵場強偏弱，是風險，不一定立即致命，但任何邊界行為都要懷疑它
3. 靜止時角度不應該大幅跳動

### 5.2 電流是否真的打進去

1. `g_debug_monitor.iq_ref_a`
2. `g_debug_monitor.iq_a`

判讀：

1. 兩者接近，代表電流輸出層成立
2. 兩者差很多，先查 current loop 或 PWM 狀態

### 5.3 電角映射是否合理

1. `g_debug_monitor.sensored_elec_angle_deg10`
2. `g_debug_monitor.sensored_speed_rpm`

判讀：

1. 若 `iq_ref_a` 與 `iq_a` 很接近，但 `sensored_speed_rpm = 0` 且馬達只震，優先懷疑電角映射錯誤
2. 若某組 sign/trim 下能穩定低速單方向轉，表示該組映射有效

### 5.4 外層 follow 是否正確

1. `g_debug_monitor.follow_angle_error_deg10`
2. `g_debug_monitor.follow_target_rpm`
3. `g_debug_monitor.follow_filtered_speed_rpm`

判讀：

1. angle error 增大時，target rpm 應朝能縮小誤差的方向變化
2. 如果 target rpm 在煞車方向，但實機卻持續朝同方向 runaway，通常是映射或方向號誌仍有問題

### 5.5 fault 判讀

1. `g_debug_monitor.current_faults`
2. `g_debug_monitor.occurred_faults`

判讀：

1. 若出現 speed feedback fault，先查速度可靠度邊界與感測品質
2. 不要把 fault 後的停轉再起轉瞬態拿來當方向結論

---

## 6. 正式調適 SOP

下面是之後必須遵守的固定順序。

### Step 0. 建立基線

目的：

確保你知道目前韌體是哪一版、基線參數是什麼。

動作：

1. 確認 `devlog.md` 已記錄最新修改
2. 以 Debug build 產生最新 ELF
3. 使用者按 Debug 進行下載與執行，不要另外做獨立 flash

原因：

如果不是明確可追溯的基線，後續所有結論都不可重現。

### Step 1. 確認感測器層

目的：

先排除資料源頭不穩。

動作：

1. 看 follower 與 knob 是否 online
2. 看 follower magnet 狀態
3. 靜止時觀察角度是否異常亂跳

通過條件：

1. online 穩定
2. 角度變化合理

若不通過：

1. 不要調 follow gain
2. 不要調 sign/trim
3. 先處理磁鐵幾何、距離、I2C 資料品質

### Step 2. 只驗證內層電角映射

目的：

確認 fixed q-axis current 能不能形成穩定扭矩。

動作：

1. 啟用 probe 模式
2. 一開始用低速 probe，不要直接用大電流高速暴衝
3. 只掃 sign 與 trim，不碰外層 follow gain

通過條件：

1. 某一組組合能穩定低速單方向轉
2. `iq_ref_a` 與 `iq_a` 接近
3. `sensored_speed_rpm` 非零且穩定

原因：

如果這一步沒過，外層控制不可能正確。

### Step 3. 確認扭矩符號對應方向

目的：

確認在有效映射下，正 q 與負 q 各對應哪個實際方向。

動作：

1. 保持 sign/trim 不變
2. 只翻 q 軸命令符號
3. 用低速 probe 驗證方向

通過條件：

1. 低速穩態下，方向可重現
2. 不依賴高速再同步瞬態做結論

原因：

這一步只是在回答：同一組電角映射下，正扭矩是順時針還是逆時針。

### Step 4. 將有效映射固化

目的：

把診斷結果轉成正式基線。

動作：

1. 將成功的 sign/trim 寫回 `Inc/user_sensored.h`
2. 關閉 probe 模式
3. 恢復正常 follow loop

原因：

phase probe 只是診斷工具，不應長期留在正式行為裡。

### Step 5. 驗證正常 follow 基本功能

目的：

確認系統已從「能單方向轉」回到「能跟隨目標角」。

動作：

1. knob 不動時，觀察馬達是否安靜穩定
2. 小角度順時針轉 knob，觀察馬達是否同方向跟上
3. 小角度逆時針轉 knob，觀察馬達是否反方向跟上
4. 避免一開始就做大角度或高速動作

通過條件：

1. 無靜止 runaway
2. 正反方向都能跟上
3. `follow_target_rpm` 與 `iq_ref_a` 方向合理

### Step 6. 最後才調 follow 手感

目的：

在方向與映射已正確後，改善實際操作體驗。

可以調的典型參數：

1. `FOLLOW_CONTROL_HOLD_ENTER_DEG10`
2. `FOLLOW_CONTROL_HOLD_EXIT_DEG10`
3. `FOLLOW_CONTROL_SPEED_KP_RPM_PER_DEG`
4. `FOLLOW_CONTROL_SPEED_KD_RPM_PER_RPM`
5. `FOLLOW_CONTROL_SPEED_TO_CURRENT_KP_MA_PER_RPM`
6. `FOLLOW_CONTROL_MAX_CURRENT_MA`
7. `FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK`

原則：

1. 一次只改一個 gain 或一組很小的相關參數
2. 每次改完只驗證一種感受問題
3. 先求穩定，再求手感

---

## 7. 不同現象對應的優先嫌疑

### 現象 A：`iq_ref_a` 與 `iq_a` 很接近，但馬達只震不轉

優先嫌疑：

1. 電角 sign/trim 錯
2. torque axis 偏 90 或 180 度
3. 感測器磁鐵品質不足

### 現象 B：馬達穩定只往單一方向 runaway

優先嫌疑：

1. 外層 follow 正負回授錯
2. 映射雖能產生扭矩，但方向定義與 follow 邏輯不一致

### 現象 C：高速轉一段後停一下，再重新起轉

優先嫌疑：

1. 速度可靠度邊界太保守
2. speed feedback fault
3. 高速 back-EMF 區的再同步瞬態

### 現象 D：低速方向穩定，但高速不穩

優先嫌疑：

1. 感測品質邊界
2. speed reliability 設定
3. 高速區電流或電壓裕量不足

### 現象 E：某個 phase 只在一個 q 軸符號有效

優先嫌疑：

1. 該 phase 不是正式候選
2. 等效 torque axis 對另一個符號不是最佳對應

---

## 8. 本專案之後的固定操作原則

1. 任何程式或設定變更都必須同步更新 `devlog.md`
2. 使用者按 Debug 就會下載與執行，除非明確要求，不再另外做獨立 flash
3. 不在同一輪同時改 sign、trim、q 軸符號、follow gain
4. 不用高速瞬態判方向
5. phase 只用來診斷，找到正解就固化成正式參數
6. 所有結論都要以可重現的低速穩態現象為主

---

## 9. 本專案目前正式基線

當前正式基線如下：

1. `USER_SENSORED_DIRECTION_SIGN = -1`
2. `USER_SENSORED_ELEC_TRIM_DEG10 = 1800`
3. `FOLLOW_CONTROL_DIRECTION_PROBE_ENABLE = 0`
4. `MAX_APPLICATION_SPEED_RPM = 3840`
5. custom sensored speed handle 已補上 1.15 倍 reliable speed headroom

若之後 follow 仍有異常，下一步應從外層 follow 參數與角度誤差行為開始檢查，而不是回頭重新亂掃 phase。

---

## 10. 換馬達時如何使用本 SOP

本 SOP 可以直接沿用到下一顆馬達，但要先分清楚你換的是哪一種情境。這份文件可以沿用的是調適流程，不是保證所有馬達都能直接沿用這次的正式參數。

### 10.1 哪些內容可以沿用

以下內容可以直接沿用：

1. 分層調適方法
2. Live Watch 欄位與判讀順序
3. 先低速 probe、再回 follow 的策略
4. phase 掃描的概念與判讀方法
5. devlog 與現場記錄格式

### 10.2 哪些內容不能直接照抄

以下內容不能因為上一顆馬達成立，就直接假設下一顆也成立：

1. `USER_SENSORED_DIRECTION_SIGN`
2. `USER_SENSORED_ELEC_TRIM_DEG10`
3. 哪個 phase 最後會是正解
4. 低速 probe 的電流大小
5. `MAX_APPLICATION_SPEED_RPM`
6. 電流限制與 slew rate
7. 外層 follow gain

只要下列任一項改變，上面這些內容就都可能失效：

1. 馬達型號
2. 極對數
3. 相線接法
4. follower 磁鐵位置、高度、強度
5. AS5600 安裝角度
6. 機構負載

### 10.3 情境 A：同型號直替

定義：

1. 同一型號馬達
2. 相同相線接法
3. 相同感測器磁鐵與安裝幾何
4. 相同供電與驅動板

建議做法：

1. 可以先用目前正式基線直接起跑
2. 不必預設先做完整八相掃描
3. 但仍必須至少重做 Step 1 與 Step 5 的驗證

最低重驗項目：

1. follower online 是否穩定
2. magnet 狀態是否正常
3. 靜止時是否 runaway
4. 小角度順時針是否正確跟上
5. 小角度逆時針是否正確跟上

什麼情況下要退回 probe：

1. 只震不轉
2. 小角度就單向發散
3. 正反方向有一邊完全不成立
4. `iq_ref_a` 與 `iq_a` 正常，但實機方向不對

若出現上述任一項，就不要繼續猜 follow gain，直接退回 Step 2 做低速 probe。

### 10.4 情境 B：不同型號，但機構與感測器架構相同

定義：

1. 仍是相同驅動板
2. 仍是相同雙 AS5600 架構
3. 但馬達型號不同，或極對數不同，或反電勢常數不同

建議做法：

1. 流程沿用本 SOP
2. 參數不要直接沿用目前正式基線
3. 要從 Step 0 開始，完整走到 Step 4

在進行 probe 前，優先先確認或更新：

1. 極對數
2. 最大速度
3. 電流上限
4. 馬達相關 generated 參數

這個情境下的原則是：

1. 可以沿用方法
2. 不能預設沿用 sign/trim 結論

### 10.5 情境 C：完全不同馬達或不同感測器幾何

定義：

1. 馬達型號不同
2. 磁鐵位置或高度改變
3. AS5600 安裝角度改變
4. 馬達與感測器的相對幾何關係改變

這個情境要視為新專案，不要把目前 phase 6 的結果當成起點。

建議做法：

1. 完整重走 Step 0 到 Step 6
2. 一開始就先準備低速 probe
3. 不要直接恢復 follow loop
4. 先重新找有效 phase，再談手感

原因：

只要磁鐵與感測器的幾何改變，mechanical angle 到 electrical angle 的有效映射就可能整體改掉，這不是調 gain 能補回來的問題。

### 10.6 換馬達時的最快決策表

如果你只是想快速判斷下一步，照下表：

1. 同型號、同安裝、同磁鐵：先用目前正式基線，失敗再退 probe
2. 不同型號、同機構：直接從 Step 0 開始，但預期要重找 sign/trim
3. 不同型號且磁鐵或 AS5600 幾何也變：直接視為新映射問題，先 probe，不要直接 follow

### 10.7 換馬達時最常犯的錯

1. 看到同樣三相無刷就直接沿用上一顆的 sign/trim
2. 馬達不轉時先去調 follow gain
3. 跳過低速 probe，直接看高速方向
4. 沒更新極對數與最大速度就開始驗證
5. 把 fault 後的再起轉方向當成最終方向結論

---

## 11. 建議的現場調試記錄格式

每次測試請至少記錄以下內容：

1. 測試日期時間
2. 韌體版本或 devlog 對應項次
3. 是否 probe mode
4. 若是 probe mode，phase 是多少
5. knob 是否動作，動作方向與幅度
6. 實機方向與是否穩定
7. `follow_angle_error_deg10`
8. `follow_target_rpm`
9. `sensored_speed_rpm`
10. `iq_ref_a`
11. `iq_a`
12. `current_faults`
13. `occurred_faults`

只有這樣，後續比較不同版本時才有完整上下文。

---

## 12. 總結

這份 SOP 的真正目的，是把調適從「憑感覺猜」改成「按層次驗證」。

固定流程如下：

1. 先感測器
2. 再內層電角映射
3. 再扭矩符號
4. 再固化正式參數
5. 最後才調外層 follow 手感

只要照這個順序做，之後即使要更換馬達、磁鐵位置、或重新對位，也能快速知道問題落在哪一層。