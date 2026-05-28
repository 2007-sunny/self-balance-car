# 雙輪自平衡小車 v2 (Self-Balancing Robot v2) 整合使用手冊

這是一份針對 `self_balance_v2` 專案的完整軟硬體技術與操作說明書。本手冊不僅包含車體的操作指南，也整合了如何在 **Visual Studio Code (VS Code)** 中進行開發、編譯以及執行藍牙遙測工具 (Telemetry Plot) 的完整流程。

---

## 一、 在 VS Code 中的開發與執行 (VS Code Setup & Usage)

這份專案建議使用 Visual Studio Code 進行開發，因為您可以同時處理 Arduino C++ 程式碼與 Python/PowerShell 測試腳本。

### 1. 開啟專案
1. 在 VS Code 中點選 `檔案 (File)` > `開啟資料夾 (Open Folder)...`。
2. 選擇專案的根目錄：`C:\Users\User\Desktop\sunny\Arduino\self_balance_v2`。

### 2. 編譯與上傳 Arduino 程式碼
1. 請確保已安裝 VS Code 的 **Arduino 擴充套件 (Microsoft)**。
2. 在底部的狀態列選擇正確的開發板（**Arduino Nano**）與通訊埠（連接 Arduino 的 COM Port）。
3. 打開 `self_balance_v2.ino` 檔案。
4. 點擊右上角的 **上傳 (Upload)** 按鈕（向右箭頭圖示）進行編譯並上傳至小車。

### 3. 執行藍牙遙測與即時繪圖工具 (Telemetry Monitor)
在 VS Code 中，您可以直接使用整合終端機 (Terminal) 來啟動藍牙監測與繪圖腳本。

1. 從頂部選單選擇 `終端機 (Terminal)` > `新增終端機 (New Terminal)`。
2. **尋找藍牙 COM Port**：如果不知道藍牙模組的通訊埠，輸入以下指令會列出所有可用的 COM Port：
   ```powershell
   .\tools\run_telemetry_plot.ps1
   ```
3. **啟動監測器**：找到正確的 COM Port 後（假設為 `COM5`），輸入並執行：
   ```powershell
   .\tools\run_telemetry_plot.ps1 -Port COM5
   ```
4. **運作與記錄**：
   - 腳本會自動啟動專案底下的 Python 虛擬環境 (`.venv`)，您無需手動安裝套件。
   - 視窗會自動彈出並即時繪製**傾角 (Angle)**、**角速度 (Gyro)** 以及**馬達速度 (Motor CMD & Actual)** 的折線圖。
   - 所有的遙測資料會自動存入專案下的 `logs/` 資料夾，檔名類似 `telemetry_YYYYMMDD_HHMMSS.csv`，方便事後分析。
5. **結束監測**：在 VS Code 終端機按下 `Ctrl + C`，或是直接關閉繪圖視窗，程式會自動停止記錄並關閉小車的資料發送模式。

---

## 二、 硬體架構與腳位定義 (Hardware & Wiring)

### 1. 核心控制器與感測器
*   **微控制器**: Arduino Nano
*   **慣性測量單元 (IMU)**: MPU6050 (I2C 位址 `0x68`)
*   **顯示器**: LCD 1602 I2C 模組 (I2C 位址 `0x27`)
*(I2C 腳位為 `A4 (SDA)` 與 `A5 (SCL)`)*

### 2. 馬達驅動 (A4988 步進馬達驅動器 x 2)
*   **左馬達 (A)**: `STEP` 接 D2, `DIR` 接 D3
*   **右馬達 (B)**: `STEP` 接 D4, `DIR` 接 D5
*   **馬達使能 (ENABLE)**: D6 (兩側共用，`LOW` = 啟動，`HIGH` = 斷電/解鎖)

### 3. 操作介面 (EC11 旋轉編碼器)
*   `CLK` (A相): A0
*   `DT` (B相): A1
*   `SW` (按鈕): A2

### 4. 無線通訊 (HC-05 藍牙模組)
*   `RX`: D10 (接 HC-05 的 TXD)
*   `TX`: D11 (接 HC-05 的 RXD，建議透過電阻分壓降至 3.3V)

---

## 三、 軟體系統與控制原理 (System Architecture)

1. **姿態解算 (MPU6050 + 互補濾波)**：以 200Hz 頻率透過互補濾波器融合加速度計與陀螺儀資料，取得平滑無飄移的真實傾角。
2. **平衡控制 (單層 PID 控制)**：僅使用角度 PID 外環。若傾角超過 45 度，會自動判定為跌倒並切斷馬達電源。
3. **馬達速度與脈衝 (Timer1 + 梯形加減速)**：Timer1 產生 20kHz 高頻脈衝。搭配梯形加減速演算法 (每 5ms 最多改變 25 steps/s) 防止步進馬達失步。
4. **參數儲存 (EEPROM)**：自動保存 PID 參數、平衡基準角、校正方向與馬達最大速限，斷電不遺失。

---

## 四、 UI 選單操作指南 (Menu Operations)

透過**旋轉編碼器**可以完全脫離電腦進行設定與操作。
*   **旋轉**: 切換選單項目 或 增減數值。
*   **短按**: 確認 / 進入子選單 / 儲存數值。
*   **長按 (> 0.8秒)**: 返回上一層選單 或 退出/緊急停止。

### 系統選單結構：
1.  **1.Start Balance**: 啟動馬達並進入自平衡模式。在平衡中**短按**可退出。
2.  **2.MPU6050 Setup**:
    *   `2-1.Angle View`: 觀看即時三軸加速度角度與融合傾角。
    *   `2-2.Base Adjust`: 手動微調平衡基準角度 (旋轉 ±0.1度)。
    *   `2-3.Tilt Calib (傾倒校正)`: 智慧校正。將車體分別往前傾斜大於 15度，再往後傾斜大於 15度，系統會自動算出基準中心點。
3.  **3.Bluetooth Tst**: 藍牙 PING 測試與封包監聽模式。
4.  **4.Motor Test**: 馬達旋轉測試 (Slow / Med / Fast)。
5.  **5.PID Params**: 微調 `Kp`, `Ki`, `Kd` 並存入 EEPROM。
6.  **6.Linear Test**: 測試步進馬達線性加減速極限。
7.  **7.Max Speed Adj**: 設定 PID 輸出的最大馬達轉速限制。

---

## 五、 藍牙遙控與通訊指令 (Bluetooth Commands)

藍牙功能在平衡模式 (`Start Balance`) 下生效。指令需以換行 (`\n`) 結尾。

*   **遙控指令**:
    *   `GO=X.X` (例如 `GO=2.5`) : 控制車子前進或後退 (限制在 ±8.0 度內)。
    *   `STOP` : 停止移動 (偏移量歸零)。
*   **線上調參**: `AP=X.X` (調Kp)、`AI=X.X` (調Ki)、`AD=X.X` (調Kd)。
*   **遙測資料控制**:
    *   `LOG=1` / `LOG=0` : 開啟 / 關閉 20Hz 的即時資料回傳（Telemetry Monitor 工具會自動發送此指令）。

---

## 六、 首次組裝開機建議流程 (First-time Setup)

1.  **馬達測試**: 進入 `4.Motor Test`，確認兩顆馬達轉向一致，且不會發出怪聲。
2.  **感測器方向校正**: 進入 `2-3.Tilt Calib`，跟隨螢幕指示，將車體分別前後傾斜，確保感測器安裝正反面都不影響運作。
3.  **微調中心點**: 若車子平衡時會緩慢滑行，進入 `2-2.Base Adjust` 微調幾零點幾度的基準角。
4.  **開始平衡與觀測**: 選擇 `1.Start Balance` 放手測試，並在 VS Code 啟動 `run_telemetry_plot.ps1` 觀察曲線是否平穩。
