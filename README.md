# CtrlKine-AMR: 2D Physics Simulator & Environment Editor
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B)
![SFML](https://img.shields.io/badge/SFML-3.0.0-8CC445?style=for-the-badge&logo=c%2B%2B)
![Windows](https://img.shields.io/badge/OS-Windows_MinGW-0078D6?style=for-the-badge&logo=windows)

## Project Overview (專案概述)
**CtrlKine-AMR** 是一個從零建構的輕量化 2D 物理模擬器與環境編輯器。本專案旨在提供一個類似 Gazebo 的 2D 虛擬環境，作為自主移動機器人 (Autonomous Mobile Robot, AMR) 在進入實體硬體部署前的演算法驗證平台。

透過高度解耦 (Decoupling) 的軟體架構，本專案實作了底層的車體運動學、動態網格地圖以及基於狀態還原的碰撞偵測，完美銜接控制理論與軟體工程的實務應用。

## Technical Features (核心技術特點)

### 1. Kinematics & Control (運動學與控制)
* **Differential Drive Model (差速驅動模型)**：將操作者的線性速度 (Linear Velocity) 與角速度 (Angular Velocity) 需求，精準映射至左右輪的獨立轉速。
* **Arcade Drive Mixing (街機混控演算法)**：透過混控邏輯，將前進與轉向的向量平滑疊加，實現絲滑的行駛手感。

### 2. Physics & Mathematics (物理與數學引擎)
* **State Rollback Collision (狀態還原碰撞偵測)**：捨棄高耗能的複雜多邊形運算，採用高效的頂點預測 (Vertex Prediction)。當車體頂點在下一幀 (Next Frame) 即將踩入障礙物網格時，瞬間觸發座標還原，達成完美的阻擋穿透效果。
* **Coordinate Mapping (座標映射)**：利用矩陣轉換，將作業系統的視窗像素座標 (Pixel Coordinates) 即時轉換為受縮放影響的真實世界座標 (World Coordinates)。

### 3. Software Engineering (軟體工程架構)
* **Decoupled Configuration (外部參數解耦)**：實體參數（如軸距、輪距、車體大小）皆透過讀取外部 `config.txt` 動態生成。修改車體物理特性完全不需重新編譯 (Recompile) 程式碼。
* **Sparse Map Structure (稀疏地圖結構)**：以 `std::set` 取代傳統的二維陣列來儲存無限網格 (Infinite Grid) 狀態，大幅降低記憶體開銷，並為未來的 A* 或 DWA 導航演算法鋪路。
* **Viewport Separation (視埠分離技術)**：利用虛擬攝影機 (`sf::View`) 切割物理模擬區與 UI 工具區，確保地圖縮放時不會干擾系統介面。

## Operations & Controls (操作指南)

### AMR Movement (車體控制)
* `Up (上方向鍵)`：前進 (Forward)
* `Down (下方向鍵)`：後退 (Backward)
* `A` / `D`：原地或行進間左轉 / 右轉 (Turn Left / Right)

### Environment Editor (環境編輯器)
* **滑鼠滾輪 (Mouse Wheel)**：在左側模擬區上下滾動，可無段縮放 (Zoom In / Zoom Out) 物理世界的視野。
* **滑鼠左鍵 (Left Click)**：點擊模擬區內的任意網格，即可觸發 Grid Snapping (網格對齊) 並精準放置障礙物。

## Build Instructions (建置與編譯)

本專案採用 `Makefile` 進行依賴管理，需確保環境中已安裝 GCC/MinGW 編譯器與 SFML 3.0.0。

**1. Clone the repository:**
```bash
git clone [https://github.com/johnson931128/CtrlKine-AMR.git](https://github.com/johnson931128/CtrlKine-AMR.git)
cd CtrlKine-AMR
```

**2. Configure the physical parameters:**
編輯根目錄下的 `config.txt`，可自由調整 `wheelBase` (軸距) 或 `trackWidth` (輪距) 等參數。

**3. Compile and Run:**
於終端機執行以下指令進行編譯與啟動：
```bash
mingw32-make
.\build\CtrlKine-AMR.exe
```
## Future Roadmap
* [ ] LiDAR Simulation (光達感測模擬)：實作射線投射 (Raycasting) 以回傳環境距離點雲。
* [ ] Path Planning (路徑規劃)：導入 A* 全局導航演算法以及更進階的路徑規劃演算法。
* [ ] PID Controller (PID 控制器)：實作精準的軌跡追跡 (Trajectory Tracking) 控制。
* [ ] AMCL 定位系統