# CtrlKine-AMR Code Structure Guide

這份文件是給目前專案的開發者導覽用，目的是快速說明每個 `.hpp` / `.cpp` 檔案的功能、定位，以及它和其他模組的關係。

如果你之後一陣子沒碰這個專案，先看這份通常就能很快回神。

## 1. 專案目前的分層概念

目前程式大致可以分成 4 層：

1. `main.cpp`
   程式入口，只負責啟動模擬器。

2. `Simulator`
   最上層協調者。負責視窗、事件、更新流程、繪製流程。

3. `Environment` / `MapData`
   地圖編輯與場景資料層。負責障礙物、工作區、起終點、編輯模式、座標系統。

4. `AMR`
   機器人本體。負責車體幾何、差速運動學、車體繪製、碰撞角點。

可以把它想成：

```txt
main
  -> Simulator
      -> Environment
          -> MapData
              -> CoordinateTypes
      -> AMR
```

## 2. 各檔案功能說明

### [src/main.cpp](C:/programing/SFML/CtrlKine-AMR/src/main.cpp)

**定位**

- 專案入口點。
- 建立 `Simulator` 物件，然後呼叫 `run()`。

**你可以把它理解成**

- 「按下執行後，第一個被呼叫的地方」。
- 這裡通常不放業務邏輯。

**未來適合放什麼**

- 幾乎不用加東西。
- 若之後有啟動參數、debug 模式、命令列設定，可以從這裡開始接。

---

### [include/Simulator.hpp](C:/programing/SFML/CtrlKine-AMR/include/Simulator.hpp)
### [src/Simulator.cpp](C:/programing/SFML/CtrlKine-AMR/src/Simulator.cpp)

**定位**

- 專案的主控制器。
- 負責整個應用程式每一幀的流程。

**主要責任**

- 建立 SFML 視窗。
- 維護 `UI View` 與 `Simulation View`。
- 處理鍵盤與滑鼠事件。
- 呼叫 `AMR.update()` 更新車體。
- 呼叫 `Environment.draw()` 與 `AMR.draw()` 繪圖。
- 做高層級碰撞檢查與 rollback。

**目前重要成員**

- `m_window`
  SFML 視窗。
- `m_uiView`
  UI 右側區域的視角。
- `m_simView`
  左側模擬區的世界視角。
- `m_amr`
  機器人本體。
- `m_env`
  地圖與編輯環境。
- `m_isPanning`, `m_lastPanPixel`
  用來處理拖曳平移視角。

**目前重要函式**

- `run()`
  遊戲迴圈 / 模擬迴圈。
- `processEvents()`
  處理滑鼠點擊、滾輪縮放、拖曳平移、模式切換。
- `update(float dt)`
  更新機器人運動，並檢查碰撞。
- `render()`
  負責畫面輸出。
- `handleEditorHotkeys(...)`
  用數字鍵切換編輯模式。

**它和別人的關係**

- 它不直接管理障礙物資料內容，而是把這件事交給 `Environment` / `MapData`。
- 它不直接處理 AMR 的幾何細節，而是交給 `AMR`。

**未來適合放什麼**

- 主流程控制。
- 模式列 UI。
- 路徑顯示開關。
- 全域快捷鍵。

**不適合放什麼**

- 障礙物如何儲存。
- A* 的細部演算法。
- 車體內部幾何運算。

---

### [include/Environment.hpp](C:/programing/SFML/CtrlKine-AMR/include/Environment.hpp)
### [src/Environment.cpp](C:/programing/SFML/CtrlKine-AMR/src/Environment.cpp)

**定位**

- 地圖編輯器的操作層。
- 介於 `Simulator` 和 `MapData` 之間。

**主要責任**

- 管理 `EditorMode`。
- 接收滑鼠左鍵對世界座標的操作。
- 依照模式決定要放障礙物、刪障礙物、設起點、設終點、畫工作區。
- 把 `MapData` 內容畫出來。

**目前的模式**

- `Select`
- `PlaceObstacle`
- `DeleteObstacle`
- `SetStartPose`
- `SetGoalPose`
- `DrawWorkZone`
- `PanView`

**目前重要概念**

- `m_map`
  真正的地圖資料。
- `m_editorMode`
  目前正在用哪個工具。
- `m_pendingZoneStart`
  畫工作區時，記錄第一次點擊的位置。

**重要函式**

- `handleLeftClick(worldPos)`
  編輯器左鍵的主要入口。
- `setEditorMode(...)`
  切換工具模式。
- `cancelActiveTool()`
  取消尚未完成的操作，例如畫到一半的工作區。
- `draw(...)`
  依序畫格線、邊界、工作區、障礙物、起終點。

**它和別人的關係**

- `Simulator` 只知道「我收到一個世界座標點」，然後把它交給 `Environment`。
- `Environment` 再決定要怎麼改 `MapData`。

**未來適合放什麼**

- 物件選取。
- Hover 預覽。
- 刪除模式高亮。
- 工作區拖曳編輯。
- Editor toolbar 的狀態對應。

**不適合放什麼**

- 真正的地圖資料儲存格式細節。
- 機器人運動學。

---

### [include/MapData.hpp](C:/programing/SFML/CtrlKine-AMR/include/MapData.hpp)
### [src/MapData.cpp](C:/programing/SFML/CtrlKine-AMR/src/MapData.cpp)

**定位**

- 地圖核心資料模型。
- 這是目前最重要的資料層之一。

**它代表什麼**

可以把它理解成：

```txt
MapData
├── world_boundary
├── grid_resolution
├── obstacles
├── work_zones
├── robot_start_pose
└── robot_goal_pose
```

**主要責任**

- 儲存障礙物。
- 儲存工作區。
- 儲存起點與終點。
- 提供世界邊界檢查。
- 持有 `CoordinateMapper`，統一 world/grid 轉換。

**重要成員**

- `m_mapper`
  管理 world <-> grid 轉換。
- `m_worldBoundary`
  地圖邊界。
- `m_obstacles`
  障礙物集合，現在用 `std::set<GridCoord>`。
- `m_workZones`
  工作區清單。
- `m_robotStartPose`, `m_robotGoalPose`
  起點與終點。

**重要函式**

- `addObstacle(...)`
- `removeObstacle(...)`
- `isObstacleAt(...)`
- `addWorkZone(...)`
- `containsWorldPoint(...)`
- `setRobotStartPose(...)`
- `setRobotGoalPose(...)`

**它和別人的關係**

- `Environment` 透過它寫入資料。
- `AMR` 的碰撞邏輯目前是經由 `Environment` 間接查它。
- 未來 A* 也會高度依賴這一層。

**未來適合放什麼**

- 存檔 / 讀檔格式。
- 導出 occupancy grid。
- 產生 A* 可走訪網格。
- 查詢某區域內有哪些障礙物。

**這一層很重要的原因**

- 以後如果沒有 `MapData`，你每加一個功能就會把畫圖資料、互動資料、演算法資料混在一起。
- 有了它，導航、碰撞、編輯器、存檔就比較能共用同一份真實資料。

---

### [include/CoordinateTypes.hpp](C:/programing/SFML/CtrlKine-AMR/include/CoordinateTypes.hpp)

**定位**

- 基礎座標與姿態型別定義。
- 幫整個專案統一世界座標與格點座標。

**目前內容**

- `GridCoord`
  離散格點座標，例如 `(col, row)`。
- `Pose2D`
  2D 位姿，包含位置與朝向。
- `CoordinateMapper`
  world 與 grid 的轉換器。

**主要責任**

- `worldToGrid(worldPos)`
- `gridToWorldTopLeft(coord)`
- `gridToWorldCenter(coord)`

**為什麼它重要**

- 這是之後做 A*、footprint collision、存檔載入時最常被依賴的基礎層。
- 如果這層穩，很多功能都能接得很乾淨。

**未來適合放什麼**

- Screen to world 的輔助工具。
- Grid rectangle / index utilities。
- 角度正規化工具。

---

### [include/AMR.hpp](C:/programing/SFML/CtrlKine-AMR/include/AMR.hpp)
### [src/AMR.cpp](C:/programing/SFML/CtrlKine-AMR/src/AMR.cpp)

**定位**

- 機器人車體模型。
- 負責機器人的狀態、幾何、運動學、繪製。

**主要責任**

- 保存車體位置與角度。
- 用差速模型更新位置。
- 計算四個角點。
- 畫出車身與輪子。

**重要資料**

- `AMRConfig`
  車體尺寸與顏色設定。
- `m_position`
  世界座標中的位置。
- `m_heading`
  車頭朝向。
- `m_bodyShape`, `m_wheels`
  SFML 的畫圖物件。

**重要函式**

- `update(dt, vL, vR)`
  差速車運動學更新。
- `draw(window)`
  繪製車體。
- `getCorners()`
  回傳四個角點，供碰撞檢查使用。

**它和別人的關係**

- `Simulator` 餵它左右輪速度。
- `Simulator` 取它的角點去檢查地圖碰撞。

**未來適合放什麼**

- footprint 改成更正式的幾何模型。
- 感測器掛點。
- LiDAR 相對位置。
- 速度限制、加速度限制。

**不適合放什麼**

- 地圖儲存。
- 編輯器互動。
- A* 尋路。

---

## 3. 目前的資料流

你可以先記下面這條主線：

```txt
滑鼠點擊
-> Simulator 取得 screen pixel
-> mapPixelToCoords 轉成 world coordinate
-> Environment 依 editor mode 解讀操作
-> MapData 寫入 obstacles / zones / poses
-> render 時再由 Environment 畫回畫面
```

而 AMR 的更新主線是：

```txt
鍵盤輸入
-> Simulator 算出左右輪速度
-> AMR.update()
-> Simulator 取 AMR corners
-> Environment / MapData 檢查碰撞
-> 若碰撞則 rollback
```

## 4. 目前每層該放什麼功能

### 若你要做 A*

- 地圖來源：`MapData`
- 座標轉換：`CoordinateTypes`
- 流程控制：`Simulator`
- 視覺化結果：`Environment`

### 若你要做存檔 / 讀檔

- 主要實作位置：`MapData`
- 呼叫入口：`Simulator` 或之後的 UI 按鈕事件

### 若你要做選取工具

- 主要實作位置：`Environment`
- 被選中的真實資料：`MapData`

### 若你要做更正式的碰撞

- 車體幾何來源：`AMR`
- 地圖障礙來源：`MapData`
- 協調檢查流程：`Simulator`

## 5. 建議你閱讀程式的順序

如果你想快速跟上，建議照這個順序看：

1. [src/main.cpp](C:/programing/SFML/CtrlKine-AMR/src/main.cpp)
2. [include/Simulator.hpp](C:/programing/SFML/CtrlKine-AMR/include/Simulator.hpp)
3. [src/Simulator.cpp](C:/programing/SFML/CtrlKine-AMR/src/Simulator.cpp)
4. [include/Environment.hpp](C:/programing/SFML/CtrlKine-AMR/include/Environment.hpp)
5. [src/Environment.cpp](C:/programing/SFML/CtrlKine-AMR/src/Environment.cpp)
6. [include/MapData.hpp](C:/programing/SFML/CtrlKine-AMR/include/MapData.hpp)
7. [src/MapData.cpp](C:/programing/SFML/CtrlKine-AMR/src/MapData.cpp)
8. [include/CoordinateTypes.hpp](C:/programing/SFML/CtrlKine-AMR/include/CoordinateTypes.hpp)
9. [include/AMR.hpp](C:/programing/SFML/CtrlKine-AMR/include/AMR.hpp)
10. [src/AMR.cpp](C:/programing/SFML/CtrlKine-AMR/src/AMR.cpp)

這樣讀會比較順，因為你會先看到「整體流程」，再往下看到「資料與細節」。

## 6. 目前專案的一句話總結

現在的 CtrlKine-AMR 可以這樣理解：

> `Simulator` 是總控台，`Environment` 是地圖編輯器操作層，`MapData` 是地圖真實資料，`CoordinateTypes` 是座標基礎，`AMR` 是機器人本體。

如果你之後想，我下一步可以再幫你補第二份文件，專門畫成：

- 類別關係圖
- 事件流程圖
- 「我要新增功能時該改哪裡」對照表

那份會更像 onboard 文件。  
