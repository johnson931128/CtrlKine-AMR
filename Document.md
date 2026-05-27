# CtrlKine-AMR User Document

這份文件記錄目前應用程式已經可以做的操作、功能，以及對應的快捷鍵。

## Current Features

- `Select` mode
  目前是基礎模式，主要作為返回中性操作狀態使用。
- `Obstacle` mode
  在格線上放置障礙物。
- `Erase` mode
  刪除格線上的障礙物。
- `Set Start` mode
  設定地圖上的起點位置。
- `Set Goal` mode
  設定地圖上的終點位置。
- `Work Zone` mode
  以拖曳方式建立工作區域，並自動吸附到 grid corner。
- `Pan` mode
  用滑鼠拖曳平移視角。
- `Zoom`
  在模擬畫布上使用滑鼠滾輪放大或縮小。
- `Save / Load Map`
  可將目前地圖存成文字檔，並重新載入。

## Toolbar

上方工具列目前是可點擊的，也可以用快捷鍵切換：

- `S` 或 `1`
  Select
- `O` 或 `2`
  Obstacle
- `E` 或 `3`
  Erase
- `T` 或 `4`
  Set Start
- `G` 或 `5`
  Set Goal
- `Z` 或 `6`
  Work Zone
- `P` 或 `7`
  Pan

## Mouse Controls

- `Left Click`
  依目前模式執行動作。
- `Left Drag` in `Pan`
  平移模擬畫布。
- `Left Drag` in `Work Zone`
  從起點拖曳到終點建立矩形工作區。
- `Mouse Wheel`
  在模擬畫布區域內縮放視角。

## Keyboard Controls

- `Up / Down`
  控制 AMR 前進 / 後退。
- `A / D`
  控制 AMR 左轉 / 右轉。
- `Esc`
  取消當前工具操作。
  如果目前在 `Work Zone` 模式，也會回到 `Select`。
- `F5`
  儲存地圖到 `saved_map.txt`
- `F9`
  從 `saved_map.txt` 載入地圖

## Inspector

右側 `Inspector` 目前會顯示：

- `Cursor`
  游標當前的 world 座標與 grid 座標。
- `Map Stats`
  Grid resolution、障礙物數量、work zone 數量、start/goal 是否已設定。
- `Robot State`
  AMR 的位置、朝向角度、目前控制模式。
- `Status`
  最近一次存檔、讀檔、模式切換等狀態訊息。

## Save / Load Format

目前地圖會存成純文字檔 `saved_map.txt`。

選這種格式的原因：

- 人眼可讀，除錯方便
- 不需要額外套件
- 很容易擴充更多欄位

目前會儲存的內容：

- `grid_resolution`
- `world_boundary`
- `start_pose`
- `goal_pose`
- `obstacles`
- `work_zones`

這不是圖片格式，所以不會存畫面外觀，而是存「真正的地圖資料」。

## Current Workflow

一個典型使用流程如下：

1. 用 `O` 放置障礙物
2. 用 `T` 設定起點
3. 用 `G` 設定終點
4. 用 `Z` 拖曳建立工作區
5. 用 `F5` 儲存地圖
6. 之後用 `F9` 載入地圖繼續編輯

## Current Limitations

目前還沒做完，但已經在 TODO 裡的項目：

- `Select` 尚未真的選取物件
- `Inspector` 還不能顯示 selected object 詳細資訊
- `Delete` 還沒有支援刪除 work zone / start / goal
- `Start / Goal` 還沒有方向設定
- `Clear Map` / `Reset View` 還沒加入
