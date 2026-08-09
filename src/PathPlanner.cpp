#include "PathPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <set>

PathResult PathPlanner::plan(const MapData& mapData) {
    PathResult result;
    result.message = "Path planner skeleton is ready. A* is not implemented yet.";

    const std::optional<GridCoord> startCell = getStartCell(mapData);
    if (!startCell.has_value()) {
        result.message = "Planning failed: start pose is missing.";
        return result;
    }

    const std::optional<GridCoord> goalCell = getGoalCell(mapData);
    if (!goalCell.has_value()) {
        result.message = "Planning failed: goal pose is missing.";
        return result;
    }

    if (isCellBlocked(mapData, *startCell)) {
        result.message = "Planning failed: start cell is blocked.";
        return result;
    }

    if (isCellBlocked(mapData, *goalCell)) {
        result.message = "Planning failed: goal cell is blocked.";
        return result;
    }

    std::set<GridCoord> openSet;
    std::map<GridCoord, GridCoord> cameFrom;
    std::map<GridCoord, float> gScore;
    std::map<GridCoord, float> fScore;

    gScore[*startCell] = 0.0f;
    fScore[*startCell] = heuristic(*startCell, *goalCell);
    openSet.insert(*startCell);

    while (!openSet.empty()) {
        auto currentIt = openSet.begin();
        for (auto candidateIt = openSet.begin(); candidateIt != openSet.end(); ++candidateIt) {
            if (fScore.at(*candidateIt) < fScore.at(*currentIt)) {
                currentIt = candidateIt;
            }
        }

        const GridCoord current = *currentIt;
        openSet.erase(currentIt);
        ++result.nodesExpanded;

        if (current == *goalCell) {
            result.path = reconstructPath(cameFrom, current);
            result.success = true;
            result.pathLength = static_cast<float>(result.path.size() - 1);
            result.message = "Path planning succeeded.";
            return result;
        }

        const float tentativeBaseGScore = gScore.at(current) + 1.0f;
        for (const GridCoord& neighbor : getNeighbors(mapData, current)) {
            const auto knownGScore = gScore.find(neighbor);
            if (knownGScore != gScore.end() && tentativeBaseGScore >= knownGScore->second) {
                continue;
            }

            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeBaseGScore;
            fScore[neighbor] = tentativeBaseGScore + heuristic(neighbor, *goalCell);
            openSet.insert(neighbor);
        }
    }

    // TODO(student):
    // 1. 建立 open set，裡面要能取出目前 fScore 最小的節點。
    // 2. 建立 cameFrom，記錄每個節點是從哪個前驅走過來的。
    // 3. 建立 gScore，記錄從 start 走到某節點的目前最佳成本。
    // 4. 建立 fScore，通常是 gScore + heuristic。
    // 5. 將 startCell 放入 open set，並初始化它的 gScore / fScore。
    // 6. 反覆從 open set 取出 fScore 最小的節點作為 current。
    // 7. 若 current == goalCell，呼叫 reconstructPath() 建立完整路徑。
    // 8. 否則展開 current 的 neighbors。
    // 9. 對每個 neighbor 計算 tentative gScore。
    // 10. 若找到更好的成本，更新 cameFrom / gScore / fScore。
    // 11. 若 neighbor 尚未在 open set，將它加入。
    // 12. 若 open set 清空仍找不到 goal，回傳失敗結果。

    // TODO(student):
    // 這裡可以先用 getNeighbors() 與 heuristic() 做小單元測試，
    // 確認地圖資料、格點轉換與 blocked 判斷都正確，再開始補 A* 主流程。

    return result;
}

std::optional<GridCoord> PathPlanner::getStartCell(const MapData& mapData) {
    // TODO(student):
    // 1. 從 MapData 取得 StartPose。
    // 2. 將 StartPose 的 world position 轉成 GridCoord。
    // 3. 若 StartPose 不存在，回傳 std::nullopt。
    if (!mapData.getRobotStartPose().has_value()) {
        return std::nullopt;
    }

    return mapData.getMapper().worldToGrid(mapData.getRobotStartPose()->position);
}

std::optional<GridCoord> PathPlanner::getGoalCell(const MapData& mapData) {
    // TODO(student):
    // 1. 從 MapData 取得 GoalPose。
    // 2. 將 GoalPose 的 world position 轉成 GridCoord。
    // 3. 若 GoalPose 不存在，回傳 std::nullopt。
    if (!mapData.getRobotGoalPose().has_value()) {
        return std::nullopt;
    }

    return mapData.getMapper().worldToGrid(mapData.getRobotGoalPose()->position);
}

bool PathPlanner::isCellBlocked(const MapData& mapData, const GridCoord& cell) {
    // TODO(student):
    // 1. 判斷 cell 是否落在世界邊界外。
    // 2. 判斷 cell 是否被 obstacle 佔用。
    // 3. 之後若要加入 footprint / forbidden zone，也可以在這裡擴充。
    const sf::Vector2f cellCenter = mapData.getMapper().gridToWorldCenter(cell);
    if (!mapData.containsWorldPoint(cellCenter)) {
        return true;
    }

    return mapData.isObstacleAt(cell);
}

std::vector<GridCoord> PathPlanner::getNeighbors(const MapData& mapData, const GridCoord& cell) {
    // TODO(student):
    // 1. 先決定要用 4-neighbor 還是 8-neighbor。
    // 2. 產生 candidate cells。
    // 3. 過濾掉 blocked cell。
    // 4. 若之後要加入 corner cutting 規則，可以在這裡處理。
    const std::vector<GridCoord> candidateCells = {
        GridCoord{cell.col + 1, cell.row},
        GridCoord{cell.col - 1, cell.row},
        GridCoord{cell.col, cell.row + 1},
        GridCoord{cell.col, cell.row - 1}
    };

    std::vector<GridCoord> neighbors;
    for (const auto& candidate : candidateCells) {
        if (!isCellBlocked(mapData, candidate)) {
            neighbors.push_back(candidate);
        }
    }

    return neighbors;
}

float PathPlanner::heuristic(const GridCoord& from, const GridCoord& to) {
    // TODO(student):
    // 1. 依照你的鄰居定義選擇 heuristic。
    // 2. 若用 4-neighbor，常見選擇是 Manhattan distance。
    // 3. 若用 8-neighbor，常見選擇是 Octile distance 或 Euclidean distance。
    return static_cast<float>(std::abs(from.col - to.col) + std::abs(from.row - to.row));
}

std::vector<GridCoord> PathPlanner::reconstructPath(
    const std::map<GridCoord, GridCoord>& cameFrom,
    const GridCoord& current
) {
    // TODO(student):
    // 1. 從 goal 開始，沿著 cameFrom 一路回推到 start。
    // 2. 將回推出來的節點順序反轉，變成 start -> goal。
    // 3. 回傳完整 path，之後 Simulator 才能畫 path polyline。
    std::vector<GridCoord> path;
    GridCoord cursor = current;

    path.push_back(cursor);
    while (true) {
        const auto it = cameFrom.find(cursor);
        if (it == cameFrom.end()) {
            break;
        }

        cursor = it->second;
        path.push_back(cursor);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
