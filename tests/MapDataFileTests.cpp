#include "MapData.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "TestSupport.hpp"

namespace {
const std::filesystem::path kTestDirectory = "build/test-data";

bool sameFloat(float lhs, float rhs) { return std::fabs(lhs - rhs) < 0.0001f; }
bool samePoint(const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
    return sameFloat(lhs.x, rhs.x) && sameFloat(lhs.y, rhs.y);
}
bool sameRect(const sf::FloatRect& lhs, const sf::FloatRect& rhs) {
    return samePoint(lhs.position, rhs.position) && samePoint(lhs.size, rhs.size);
}

MapData makeCompleteMap() {
    MapData map(25.0f);
    map.setWorldBoundary(sf::FloatRect(sf::Vector2f(-100.0f, -50.0f), sf::Vector2f(500.0f, 400.0f)));
    map.setRobotStartPose(Pose2D{sf::Vector2f(0.0f, 25.0f), 0.5f});
    map.setRobotGoalPose(Pose2D{sf::Vector2f(100.0f, 125.0f), 1.25f});
    map.addObstacle(GridCoord{0, 0});
    map.addObstacle(GridCoord{-1, 1});
    map.addWorkZone(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(50.0f, 75.0f)));
    map.addWorkZone(sf::FloatRect(sf::Vector2f(-50.0f, -25.0f), sf::Vector2f(25.0f, 25.0f)));
    return map;
}

bool sameMap(const MapData& lhs, const MapData& rhs) {
    if (!sameFloat(lhs.getGridResolution(), rhs.getGridResolution())
        || !sameRect(lhs.getWorldBoundary(), rhs.getWorldBoundary())
        || lhs.getObstacles() != rhs.getObstacles()
        || lhs.getWorkZones().size() != rhs.getWorkZones().size()
        || lhs.getRobotStartPose().has_value() != rhs.getRobotStartPose().has_value()
        || lhs.getRobotGoalPose().has_value() != rhs.getRobotGoalPose().has_value()) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.getWorkZones().size(); ++i) {
        if (!sameRect(lhs.getWorkZones()[i].bounds, rhs.getWorkZones()[i].bounds)) {
            return false;
        }
    }
    if (lhs.getRobotStartPose().has_value()
        && (!samePoint(lhs.getRobotStartPose()->position, rhs.getRobotStartPose()->position)
            || !sameFloat(lhs.getRobotStartPose()->heading, rhs.getRobotStartPose()->heading))) {
        return false;
    }
    if (lhs.getRobotGoalPose().has_value()
        && (!samePoint(lhs.getRobotGoalPose()->position, rhs.getRobotGoalPose()->position)
            || !sameFloat(lhs.getRobotGoalPose()->heading, rhs.getRobotGoalPose()->heading))) {
        return false;
    }
    return true;
}

bool writeFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }
    file << contents;
    return file.good();
}

bool loadFailsAtomically(const std::filesystem::path& path) {
    MapData map = makeCompleteMap();
    const MapData before = map;
    try {
        return !map.loadFromFile(path.string()) && sameMap(map, before);
    } catch (...) {
        return false;
    }
}

std::string validHeader() {
    return "grid_resolution 25\n"
           "world_boundary -100 -50 500 400\n"
           "start_pose 0 25 0.5\n"
           "goal_pose 100 125 1.25\n";
}
}

int main() {
    std::filesystem::create_directories(kTestDirectory);
    TestSuite suite;
    const auto completePath = kTestDirectory / "map_round_trip.txt";

    runTest(suite, "MAP-012", "save content includes all records", [&] {
        const MapData map = makeCompleteMap();
        if (!map.saveToFile(completePath.string())) {
            return false;
        }
        std::ifstream file(completePath);
        std::stringstream contents;
        contents << file.rdbuf();
        const std::string text = contents.str();
        return text.find("grid_resolution 25") != std::string::npos
            && text.find("world_boundary -100 -50 500 400") != std::string::npos
            && text.find("start_pose 0 25 0.5") != std::string::npos
            && text.find("goal_pose 100 125 1.25") != std::string::npos
            && text.find("obstacle_count 2") != std::string::npos
            && text.find("work_zone_count 2") != std::string::npos;
    });
    runTest(suite, "MAP-013", "save-load round trip", [&] {
        const MapData source = makeCompleteMap();
        MapData loaded;
        return source.saveToFile(completePath.string())
            && loaded.loadFromFile(completePath.string())
            && sameMap(source, loaded);
    });

    const std::string validRecords = validHeader()
        + "obstacle_count 1\n"
          "obstacle 0 0\n"
          "work_zone_count 1\n"
          "work_zone 0 0 25 25\n";
    const std::vector<std::pair<std::string, std::string>> invalidFiles = {
        {"malformed_grid", "grid_resolution nope\n"},
        {"malformed_boundary", "grid_resolution 25\nworld_boundary -100 -50 bad 400\n"},
        {"malformed_pose", validRecords + "start_pose bad 1 2\n"},
        {"unknown_key", validRecords + "mystery 1 2\n"},
        {"missing_goal", "grid_resolution 25\nworld_boundary -100 -50 500 400\nstart_pose 0 25 0.5\nobstacle_count 0\nwork_zone_count 0\n"},
        {"missing_world_boundary", "grid_resolution 25\nstart_pose none\ngoal_pose none\nobstacle_count 0\nwork_zone_count 0\n"},
        {"obstacle_count_mismatch", validHeader() + "obstacle_count 2\nobstacle 0 0\nwork_zone_count 0\n"},
        {"zone_count_mismatch", validHeader() + "obstacle_count 0\nwork_zone_count 2\nwork_zone 0 0 25 25\n"},
        {"invalid_field_count", validHeader() + "obstacle_count 1 extra\nobstacle 0 0 extra\nwork_zone_count 0\n"},
        {"non_positive_resolution", "grid_resolution 0\nworld_boundary -100 -50 500 400\nstart_pose none\ngoal_pose none\nobstacle_count 0\nwork_zone_count 0\n"},
        {"non_positive_boundary", "grid_resolution 25\nworld_boundary -100 -50 0 400\nstart_pose none\ngoal_pose none\nobstacle_count 0\nwork_zone_count 0\n"},
        {"non_positive_zone", validHeader() + "obstacle_count 0\nwork_zone_count 1\nwork_zone 0 0 0 25\n"}
    };
    for (const auto& [name, contents] : invalidFiles) {
        const auto path = kTestDirectory / (name + ".txt");
        runTest(suite, "MAP-014", "invalid file rejected atomically: " + name, [&] {
            return writeFile(path, contents) && loadFailsAtomically(path);
        });
    }

    const auto emptyPosePath = kTestDirectory / "empty_poses.txt";
    runTest(suite, "MAP-012", "save content represents unset poses", [&] {
        MapData map;
        map.clear();
        if (!map.saveToFile(emptyPosePath.string())) {
            return false;
        }
        std::ifstream file(emptyPosePath);
        std::stringstream contents;
        contents << file.rdbuf();
        const std::string text = contents.str();
        return text.find("start_pose none") != std::string::npos
            && text.find("goal_pose none") != std::string::npos;
    });

    return suite.exitCode();
}
