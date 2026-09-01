CXX = g++
# 更新 Include 路徑至之前成功的 SFML-3.0.0 資料夾
CXXFLAGS = -std=c++17 -Wall -Iinclude -I"C:/lib/SFML-3.0.0/include"

# 更新 Lib 路徑至之前成功的 SFML-3.0.0 資料夾
LDFLAGS = -L"C:/lib/SFML-3.0.0/lib"
LDLIBS = -lsfml-graphics -lsfml-window -lsfml-system

SRC_DIR = src
BUILD_DIR = build

SRC_DIRS = $(SRC_DIR) \
	$(SRC_DIR)/core \
	$(SRC_DIR)/editor \
	$(SRC_DIR)/map \
	$(SRC_DIR)/sensors \
	$(SRC_DIR)/navigation \
	$(SRC_DIR)/localization \
	$(SRC_DIR)/slam \
	$(SRC_DIR)/ui \
	$(SRC_DIR)/app
SRCS = $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.cpp))
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

TARGET = $(BUILD_DIR)/CtrlKine-AMR.exe

TEST_BUILD_DIR = $(BUILD_DIR)/tests
TEST_MAPPER_TARGET = $(TEST_BUILD_DIR)/CoordinateMapperTests.exe
TEST_MAP_TARGET = $(TEST_BUILD_DIR)/MapDataTests.exe
TEST_FILE_TARGET = $(TEST_BUILD_DIR)/MapDataFileTests.exe
TEST_PATH_TARGET = $(TEST_BUILD_DIR)/PathPlannerTests.exe
TEST_EXECUTION_TARGET = $(TEST_BUILD_DIR)/PathExecutionTests.exe
TEST_RUNTIME_TARGET = $(TEST_BUILD_DIR)/SimulatorRuntimeTests.exe
TEST_LOCALIZATION_SENSOR_TARGET = $(TEST_BUILD_DIR)/LocalizationSensorTests.exe
TEST_PARTICLE_FILTER_TARGET = $(TEST_BUILD_DIR)/ParticleFilterTests.exe
TEST_LOCALIZATION_INTEGRATION_TARGET = $(TEST_BUILD_DIR)/LocalizationIntegrationTests.exe
TEST_LOCALIZATION_CONFIG_TARGET = $(TEST_BUILD_DIR)/LocalizationConfigTests.exe
TEST_SLAM_OCCUPANCY_TARGET = $(TEST_BUILD_DIR)/SlamOccupancyGridTests.exe
TEST_SCAN_MATCHER_TARGET = $(TEST_BUILD_DIR)/CorrelativeScanMatcherTests.exe
TEST_SLAM_FRONTEND_TARGET = $(TEST_BUILD_DIR)/SlamFrontendTests.exe
TEST_SLAM_INTEGRATION_TARGET = $(TEST_BUILD_DIR)/SlamIntegrationTests.exe
TEST_LOCALIZATION_STRESS_TARGET = $(TEST_BUILD_DIR)/LocalizationStressTests.exe
TEST_SLAM_STRESS_TARGET = $(TEST_BUILD_DIR)/SlamStressTests.exe
LOCALIZATION_BENCHMARK_TARGET = $(TEST_BUILD_DIR)/LocalizationBenchmark.exe
SLAM_BENCHMARK_TARGET = $(TEST_BUILD_DIR)/SlamBenchmark.exe
TEST_TARGETS = $(TEST_MAPPER_TARGET) $(TEST_MAP_TARGET) $(TEST_FILE_TARGET) $(TEST_PATH_TARGET) $(TEST_EXECUTION_TARGET) $(TEST_RUNTIME_TARGET) $(TEST_LOCALIZATION_SENSOR_TARGET) $(TEST_PARTICLE_FILTER_TARGET) $(TEST_LOCALIZATION_INTEGRATION_TARGET) $(TEST_LOCALIZATION_CONFIG_TARGET) $(TEST_SLAM_OCCUPANCY_TARGET) $(TEST_SCAN_MATCHER_TARGET) $(TEST_SLAM_FRONTEND_TARGET) $(TEST_SLAM_INTEGRATION_TARGET)

all: $(TARGET)

test: $(TEST_TARGETS)
	@status=0; \
	$(TEST_MAPPER_TARGET) || status=1; \
	$(TEST_MAP_TARGET) || status=1; \
	$(TEST_FILE_TARGET) || status=1; \
	$(TEST_PATH_TARGET) || status=1; \
	$(TEST_EXECUTION_TARGET) || status=1; \
	$(TEST_RUNTIME_TARGET) || status=1; \
	$(TEST_LOCALIZATION_SENSOR_TARGET) || status=1; \
	$(TEST_PARTICLE_FILTER_TARGET) || status=1; \
	$(TEST_LOCALIZATION_INTEGRATION_TARGET) || status=1; \
	$(TEST_LOCALIZATION_CONFIG_TARGET) || status=1; \
	$(TEST_SLAM_OCCUPANCY_TARGET) || status=1; \
	$(TEST_SCAN_MATCHER_TARGET) || status=1; \
	$(TEST_SLAM_FRONTEND_TARGET) || status=1; \
	$(TEST_SLAM_INTEGRATION_TARGET) || status=1; \
	exit $$status

test-localization-stress: $(TEST_LOCALIZATION_STRESS_TARGET)
	$(TEST_LOCALIZATION_STRESS_TARGET)

test-slam-stress: $(TEST_SLAM_STRESS_TARGET)
	$(TEST_SLAM_STRESS_TARGET)

localization-benchmark: $(LOCALIZATION_BENCHMARK_TARGET)
	$(LOCALIZATION_BENCHMARK_TARGET)

slam-benchmark: $(SLAM_BENCHMARK_TARGET)
	$(SLAM_BENCHMARK_TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/CoordinateMapperTests.o: tests/CoordinateMapperTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapDataTests.o: tests/MapDataTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapDataFileTests.o: tests/MapDataFileTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/PathPlannerTests.o: tests/PathPlannerTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/PathExecutionTests.o: tests/PathExecutionTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SimulatorRuntimeTests.o: tests/SimulatorRuntimeTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/LocalizationSensorTests.o: tests/LocalizationSensorTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/ParticleFilterTests.o: tests/ParticleFilterTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/LocalizationIntegrationTests.o: tests/LocalizationIntegrationTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/LocalizationConfigTests.o: tests/LocalizationConfigTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SlamOccupancyGridTests.o: tests/SlamOccupancyGridTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/CorrelativeScanMatcherTests.o: tests/CorrelativeScanMatcherTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SlamFrontendTests.o: tests/SlamFrontendTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SlamIntegrationTests.o: tests/SlamIntegrationTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/LocalizationStressTests.o: tests/LocalizationStressTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SlamStressTests.o: tests/SlamStressTests.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/LocalizationBenchmark.o: tests/LocalizationBenchmark.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/SlamBenchmark.o: tests/SlamBenchmark.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -Itests -c $< -o $@

$(TEST_BUILD_DIR)/MapData.o: src/map/MapData.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_MAPPER_TARGET): $(TEST_BUILD_DIR)/CoordinateMapperTests.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_MAP_TARGET): $(TEST_BUILD_DIR)/MapDataTests.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_FILE_TARGET): $(TEST_BUILD_DIR)/MapDataFileTests.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_PATH_TARGET): $(TEST_BUILD_DIR)/PathPlannerTests.o $(TEST_BUILD_DIR)/PathPlanner.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_EXECUTION_TARGET): $(TEST_BUILD_DIR)/PathExecutionTests.o $(TEST_BUILD_DIR)/PathExecution.o $(TEST_BUILD_DIR)/AMR.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_RUNTIME_TARGET): $(TEST_BUILD_DIR)/SimulatorRuntimeTests.o $(BUILD_DIR)/app/Simulator.o $(BUILD_DIR)/ui/ApplicationLayout.o $(BUILD_DIR)/ui/EditorToolbar.o $(BUILD_DIR)/ui/InspectorPanel.o $(BUILD_DIR)/editor/Environment.o $(BUILD_DIR)/map/MapValidator.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/LocalizationConfig.o $(BUILD_DIR)/localization/LocalizationVisualization.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(BUILD_DIR)/localization/ParticleFilter.o $(BUILD_DIR)/slam/SlamFrontend.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o $(BUILD_DIR)/slam/SlamVisualization.o $(TEST_BUILD_DIR)/PathExecution.o $(TEST_BUILD_DIR)/PathPlanner.o $(TEST_BUILD_DIR)/MapData.o $(TEST_BUILD_DIR)/AMR.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_LOCALIZATION_SENSOR_TARGET): $(TEST_BUILD_DIR)/LocalizationSensorTests.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/LocalizationVisualization.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_PARTICLE_FILTER_TARGET): $(TEST_BUILD_DIR)/ParticleFilterTests.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/LocalizationVisualization.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/localization/ParticleFilter.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_LOCALIZATION_INTEGRATION_TARGET): $(TEST_BUILD_DIR)/LocalizationIntegrationTests.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(BUILD_DIR)/localization/ParticleFilter.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_LOCALIZATION_CONFIG_TARGET): $(TEST_BUILD_DIR)/LocalizationConfigTests.o $(BUILD_DIR)/localization/LocalizationConfig.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(BUILD_DIR)/localization/ParticleFilter.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_SLAM_OCCUPANCY_TARGET): $(TEST_BUILD_DIR)/SlamOccupancyGridTests.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_SCAN_MATCHER_TARGET): $(TEST_BUILD_DIR)/CorrelativeScanMatcherTests.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_SLAM_FRONTEND_TARGET): $(TEST_BUILD_DIR)/SlamFrontendTests.o $(BUILD_DIR)/slam/SlamFrontend.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o $(BUILD_DIR)/sensors/LidarSimulator.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_SLAM_INTEGRATION_TARGET): $(TEST_BUILD_DIR)/SlamIntegrationTests.o $(BUILD_DIR)/slam/SlamFrontend.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o $(BUILD_DIR)/slam/SlamVisualization.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/localization/ParticleFilter.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_LOCALIZATION_STRESS_TARGET): $(TEST_BUILD_DIR)/LocalizationStressTests.o $(BUILD_DIR)/localization/AmclLocalizer.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/localization/ParticleFilter.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_SLAM_STRESS_TARGET): $(TEST_BUILD_DIR)/SlamStressTests.o $(BUILD_DIR)/slam/SlamFrontend.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(LOCALIZATION_BENCHMARK_TARGET): $(TEST_BUILD_DIR)/LocalizationBenchmark.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/localization/MapLikelihoodField.o $(BUILD_DIR)/localization/ParticleFilter.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(SLAM_BENCHMARK_TARGET): $(TEST_BUILD_DIR)/SlamBenchmark.o $(BUILD_DIR)/slam/SlamFrontend.o $(BUILD_DIR)/slam/CorrelativeScanMatcher.o $(BUILD_DIR)/slam/SlamOccupancyGrid.o $(BUILD_DIR)/slam/OccupancyGridMapper.o $(BUILD_DIR)/sensors/LidarSimulator.o $(BUILD_DIR)/sensors/OdometrySimulator.o $(TEST_BUILD_DIR)/MapData.o
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(TEST_BUILD_DIR)/PathPlanner.o: src/navigation/PathPlanner.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/PathExecution.o: src/navigation/PathExecution.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/AMR.o: src/core/AMR.cpp
	@mkdir -p $(TEST_BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
