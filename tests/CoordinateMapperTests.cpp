#include "CoordinateTypes.hpp"

#include <cmath>

#include "TestSupport.hpp"

namespace {
bool sameGrid(const GridCoord& actual, int col, int row) {
    return actual.col == col && actual.row == row;
}

bool samePoint(const sf::Vector2f& actual, float x, float y) {
    return std::fabs(actual.x - x) < 0.0001f && std::fabs(actual.y - y) < 0.0001f;
}
}

int main() {
    TestSuite suite;
    CoordinateMapper mapper(50.0f);

    runTest(suite, "COORD-001", "GridCoord axis fields", [] {
        const GridCoord coord{2, 3};
        return coord.col == 2 && coord.row == 3 && GridCoord{2, 3} == coord;
    });
    runTest(suite, "COORD-002", "worldToGrid floor semantics", [&] {
        return sameGrid(mapper.worldToGrid(sf::Vector2f(0.0f, 0.0f)), 0, 0)
            && sameGrid(mapper.worldToGrid(sf::Vector2f(49.9f, 49.9f)), 0, 0)
            && sameGrid(mapper.worldToGrid(sf::Vector2f(50.0f, 50.0f)), 1, 1)
            && sameGrid(mapper.worldToGrid(sf::Vector2f(-0.1f, -0.1f)), -1, -1);
    });
    runTest(suite, "COORD-003", "gridToWorldTopLeft", [&] {
        return samePoint(mapper.gridToWorldTopLeft(GridCoord{2, 3}), 100.0f, 150.0f);
    });
    runTest(suite, "COORD-004", "gridToWorldCenter", [&] {
        return samePoint(mapper.gridToWorldCenter(GridCoord{2, 3}), 125.0f, 175.0f);
    });
    runTest(suite, "COORD-005", "CoordinateMapper rejects non-positive resolution", [&] {
        mapper.setGridResolution(0.0f);
        const bool zeroRejected = mapper.getGridResolution() == 50.0f;
        mapper.setGridResolution(-10.0f);
        return zeroRejected && mapper.getGridResolution() == 50.0f;
    });
    runTest(suite, "COORD-005", "CoordinateMapper constructor rejects non-positive resolution", [] {
        return CoordinateMapper(0.0f).getGridResolution() > 0.0f
            && CoordinateMapper(-1.0f).getGridResolution() > 0.0f;
    });

    return suite.exitCode();
}
