#pragma once

#include <cmath>
#include <exception>
#include <iostream>
#include <string>

class TestSuite {
public:
    void expect(bool condition, const std::string& requirement, const std::string& detail) {
        if (condition) {
            ++m_passed;
            std::cout << "PASS [" << requirement << "] " << detail << "\n" << std::flush;
        } else {
            ++m_failed;
            std::cout << "FAIL [" << requirement << "] " << detail << "\n" << std::flush;
        }
    }

    int exitCode() const {
        std::cout << "\nSummary: " << m_passed << " PASS, " << m_failed << " FAIL\n";
        return m_failed == 0 ? 0 : 1;
    }

private:
    int m_passed = 0;
    int m_failed = 0;
};

template <typename Function>
void runTest(TestSuite& suite, const std::string& requirement, const std::string& detail, Function&& function) {
    try {
        suite.expect(function(), requirement, detail);
    } catch (const std::exception& exception) {
        suite.expect(false, requirement, detail + " (exception: " + exception.what() + ")");
    } catch (...) {
        suite.expect(false, requirement, detail + " (unknown exception)");
    }
}
