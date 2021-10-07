#pragma once
#ifndef TEST_MANAGER_HPP
#define TEST_MANAGER_HPP

#include <vector>
#include "Test.hpp"

namespace TestManager {

    static std::vector<test*> TEST_REGISTRY = {};

    void registerTest(test* _T);

    void runTests(int iterations);
}

#endif // TEST_MANAGER_HPP