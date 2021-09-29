#include "TestManager.hpp"

void TestManager::registerTest(test* _T) {
    TestManager::TEST_REGISTRY.emplace_back(_T);
}

void TestManager::runTests(int iterations) {
    for(test* _t : TestManager::TEST_REGISTRY) {
        _t->TEST(iterations);
    }
}