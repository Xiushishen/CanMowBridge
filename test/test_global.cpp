#include "gtest/gtest.h"

class GlobalTest : public testing::Environment {
public:
    void SetUp() {
        std::cout << "SetUp" << std::endl;
    }
    void TearDown() {
        std::cout << "TearDown" << std::endl;
    }
};

int main(int argc, char **argv) {
    std::cout << "Running main() from gtest_main.cc\n";
    
    testing::InitGoogleTest(&argc, argv);
    testing::Environment* env = new GlobalTest();
    testing::AddGlobalTestEnvironment(env);
    return RUN_ALL_TESTS();
}
