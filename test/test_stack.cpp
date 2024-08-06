#include <iostream>
#include <stack>

#include "gtest/gtest.h"

class MyStackTest2 : public testing::Test {
protected:
    virtual void SetUp() {
        st.push(34);
        st.push(28);
        st.push(56);
    }

    virtual void TearDown() {}

    // 构造函数，也可以做一部分SetUp函数的工作
    MyStackTest2() {
        std::cout << "MyStackTest is constructed." << std::endl;
        st.push(22);
    }
    // 析构函数，也可以做一部分TearDown函数的工作
    ~MyStackTest2() {
        std::cout << "Destructed MyStackTest." << std::endl;
    }

    std::stack<int> st;
};

// 如果使用了测试装置，就必须使用TEST_F(测试装置类， 自定义名称)
TEST_F(MyStackTest2, testPop) {
    // 在这里会自动构造一个MyStack的实例，并调用SetUp函数
    st.pop();
    EXPECT_EQ(28, st.top());
    // 在这里会调用TearDown函数
}

// testPop2和testPop来自同一个测试装置，但是来自不同的测试装置实例，两个实例是相互独立的个体
TEST_F(MyStackTest2, testPop2) {
    // 在这里会自动构造一个MyStack的实例，并调用SetUp函数
    int val = st.top();
    st.pop();
    EXPECT_EQ(56, val);
    // 在这里会调用TearDown函数
}
