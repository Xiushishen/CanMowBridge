#include <iostream>
#include <stack>

#include "gtest/gtest.h"

using namespace std;

//测试实例1
TEST(MyStackTest1, simpletest1) {
    stack<int> st;
    st.push(4);
    EXPECT_EQ(4, st.top()); //使用Google Test宏进行测试（非致命断言）
}

//测试实例2s
TEST(MyStackTest1, simpletest2) {
    stack<int> st;
    st.push(9);
    st.push(28);
    
    int val = st.top();
    st.pop();
    EXPECT_EQ(28, val); // 28等于val则测试通过（非致命断言）
    EXPECT_NE(0, val); // 0不等于val则测试通过（非致命断言）
    EXPECT_GT(29, val); // 29大于val则测试通过（非致命断言）
    EXPECT_GE(29, val); // 29大于等于val则测试通过（非致命断言）
    EXPECT_TRUE(val == 28) << "val is not equal to 28"; // val == 28结果为false，输出后面日志（非致命断言）
}
