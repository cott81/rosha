/*
 * test_RecoveryManager.cpp
 *
 *  Created on: May 20, 2014
 *      Author: dominik
 */


// Bring in my package's API, which is what I'm testing
#include "../include/rosha_recovery_manager/RecoveryManager.h"
// Bring in gtest
#include <gtest/gtest.h>

/*
// Declare a test
TEST(TestSuite, testCase1)
{
//<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
  EXPECT_EQ(3, 3);
  EXPECT_TRUE(3 == 3);

}

// Declare another test
TEST(TestSuite, testCase2)
{
//<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
  EXPECT_EQ(3, 3);
  EXPECT_TRUE(3 == 3);
}
*/

TEST(RecoveryManager, dummyFunctionToTest)
{
  //create scenario to test
  RecoveryManager rm;
  EXPECT_EQ(3*5, rm.GetStupidValue(3));
}

TEST(RecoveryManager, dummyFunctionToTest2)
{
  //create scenario to test
  RecoveryManager rm;
  EXPECT_EQ(3*5, rm.GetStupidValue(3));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}


