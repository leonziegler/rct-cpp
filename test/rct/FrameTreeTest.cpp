#include <stdexcept>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rct/impl/FrameTreeSimple.h"

using namespace std;
using namespace testing;
using namespace rct;

TEST(FrameTreeTest, testAll) {


    FrameTreeSimple tree(boost::posix_time::time_duration(0,0,30));

    EXPECT_EQ(0, tree.getFrameStrings().size());

}
