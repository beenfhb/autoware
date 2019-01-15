#include <gnss/geo_pos_conv.hpp>
#include <ros/ros.h>
#include <cstdlib>
#include <gtest/gtest.h>

class TestSuite : public ::testing::Test {
public:
    TestSuite()
    {
    }
    ~TestSuite()
    {
    }
};


//using values from http://asp.ncm-git.co.jp/QuickConvert/BL2TM.aspx
TEST(TestSuite, llh_to_xyz){
    geo_pos_conv geo;

    geo.set_plane(9);
    geo.llh_to_xyz(35.72012160,139.75289851, 0);

    int x = static_cast<int>(geo.x());
    int y = static_cast<int>(geo.y());

    ASSERT_EQ(x,-31048 ) << "x[m] in japan plane rectangular coordinate should be " << -31048;
    ASSERT_EQ(y,-7277 ) << "y[m] in japan plane rectangular coordinate should be " << -7277;
}

TEST(TestSuite, llh_to_mgrs){
    geo_pos_conv geo;

    geo.set_plane(9);
    geo.llh_to_xyz(-55.707591, 49.762062, 0, true);

    int x = static_cast<int>(geo.x());
    int y = static_cast<int>(geo.y());

    ASSERT_EQ(x, 22212 ) << "x[m] in MGRS should be " << 22212;
    ASSERT_EQ(y, 25769 ) << "y[m] in MGRS should be " << 25769;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "TestNode");
    return RUN_ALL_TESTS();
}
