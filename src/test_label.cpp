#include <routingkit/label.h>
#include <gtest/gtest.h>

using namespace std;

const unsigned int CAR = 0;
const unsigned int BUS = 1;
const unsigned int BIKE = 2;
const unsigned int PEDESTRIAN = 3;

TEST(LabelTest, SetAndGetBits) {
    Label l1;
    l1.set_bit(true, CAR);
    l1.set_bit(true, BUS);
    l1.set_bit(true, PEDESTRIAN);
    l1.set_bit(false, BIKE);

    EXPECT_EQ(l1.get_bit(CAR), true);
    EXPECT_EQ(l1.get_bit(BUS), true);
    EXPECT_EQ(l1.get_bit(BIKE), false);
    EXPECT_EQ(l1.get_bit(PEDESTRIAN), true);
    EXPECT_EQ(l1.get_bit(4), false);
}

TEST(LabelTest, IsAllowed) {
    Label l1;
    l1.set_bit(true, CAR);
    l1.set_bit(true, BUS);
    l1.set_bit(true, PEDESTRIAN);
    l1.set_bit(false, BIKE);

    Label restrictions;
    restrictions.set_bit(true, BIKE);
    EXPECT_TRUE(l1.is_allowed(restrictions));

    l1.set_bit(true, BIKE);
    l1.set_bit(false, CAR);
    EXPECT_FALSE(l1.is_allowed(restrictions));
    EXPECT_FALSE(l1.is_allowed(l1));
}

TEST(LabelTest, SubsetAndSuperset) {
    Label l2;
    Label l3;
    l2.set_bit(true, CAR);
    l2.set_bit(true, BUS);
    l2.set_bit(false, BIKE);
    l2.set_bit(false, PEDESTRIAN);
    l3.set_bit(true, CAR);
    l3.set_bit(true, BUS);
    l3.set_bit(false, BIKE);
    l3.set_bit(false, PEDESTRIAN);

    EXPECT_TRUE(l2.is_subset_of(l3));
    EXPECT_TRUE(l3.is_subset_of(l2));
    EXPECT_TRUE(l3.is_superset_of(l2));
    EXPECT_TRUE(l2.is_superset_of(l3));

    l2.set_bit(true, BIKE);
    EXPECT_FALSE(l2.is_subset_of(l3));
    EXPECT_TRUE(l3.is_subset_of(l2));
    EXPECT_TRUE(l2.is_superset_of(l3));
    EXPECT_FALSE(l3.is_superset_of(l2));
}

TEST(LabelTest, UnionAndIntersection) {
    Label l2;
    Label l3;
    l2.set_bit(true, CAR);
    l2.set_bit(true, BUS);
    l2.set_bit(true, BIKE);
    l2.set_bit(false, PEDESTRIAN);
    l3.set_bit(true, CAR);
    l3.set_bit(true, BUS);
    l3.set_bit(false, BIKE);
    l3.set_bit(false, PEDESTRIAN);

    Label l4 = l2.unite(l3);
    EXPECT_TRUE(l4.get_bit(CAR));
    EXPECT_TRUE(l4.get_bit(BUS));
    EXPECT_TRUE(l4.get_bit(BIKE));
    EXPECT_FALSE(l4.get_bit(PEDESTRIAN));

    Label l5 = l2.intersect(l3);
    EXPECT_TRUE(l5.get_bit(CAR));
    EXPECT_TRUE(l5.get_bit(BUS));
    EXPECT_FALSE(l5.get_bit(BIKE));
    EXPECT_FALSE(l5.get_bit(PEDESTRIAN));
}

TEST(LabelTest, FullyRestrictedAndInversion) {
    Label l6 = Label::fully_restricted();
    Label l7 = Label::fully_restricted().invert();
    Label l8;
    Label l5;
    l5.set_bit(true, CAR);
    l5.set_bit(true, BUS);
    l5.set_bit(false, BIKE);
    l5.set_bit(false, PEDESTRIAN);
    Label l9 = l5;
    l9.invert();
    
    for(unsigned i = 0; i < 64; ++i){
        EXPECT_TRUE(l6.get_bit(i));
        EXPECT_FALSE(l7.get_bit(i));
        l8.set_bit(false, i);
        EXPECT_EQ(!l9.get_bit(i), l5.get_bit(i));
    }
}
