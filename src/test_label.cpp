#include <routingkit/label.h>
#include "expect.h"

using namespace RoutingKit;
using namespace std;


int main(){
    const unsigned int CAR = 0;
    const unsigned int BUS = 1;
    const unsigned int BIKE = 2;
    const unsigned int PEDESTRIAN = 3;

    Label l1 = Label();
    
    // Prohibition of CAR, BUS, and PEDESTRIAN
    // Allow BIKE
    l1.set_bit(true, CAR);
    l1.set_bit(true, BUS);
    l1.set_bit(true, PEDESTRIAN);
    l1.set_bit(false, BIKE);

    EXPECT_CMP(l1.get_bit(CAR), ==, true);
    EXPECT_CMP(l1.get_bit(BUS), ==, true);
    EXPECT_CMP(l1.get_bit(BIKE), ==, false);
    EXPECT_CMP(l1.get_bit(PEDESTRIAN), ==, true);
    EXPECT_CMP(l1.get_bit(4), ==, false);

    // Restriction label to check, whether BIKE is allowed
    Label restrictions = Label();
    restrictions.set_bit(true, BIKE);

    // Check if the label matches the restrictions
    EXPECT(l1.is_allowed(restrictions) == true);

    // Set the road restriction to prohibit BIKE and allow CAR
    l1.set_bit(true, BIKE);
    l1.set_bit(false, CAR);

    // Now the label should not match the restrictions
    EXPECT(l1.is_allowed(restrictions) == false);

    EXPECT(l1.is_allowed(l1) == false);


    // Subset and superset checks
    Label l2 = Label();
    Label l3 = Label();

    l2.set_bit(true, CAR);
    l2.set_bit(true, BUS);
    l2.set_bit(false, BIKE);
    l2.set_bit(false, PEDESTRIAN);
    l3.set_bit(true, CAR);
    l3.set_bit(true, BUS);
    l3.set_bit(false, BIKE);
    l3.set_bit(false, PEDESTRIAN);
    
    // Both labels are equal, so they are subsets and supersets of each other
    EXPECT(l2.is_subset_of(l3) == true);
    EXPECT(l3.is_subset_of(l2) == true);
    EXPECT(l3.is_superset_of(l2) == true);
    EXPECT(l2.is_superset_of(l3) == true);

    // Now change l2 to include BIKE which is not in l3
    l2.set_bit(true, BIKE);
    EXPECT(l2.is_subset_of(l3) == false);
    EXPECT(l3.is_subset_of(l2) == true);
    EXPECT(l2.is_superset_of(l3) == true);
    EXPECT(l3.is_superset_of(l2) == false);
    
    // Union and intersection checks
    Label l4 = l2.unite(l3);
    EXPECT(l4.get_bit(CAR) == true);
    EXPECT(l4.get_bit(BUS) == true);
    EXPECT(l4.get_bit(BIKE) == true);
    EXPECT(l4.get_bit(PEDESTRIAN) == false);

    Label l5 = l2.intersect(l3);
    EXPECT(l5.get_bit(CAR) == true);
    EXPECT(l5.get_bit(BUS) == true);
    EXPECT(l5.get_bit(BIKE) == false);
    EXPECT(l5.get_bit(PEDESTRIAN) == false);

    // Fully restricted labels and inversion
    Label l6 = Label::fully_restricted();
    Label l7 = Label::fully_restricted();
    l7.invert();
    Label l8 = Label();
    for(unsigned i = 0; i < 64; ++i){
        EXPECT(l6.get_bit(i) == true);
        EXPECT(l7.get_bit(i) == false);
        l8.set_bit(false, i);
    }

	return expect_failed;
}
