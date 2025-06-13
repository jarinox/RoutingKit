#include <routingkit/label.h>
#include "expect.h"

using namespace RoutingKit;
using namespace std;


int main(){
    const unsigned int CAR = 0;
    const unsigned int BUS = 1;
    const unsigned int BIKE = 2;
    const unsigned int PEDESTRIAN = 3;

    Label l1 = Label(0);
    
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
    Label restrictions = Label(0);
    restrictions.set_bit(true, BIKE);

    // Check if the label matches the restrictions
    EXPECT(l1.is_allowed(restrictions.get_label()) == true);

    // Set the road restriction to prohibit BIKE and allow CAR
    l1.set_bit(true, BIKE);
    l1.set_bit(false, CAR);

    // Now the label should not match the restrictions
    EXPECT(l1.is_allowed(restrictions.get_label()) == false);

    EXPECT(l1.is_allowed(l1.get_label()) == false);


    // Subset and superset checks
    Label l2 = Label(0);
    Label l3 = Label(0);

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
    

	return expect_failed;
}
