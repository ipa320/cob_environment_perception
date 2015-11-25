#include <cob_3d_mapping_geometry_map_v2/types/object.h>
#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <gtest/gtest.h>

using namespace cob_3d_geometry_map;

Eigen::Affine3f random_transformation() {
	return Eigen::Translation3f(Eigen::Vector3f::Random())*Eigen::AngleAxisf(0.234, Eigen::Vector3f::Random().normalized());
}

template<class T>
void random_volume(T &vol, const int num=10) {
	vol.pose() = cast(random_transformation());
	vol._bb().setEmpty();
	for(int i=0; i<num; i++)
		vol._bb().extend(Eigen::Vector3f::Random());
}

// Declare a test
TEST(ObjectVolume, overlaps)
{
	Object::ContextPtr ctxt;
	
	//negative samples
	for(size_t i=0; i<1000000; i++) {
		ObjectVolume vol1(ctxt), vol2(ctxt);
		random_volume(vol1, 0);
		random_volume(vol2, 0);
		
		const Eigen::Vector3f n = Eigen::Vector3f::Random().normalized();
		for(int i=0; i<20; i++) {
			Eigen::Vector3f v = Eigen::Vector3f::Random();
			if(v.dot(n)>0)
				vol1._bb().extend(cast(vol1.pose()).inverse()*(v));
			else
				vol2._bb().extend(cast(vol2.pose()).inverse()*v);
		}
		
		if(!vol1.bb_in_context().intersection(vol2.bb_in_context()).isEmpty()) continue;
		
		EXPECT_FALSE(vol1.overlaps(vol2));
		EXPECT_FALSE(vol2.overlaps(vol1));
	}
	
	//positive samples
	for(size_t i=0; i<1000000; i++) {
		ObjectVolume vol1(ctxt), vol2(ctxt);
		random_volume(vol1);
		random_volume(vol2);
		vol2._bb().extend(cast(vol2.pose()).inverse()* cast(vol1.pose())*vol1._bb().sample());
		
		EXPECT_TRUE(vol1.overlaps(vol2));
		EXPECT_TRUE(vol2.overlaps(vol1));
	}
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
