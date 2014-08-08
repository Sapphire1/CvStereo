/*!
 * \file
 * \brief 
 * \author Łukasz Żmuda
 */

#ifndef TEST_HPP_
#define TEST_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/CameraInfo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Types/HomogMatrix.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>

namespace Processors {
namespace Test {


/*!
 * \class Test
 * \brief Test processor class.
 *
 * Test processor.
 */
class Test: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Test(const std::string & name = "Test");

	/*!
	 * Destructor
	 */
	virtual ~Test();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

   
	// Input data streams

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
	Base::DataStreamIn <cv::Mat> in_centerMassPoint;

	// Output data streams
	Base::DataStreamOut <cv::Mat> out_mask;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_filtered_cloud;

	// Properties
	//Base::Property<string> algorythm_type;
	//Base::Property<int> numberOfDisparities;

	// Handlers
	Base::EventHandler2 h_DepthTransformation;

	// Handlers
	void DepthTransformation();
};

} //: namespace DepthTransform
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Test", Processors::Test::Test)

#endif /* TEST_HPP_ */