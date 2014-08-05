/*!
 * \file
 * \brief 
 * \author Łukasz Żmuda
 */

#ifndef CUBICMASKEXTRACTOR_HPP_
#define CUBICMASKEXTRACTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Types/CameraInfo.hpp>
#include "Types/HomogMatrix.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>





namespace Processors {
namespace CubicMaskExtractor {

/*!
 * \class CubicMaskExtractor
 * \brief CubicMaskExtractor processor class.
 *
 * CubicMaskExtractor processor.
 */
class CubicMaskExtractor: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CubicMaskExtractor(const std::string & name = "CubicMaskExtractor");

	/*!
	 * Destructor
	 */
	virtual ~CubicMaskExtractor();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();
	
	Base::Property<double> prop_XAxisMin_treshold;
	Base::Property<double> prop_XAxisMax_treshold;
	Base::Property<double> prop_YAxisMin_treshold;
	Base::Property<double> prop_YAxisMax_treshold;
	Base::Property<double> prop_ZAxisMin_treshold;
	Base::Property<double> prop_ZAxisMax_treshold;

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
	Base::DataStreamIn <cv::Point3d> in_centerMassPoint;
	// Output data streams
	Base::DataStreamOut <cv::Mat> out_mask;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_filtered_cloud;
	
	// Handlers
	Base::EventHandler2 h_filter;

	
	// Handlers
	void filter();
	
	cv::Mat tmp_img;

};

} //: namespace CubicMaskExtractor
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CubicMaskExtractor", Processors::CubicMaskExtractor::CubicMaskExtractor)

#endif /* CUBICMASKEXTRACTOR_HPP_ */
