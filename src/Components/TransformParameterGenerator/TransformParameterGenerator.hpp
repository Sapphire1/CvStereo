/*!
 * \file
 * \brief 
 * \author Łukasz Żmuda
 */

#ifndef TRANSFORMPARAMETERGENERATOR_HPP_
#define TRANSFORMPARAMETERGENERATOR_HPP_

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
//#include <Types/Objects3D/Object3D.hpp>
//#include "/home/lzmuda/DCL/DCL_CvBasic/src/Types/HomogMatrix.hpp"
#include "Types/HomogMatrix.hpp"

namespace Processors {
namespace TransformParameterGenerator {


/*!
 * \class TransformParameterGenerator
 * \brief TransformParameterGenerator processor class.
 *
 * TransformParameterGenerator processor.
 */
class TransformParameterGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	TransformParameterGenerator(const std::string & name = "TransformParameterGenerator");

	/*!
	 * Destructor
	 */
	virtual ~TransformParameterGenerator();

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

    /*!
     * Generate Q matrix for 2 projection matrices
     */
   
	// Input data streams

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyz;

	// Output data streams
	Base::DataStreamOut <cv::Mat> out_rvec;
	Base::DataStreamOut <cv::Mat> out_tvec;


	// Properties
	//Base::Property<string> algorythm_type;
	//Base::Property<int> numberOfDisparities;

	// Handlers
	Base::EventHandler2 h_TransformMaticesGenerator;

	// Handlers
	void TransformMaticesGenerator();
};

} //: namespace TransformParameterGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("TransformParameterGenerator", Processors::TransformParameterGenerator::TransformParameterGenerator)

#endif /* TRANSFORMPARAMETERGENERATOR_HPP_ */