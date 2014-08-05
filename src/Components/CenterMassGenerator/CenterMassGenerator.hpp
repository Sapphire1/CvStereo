/*!
 * \file
 * \brief 
 * \author Łukasz Żmuda
 */

#ifndef CENTERMASSGENERATION_HPP_
#define CENTERMASSGENERATION_HPP_

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

namespace Processors {
namespace CenterMassGenerator {


/*!
 * \class CenterMassGenerator
 * \brief CenterMassGenerator processor class.
 *
 * CenterMassGenerator processor.
 */
class CenterMassGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CenterMassGenerator(const std::string & name = "CenterMassGenerator");

	/*!
	 * Destructor
	 */
	virtual ~CenterMassGenerator();

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

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;

	// Output data streams
	Base::DataStreamOut <cv::Point3d> out_centerMassPoint;


	// Properties
	//Base::Property<string> algorythm_type;
	//Base::Property<int> numberOfDisparities;

	// Handlers
	Base::EventHandler2 h_CenterMassGeneration;

	// Handlers
	void CenterMassGeneration();
};

} //: namespace CenterMassGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CenterMassGenerator", Processors::CenterMassGenerator::CenterMassGenerator)

#endif /* CENTERMASSGENERATION_HPP_ */