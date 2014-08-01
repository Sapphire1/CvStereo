/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef STEREOWRITER_HPP_
#define STEREOWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

namespace Processors {
namespace StereoWriter {

/*!
 * \class StereoWriter
 * \brief StereoWriter processor class.
 *
 * 
 */
class StereoWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	StereoWriter(const std::string & name = "StereoWriter");

	/*!
	 * Destructor
	 */
	virtual ~StereoWriter();

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
	Base::DataStreamIn<cv::Mat> in_rgb_stereo;
	Base::DataStreamIn<cv::Mat> in_depth_xyz;

	// Output data streams

	// Handlers
	Base::EventHandler2 h_trigger;
	Base::EventHandler2 h_onSave;

	// Properties
	Base::Property<std::string> dir;
	Base::Property<std::string> filename;
	
	// Handlers
	void trigger();
	void onSave();
	
	
	bool save;

};

} //: namespace StereoWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StereoWriter", Processors::StereoWriter::StereoWriter)

#endif /* STEREOWRITER_HPP_ */
