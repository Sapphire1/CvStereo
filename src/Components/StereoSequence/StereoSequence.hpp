/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef STEREOSEQUENCE_HPP_
#define STEREOSEQUENCE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace StereoSequence {

/*!
 * \class StereoSequence
 * \brief StereoSequence processor class.
 *
 * 
 */
class StereoSequence: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	StereoSequence(const std::string & name = "StereoSequence");

	/*!
	 * Destructor
	 */
	virtual ~StereoSequence();

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

	bool findFiles();
	
	/// list of files in sequence
	std::vector<std::string> files_rgb_stereo;
	std::vector<std::string> files_depth_xyz;
	
	/// current frame
	cv::Mat img;
	cv::Mat img_depth_xyz;

	/// Index of current frame.
	int frame;

    /// Flag indicating whether the image was already loaded or not.
	bool trig;	
		
    /// Trigger - used for loading next image in case of several sequences present.
    Base::DataStreamIn<Base::UnitType> in_load_next_image_trigger;

	/// Output data stream
	Base::DataStreamOut<cv::Mat> out_img;


	/// Event handler.
//    Base::EventHandler<Sequence> h_on_load_next_image_trigger;

    /*!
     * Event handler function - moves image index to the next frame of the sequence.
     */
    void onLoadNextImage();

    /// Event handler - moves image index to the next frame of the sequence.
    Base::EventHandler2 h_onLoadNextImage;


    /*!
     * Event handler function - moves image index to the next frame of the sequence, externally triggered version.
     */
    void onTriggeredLoadNextImage();

    /// Event handler - moves image index to the next frame of the sequence, externally triggered version.
    Base::EventHandler2 h_onTriggeredLoadNextImage;


    /*!
	 * Event handler function - loads image from the sequence.
	 */
	void onLoadImage();

	/// Event handler - loads image from the sequence.
	Base::EventHandler2 h_onLoadImage;

	/*!
	 * Event handler function - reload the sequence.
	 */
	void onSequenceReload();

	/// Event handler - reload the sequence.
	Base::EventHandler2 h_onSequenceReload;

    /*!
     * Event handler function - triggers image refresh.
     */
    void onRefreshImage();

    /// Event handler - refreshes the image (old or new, depending on the rest of settings.
    Base::EventHandler2 h_onRefreshImage;	
	// Input data streams

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_rgb_stereo;
	Base::DataStreamOut<cv::Mat> out_depth_xyz;

	// Handlers

	// Properties
	Base::Property<std::string> prop_directory;
	Base::Property<std::string> prop_pattern;
	Base::Property<bool> prop_sort;
	Base::Property<bool> prop_triggered;
	Base::Property<bool> prop_iterate;
	Base::Property<bool> prop_loop;

	
	// Handlers

};

} //: namespace StereoSequence
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StereoSequence", Processors::StereoSequence::StereoSequence)

#endif /* STEREOSEQUENCE_HPP_ */
