/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "StereoSequence.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace StereoSequence {

StereoSequence::StereoSequence(const std::string & name) :
		Base::Component(name) , 
		prop_directory("prop_directory", std::string(".")), 
		prop_pattern("prop_pattern", std::string(".*\\.")), 
		prop_sort("prop_sort", true), 
		prop_triggered("prop_triggered", false), 
		prop_iterate("prop_iterate", true), 
		prop_loop("prop_loop", false) {
	registerProperty(prop_directory);
	registerProperty(prop_pattern);
	registerProperty(prop_sort);
	registerProperty(prop_triggered);
	registerProperty(prop_iterate);
	registerProperty(prop_loop);

}

StereoSequence::~StereoSequence() {
}


void StereoSequence::prepareInterface() {
    // Register streams.
	registerStream("out_rgb_stereo", &out_rgb_stereo);
	registerStream("out_depth_xyz", &out_depth_xyz);
    registerStream("in_load_next_image_trigger", &in_load_next_image_trigger);

    // Register handlers - loads image, NULL dependency.
	h_onLoadImage.setup(boost::bind(&StereoSequence::onLoadImage, this) );
	registerHandler("onLoadImage", &h_onLoadImage);
    addDependency("onLoadImage", NULL);

    // Register handlers - next image, can be triggered manually (from GUI) or by new data present in_load_next_image_trigger dataport.
    // 1st version - manually.
    h_onLoadNextImage.setup(boost::bind(&StereoSequence::onLoadNextImage, this) );
    registerHandler("Next image", &h_onLoadNextImage);

    // 2nd version - external tritter.
    h_onTriggeredLoadNextImage.setup(boost::bind(&StereoSequence::onTriggeredLoadNextImage, this) );
    registerHandler("onTriggeredLoadNextImage", &h_onTriggeredLoadNextImage);
    addDependency("onTriggeredLoadNextImage", &in_load_next_image_trigger);


    // Register handlers - reloads sequence, triggered manually.
    h_onSequenceReload.setup(boost::bind(&StereoSequence::onSequenceReload, this) );
    registerHandler("Reload sequence", &h_onSequenceReload);

    // Register handlers - trigger (load frame), triggered manually.
    h_onRefreshImage.setup(boost::bind(&StereoSequence::onRefreshImage, this) );
    registerHandler("Refresh image", &h_onRefreshImage);

}

bool StereoSequence::onInit() {
	CLOG(LTRACE) << "Sequence::initialize\n";

	if (!findFiles()) {
		CLOG(LERROR) << name() << ": There are no files matching regex "
				<< prop_pattern << " in " << prop_directory;
		return false;
	}

	return true;
}

bool StereoSequence::onFinish() {
	CLOG(LTRACE) << "Sequence::finish\n";

	return true;
}

void StereoSequence::onLoadImage() {
	CLOG(LDEBUG) << "Sequence::onLoadImage";

	// Check triggering mode.
	if (prop_triggered && !trig)
		return;
	trig = false;
	// Check iterate mode.
	if (prop_iterate)
		frame++;

	if (frame <0)
		frame = 0;

	// Check the size of the dataset.
	if (frame >= files_rgb_stereo.size()) {
		if (prop_loop) {
			frame = 0;
			CLOG(LINFO) << name() << ": loop";
			// TODO: endOfSequence->raise();
		} else {
			frame = files_rgb_stereo.size() -1;
			CLOG(LINFO) << name() << ": end of sequence";
			// TODO: endOfSequence->raise();
		}

	}

	CLOG(LTRACE) << "Sequence: reading image " << files_rgb_stereo[frame];
	CLOG(LTRACE) << "Sequence: reading image " << files_depth_xyz[frame];
	try {
		img = cv::imread(files_rgb_stereo[frame], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
		
	} catch (...) {
		CLOG(LWARNING) << name() << ": image reading failed! ["
				<< files_rgb_stereo[frame] << "]";
	}
	
	try {
		cv::FileStorage file(files_depth_xyz[frame], cv::FileStorage::READ);
		file["img"] >> img_depth_xyz;
		
	} catch (...) {
		CLOG(LWARNING) << name() << ": image reading failed! ["
				<< files_depth_xyz[frame] << "]";
	}

	// Write image to the output port.
	out_rgb_stereo.write(img);
	out_depth_xyz.write(img_depth_xyz);
}


void StereoSequence::onRefreshImage(){
    CLOG(LDEBUG) << "Sequence::onRefreshImage - trigger";
    trig = true;
}


void StereoSequence::onTriggeredLoadNextImage(){
    CLOG(LDEBUG) << "Sequence::onTriggeredLoadNextImage - next image from the sequence will be loaded";
    in_load_next_image_trigger.read();
    frame++;
}


void StereoSequence::onLoadNextImage(){
	CLOG(LDEBUG) << "Sequence::onLoadNextImage - next image from the sequence will be loaded";
	frame++;
}


void StereoSequence::onSequenceReload() {
	// Set first frame index number.
	if (prop_iterate)
		frame = -1;
	else
		frame = 0;
	// Try to load new sequence.
	if (!findFiles()) {
		CLOG(LERROR) << name() << ": There are no files matching regex "
				<< prop_pattern << " in " << prop_directory;
		frame = -1;
	}
}

bool StereoSequence::onStop() {
	return true;
}

bool StereoSequence::onStart() {
	return true;
}

bool StereoSequence::findFiles() {
	files_depth_xyz.clear();
	files_rgb_stereo.clear();
	
	std::string pattern_depth_xyz = std::string(prop_pattern) + std::string("yml") ;
	std::string pattern_rgb_stereo = std::string(prop_pattern) + std::string("png") ;
	
	files_depth_xyz = Utils::searchFiles(prop_directory, pattern_depth_xyz);
	files_rgb_stereo = Utils::searchFiles(prop_directory, pattern_rgb_stereo);

	if (prop_sort){
		std::sort(files_depth_xyz.begin(), files_depth_xyz.end());
		std::sort(files_rgb_stereo.begin(), files_rgb_stereo.end());
	}

	CLOG(LINFO) << "Sequence loaded.";
	BOOST_FOREACH(std::string fname, files_depth_xyz)
		CLOG(LINFO) << fname;
	BOOST_FOREACH(std::string fname, files_rgb_stereo)
		CLOG(LINFO) << fname;

	return (!files_rgb_stereo.empty() && !files_depth_xyz.empty());
}


} //: namespace StereoSequence
} //: namespace Processors
