/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "StereoWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace StereoWriter {

StereoWriter::StereoWriter(const std::string & name) :
		Base::Component(name),
		dir("save.directory", std::string("./") ),
		filename("save.filename", std::string("test") )   {

}

StereoWriter::~StereoWriter() {
}

void StereoWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_rgb_stereo", &in_rgb_stereo);
	registerStream("in_depth_xyz", &in_depth_xyz);
	// Register handlers
	h_trigger.setup(boost::bind(&StereoWriter::trigger, this));
	registerHandler("trigger", &h_trigger);
	addDependency("trigger", &in_rgb_stereo);
	addDependency("trigger", &in_depth_xyz);
	h_onSave.setup(boost::bind(&StereoWriter::onSave, this));
	registerHandler("onSave", &h_onSave);

}

bool StereoWriter::onInit() {
	save = false;
	return true;
}

bool StereoWriter::onFinish() {
	return true;
}

bool StereoWriter::onStop() {
	return true;
}

bool StereoWriter::onStart() {
	return true;
}

void StereoWriter::trigger() {
	save = true;
}

void StereoWriter::onSave() {
	CLOG(LTRACE) <<  "StereoWriter::onSave" << endl;
	if(save){
		cv::Mat stereo_rgb = in_rgb_stereo.read();
		cv::Mat depth_xyz = in_depth_xyz.read();
		
		// Change compression to lowest.
	    vector<int> param;
	    param.push_back(CV_IMWRITE_PNG_COMPRESSION);
	    param.push_back(0); // MAX_MEM_LEVEL = 9 
		// Save image.
		std::string tmp_name = std::string(dir) + std::string("/") + std::string(filename) + std::string(".png");
		imwrite(tmp_name, stereo_rgb, param);
		CLOG(LINFO) <<  "Saved to file " << tmp_name <<std::endl;
		
		tmp_name = std::string(dir) + std::string("/") + std::string(filename) + std::string(".yml");
		cv::FileStorage file(tmp_name, cv::FileStorage::WRITE);
		file << "img" << depth_xyz;
				
		
		save = false;
	}
	else{
		CLOG(LTRACE) << "StereoWriter::onSave failed" << endl;
	}
}



} //: namespace StereoWriter
} //: namespace Processors
