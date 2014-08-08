/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "TransformParameterGenerator.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace TransformParameterGenerator {
  
using Types::HomogMatrix;

TransformParameterGenerator::TransformParameterGenerator(const std::string & name) :
		Base::Component(name)
{



}

TransformParameterGenerator::~TransformParameterGenerator() {

}

void TransformParameterGenerator::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_image_xyz", &in_image_xyz);
	registerStream("out_rvec", &out_rvec);
	registerStream("out_tvec", &out_tvec);

	// Register handlers
	h_TransformMaticesGenerator.setup(boost::bind(&TransformParameterGenerator::TransformMaticesGenerator, this));
	registerHandler("TransformMaticesGenerator", &h_TransformMaticesGenerator);
	addDependency("TransformMaticesGenerator", &in_image_xyz);

}

bool TransformParameterGenerator::onInit() {

	return true;
}

bool TransformParameterGenerator::onFinish() {
	return true;
}

bool TransformParameterGenerator::onStop() {
	return true;
}

bool TransformParameterGenerator::onStart() {
	return true;
}

void TransformParameterGenerator::TransformMaticesGenerator() {
	try{
	  // 90 degrees rotation around Z axis and moving by 1 meter along X axis
	  cv::Mat tvec = (cv::Mat_<double>(3,1) << 1, 0, 0);
	  cv::Mat rvec = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 1, 0); //180 X
	  
	  LOG(LINFO) << "Writing mask to data stream";
	  out_tvec.write(tvec);
	  out_rvec.write(rvec);
	 
	  
	} catch (...)
	{
		LOG(LERROR) << "TransformMaticesGenerator: Error occured in processing input";
	}
	
}


} //: namespace TransformParameterGenerator
} //: namespace Processors