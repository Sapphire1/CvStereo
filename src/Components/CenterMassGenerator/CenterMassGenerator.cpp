/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "CenterMassGenerator.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace CenterMassGenerator {

CenterMassGenerator::CenterMassGenerator(const std::string & name) :
		Base::Component(name)
{



}

CenterMassGenerator::~CenterMassGenerator() {

}

void CenterMassGenerator::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_image_xyz", &in_image_xyz);
	registerStream("out_centerMassPoint", &out_centerMassPoint);

	// Register handlers
	h_CenterMassGeneration.setup(boost::bind(&CenterMassGenerator::CenterMassGeneration, this));
	registerHandler("CenterMassGeneration", &h_CenterMassGeneration);
	addDependency("CenterMassGeneration", &in_image_xyz);

}

bool CenterMassGenerator::onInit() {

	return true;
}

bool CenterMassGenerator::onFinish() {
	return true;
}

bool CenterMassGenerator::onStop() {
	return true;
}

bool CenterMassGenerator::onStart() {
	return true;
}

void CenterMassGenerator::CenterMassGeneration() {
	try{
	  cv::Point3d point = cv::Point3d(1.1,1.1,1.1);
	  LOG(LINFO) << "Writing mask to data stream";
	  out_centerMassPoint.write(point);
	 
	  
	} catch (...)
	{
		LOG(LERROR) << "CenterMassGeneration: Error occured in processing input";
	}
	
}


} //: namespace TransformParameterGenerator
} //: namespace Processors