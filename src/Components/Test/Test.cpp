/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "Test.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace Test {
  
using Types::HomogMatrix;

Test::Test(const std::string & name) :
		Base::Component(name)
{



}

Test::~Test() {

}

void Test::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_centerMassPoint", &in_centerMassPoint);

	registerStream("out_filtered_cloud", &out_filtered_cloud);

	// Register handlers
	h_DepthTransformation.setup(boost::bind(&Test::DepthTransformation, this));
	registerHandler("DepthTransformation", &h_DepthTransformation);
	addDependency("DepthTransformation", &in_cloud_xyz);

}

bool Test::onInit() {

	return true;
}

bool Test::onFinish() {
	return true;
}

bool Test::onStop() {
	return true;
}

bool Test::onStart() {
	return true;
}

void Test::DepthTransformation() {
	try{

	  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = in_cloud_xyz.read();
	  cv::Mat centerMassPoint = in_centerMassPoint.read();
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredX (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredY (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredZ (new pcl::PointCloud<pcl::PointXYZ>);
	  // Create the filtering object
	  //pcl::PassThrough<pcl::PointXYZ> pass;
	  
	} catch (...)
	{
		LOG(LERROR) << "DepthTransformation:Error occured in processing input";
	}
}


} //: namespace Test
} //: namespace Processors