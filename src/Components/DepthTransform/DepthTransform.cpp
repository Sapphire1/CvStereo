/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "DepthTransform.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthTransform {
  
using Types::HomogMatrix;

DepthTransform::DepthTransform(const std::string & name) :
		Base::Component(name)
{



}

DepthTransform::~DepthTransform() {

}

void DepthTransform::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_tvec", &in_tvec);
	registerStream("in_rvec", &in_rvec);
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("out_cloud_xyz", &out_cloud_xyz);

	// Register handlers
	h_DepthTransformation.setup(boost::bind(&DepthTransform::DepthTransformation, this));
	registerHandler("DepthTransformation", &h_DepthTransformation);
	addDependency("DepthTransformation", &in_cloud_xyz);

}

bool DepthTransform::onInit() {

	return true;
}

bool DepthTransform::onFinish() {
	return true;
}

bool DepthTransform::onStop() {
	return true;
}

bool DepthTransform::onStart() {
	return true;
}

void DepthTransform::DepthTransformation() {
	try{
	  HomogMatrix hm;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = in_cloud_xyz.read();
	  cv::Mat tvec = in_tvec.read();
	  cv::Mat rvec = in_rvec.read();
	  
	  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointXYZ transformedPoint;
	  ////////////zaslepka////////////////////////////
	  /*
	  // Fill in the input_cloud data
	  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  input_cloud->width  = 5;
	  input_cloud->height = 1;
	  input_cloud->points.resize (input_cloud->width * input_cloud->height);

	  for (size_t i = 0; i < input_cloud->points.size(); ++i)
	  {
	    input_cloud->points[i].x = 2.0f+i;
	    input_cloud->points[i].y = 4.0f+i;
	    input_cloud->points[i].z = 6.0f+i;
	  }
	  */
	   //////////////koniec zaslepki///////////////////////
  
	  /*
	  std::cout << "Cloud before transformating: " << std::endl;
	  for (size_t i = 0; i < input_cloud->points.size(); ++i)
	    std::cout << "    " << input_cloud->points[i].x << " "
                        << input_cloud->points[i].y << " "
                        << input_cloud->points[i].z << std::endl;
	  
	  */		
	  //s = (p-t)*R'		
	  //R=R'
	  rvec = rvec.t();
	  
	  
	  Eigen::Matrix4f transformation_matrix_translation = Eigen::Matrix4f::Identity();
	  Eigen::Matrix4f transformation_matrix_rotation = Eigen::Matrix4f::Identity();
	  
	  
	  // translation
	  transformation_matrix_translation (0,0) = 1;
	  transformation_matrix_translation (0,1) = 0;
	  transformation_matrix_translation (0,2) = 0;
	  transformation_matrix_translation (0,3) = -tvec.at<double>(0,0);
	  
	  transformation_matrix_translation (1,0) = 0;
	  transformation_matrix_translation (1,1) = 1;
	  transformation_matrix_translation (1,2) = 0;
	  transformation_matrix_translation (1,3) = -tvec.at<double>(1,0);
	  
	  transformation_matrix_translation (2,0) = 0;
	  transformation_matrix_translation (2,1) = 0;
	  transformation_matrix_translation (2,2) = 1;
	  transformation_matrix_translation (2,3) = -tvec.at<double>(2,0);
	  
	  transformation_matrix_translation (3,0) = 0;
	  transformation_matrix_translation (3,1) = 0;
	  transformation_matrix_translation (3,2) = 0;
	  transformation_matrix_translation (3,3) = 1;
	  
	   std::cout<< "Rotation matrix :\n";
	  std::cout<< "    | "<< transformation_matrix_translation (0,0)<<" "<< transformation_matrix_translation (0,1) << " "<<transformation_matrix_translation (0,2)<<" | \n" ;
	  std::cout<< "R = | "<< transformation_matrix_translation (1,0)<<" "<< transformation_matrix_translation (1,1) << " "<<transformation_matrix_translation (1,2)<<" | \n" ;
	  std::cout<< "    | "<< transformation_matrix_translation (2,0)<<" "<< transformation_matrix_translation (2,1) << " "<<transformation_matrix_translation (2,2)<<" | \n" ;
	  std::cout<< "Translation vector :\n";
	  std::cout<< "t = "<< transformation_matrix_translation (0,3) << " "<< transformation_matrix_translation (1,3) << " " << transformation_matrix_translation (2,3)<<"\n\n";

	  // transformation
	  pcl::transformPointCloud (*input_cloud, *transformed_cloud, transformation_matrix_translation);

	  
	  input_cloud = transformed_cloud;
	 

	  // rotation
	  transformation_matrix_rotation (0,0) = rvec.at<double>(0,0);
	  transformation_matrix_rotation (0,1) = rvec.at<double>(0,1);
	  transformation_matrix_rotation (0,2) = rvec.at<double>(0,2);
	  transformation_matrix_rotation (0,3) = 0;
	  
	  transformation_matrix_rotation (1,0) = rvec.at<double>(1,0);
	  transformation_matrix_rotation (1,1) = rvec.at<double>(1,1);
	  transformation_matrix_rotation (1,2) = rvec.at<double>(1,2);
	  transformation_matrix_rotation (1,3) = 0;
	  
	  transformation_matrix_rotation (2,0) = rvec.at<double>(2,0);
	  transformation_matrix_rotation (2,1) = rvec.at<double>(2,1);
	  transformation_matrix_rotation (2,2) = rvec.at<double>(2,2);
	  transformation_matrix_rotation (2,3) = 0;
	  
	  transformation_matrix_rotation (3,0) = 0;
	  transformation_matrix_rotation (3,1) = 0;
	  transformation_matrix_rotation (3,2) = 0;
	  transformation_matrix_rotation (3,3) = 1;
	  

	  std::cout<< "Rotation matrix :\n";
	  std::cout<< "    | "<< transformation_matrix_rotation (0,0)<<" "<< transformation_matrix_rotation (0,1) << " "<<transformation_matrix_rotation (0,2)<<" | \n" ;
	  std::cout<< "R = | "<< transformation_matrix_rotation (1,0)<<" "<< transformation_matrix_rotation (1,1) << " "<<transformation_matrix_rotation (1,2)<<" | \n" ;
	  std::cout<< "    | "<< transformation_matrix_rotation (2,0)<<" "<< transformation_matrix_rotation (2,1) << " "<<transformation_matrix_rotation (2,2)<<" | \n" ;
	  std::cout<< "Translation vector :\n";
	  std::cout<< "t = "<< transformation_matrix_rotation (0,3) << " "<< transformation_matrix_rotation (1,3) << " " << transformation_matrix_rotation (2,3)<<"\n\n";

	  // transformation
	  pcl::transformPointCloud (*input_cloud, *transformed_cloud, transformation_matrix_rotation);

	  /*
	  std::cout << "Cloud after transformating: " << std::endl;
	  for (size_t i = 0; i < transformed_cloud->points.size(); ++i)
	    std::cout << "    " << transformed_cloud->points[i].x << " "
                        << transformed_cloud->points[i].y << " "
                        << transformed_cloud->points[i].z << std::endl;
	  */
	  LOG(LINFO) << "Writing to data stream";
	  out_cloud_xyz.write(transformed_cloud);
	  
	} catch (...)
	{
		LOG(LERROR) << "DepthTransformation:Error occured in processing input";
	}
}


} //: namespace DepthTransform
} //: namespace Processors