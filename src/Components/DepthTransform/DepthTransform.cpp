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
	registerStream("in_image_xyz", &in_image_xyz);
	registerStream("out_image_xyz", &out_image_xyz);

	// Register handlers
	h_DepthTransformation.setup(boost::bind(&DepthTransform::DepthTransformation, this));
	registerHandler("DepthTransformation", &h_DepthTransformation);
	addDependency("DepthTransformation", &in_image_xyz);

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
	  cv::Mat depth_image = in_image_xyz.read();
	  cv::Mat tvec = in_tvec.read();
	  cv::Mat rvec = in_rvec.read();
	  cv::Mat transformed_image;
	  cv::Size depth_size = depth_image.size();
	  std::cout<<"tvec : "<<tvec<<"\n";
	  std::cout<<"rvec : "<<rvec<<"\n";
	  cv::Mat tmp_img;// = cv::Mat(depth_image);
	  tmp_img.create(depth_size, CV_32FC3);
	  //////////////////////////////////////////////////////////////////////////
	  
	// iteracja po obrazie glebi, podstawienie pod maske odpowiednich pikseli
        if (depth_image.isContinuous() && tmp_img.isContinuous()) {
         depth_size.width *= depth_size.height;
         depth_size.height = 1;
        }
        
            float newX, newY, newZ;
            depth_size.width *= 3;
	
	    
	    ////////////////////////////////////////
	    /*
	    HomogMatrix hm;

	    hm.elements[0][0] = rvec.at<double>(0,0);
	    hm.elements[0][1] = rvec.at<double>(0,1);
	    hm.elements[0][2] = rvec.at<double>(0,2);
	    hm.elements[0][3] = 0;
	    
	    hm.elements[1][0] = rvec.at<double>(1,0);
	    hm.elements[1][1] = rvec.at<double>(1,1);
	    hm.elements[1][2] = rvec.at<double>(1,2);
	    hm.elements[1][3] = 0;
	    
	    hm.elements[2][0] = rvec.at<double>(2,0);
	    hm.elements[2][1] = rvec.at<double>(2,1);
	    hm.elements[2][2] = rvec.at<double>(2,2);
	    hm.elements[2][3] = 0;

	    hm.elements[3][0] = 0;
	    hm.elements[3][1] = 0;
	    hm.elements[3][2] = 0;
	    hm.elements[3][3] = 1;
	    
	    */
	
	    ///////////////////////////////////////
	    
        for (int i = 0; i < depth_size.height; i++)
        {
            const float* depth_ptr = depth_image.ptr <float> (i);
	    float* depth_ptr_tmp = tmp_img.ptr <float> (i);
            int j, k = 0;
	    int val = 0;
	    
            for (j = 0; j < depth_size.width; j += 3)
            {
		 // get x, y, z from depth_image
		 float x = depth_ptr[j];
                 float y = depth_ptr[j + 1];
		 float z= depth_ptr[j + 2];
		 
		  if(z==10000)
		  {
		      depth_ptr_tmp[j]=x;
		      depth_ptr_tmp[j + 1]=y;
		      depth_ptr_tmp[j + 2]=z;
		      continue;
		  }

		 
		// float x=1,y=5,z=10;
		//std::cout<<"Bylo: "<<x<<" "<<y<<" "<<z<<"\n";
		
		x=x-tvec.at<double>(0,0);
		y=y-tvec.at<double>(1,0);
		z=z-tvec.at<double>(2,0);
		//std::cout<<"Jest: "<<x<<" "<<y<<" "<<z<<"\n";

		rvec=rvec.t();
      		newX = x*rvec.at<double>(0,0) + y*rvec.at<double>(0,1)+z*rvec.at<double>(0,2);
		newY = x*rvec.at<double>(1,0) + y*rvec.at<double>(1,1)+z*rvec.at<double>(1,2);
		newZ = x*rvec.at<double>(2,0) + y*rvec.at<double>(2,1)+z*rvec.at<double>(2,2);
		//std::cout<<"Jest po rotacji: "<<newX<<" "<<newY<<" "<<newZ<<"\n";

		depth_ptr_tmp[j]=newX;
                depth_ptr_tmp[j + 1]=newY;
                depth_ptr_tmp[j + 2]=newZ;
            }
        }
        /*
        for (int i = 0; i < depth_size.height; i++)
        {
	    float* depth_ptr_tmp = tmp_img.ptr <float> (i);
            int j, k = 0;
	    int val = 0;
	    
            for (j = 0; j < depth_size.width; j += 3)
            {
		std::cout << "Czytam: "<<depth_ptr_tmp[j]<<" "<<depth_ptr_tmp[j + 1]<<" "<<depth_ptr_tmp[j + 2]<<"\n";
            }
        }
        */
        
        
        
        
        
	  LOG(LINFO) << "Writing to data stream";
	  out_image_xyz.write(tmp_img);
	  
	} catch (...)
	{
		LOG(LERROR) << "DepthTransformation:Error occured in processing input";
	}
}


} //: namespace DepthTransform
} //: namespace Processors