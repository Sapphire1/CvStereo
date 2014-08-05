/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include <memory>
#include <string>

#include "CubicMaskExtractor.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "Property.hpp"


namespace Processors {
namespace CubicMaskExtractor {

CubicMaskExtractor::CubicMaskExtractor(const std::string & name) :
		Base::Component(name),
		prop_XAxisMin_treshold("ransacFilter.XAxisMin_treshold", -0.5),
		prop_XAxisMax_treshold("ransacFilter.XAxisMax_treshold", 2.0),
		prop_YAxisMin_treshold("ransacFilter.YAxisMin_treshold", -2.0),
		prop_YAxisMax_treshold("ransacFilter.YAxisMax_treshold", 1.0),
		prop_ZAxisMin_treshold("ransacFilter.ZAxisMin_treshold", 0.5),
		prop_ZAxisMax_treshold("ransacFilter.ZAxisMax_treshold", 2.0)
{
		prop_XAxisMin_treshold.addConstraint("-2.0");
		prop_XAxisMin_treshold.addConstraint("2.0");
		prop_XAxisMax_treshold.addConstraint("-2.0");
		prop_XAxisMax_treshold.addConstraint("2.0");
		prop_YAxisMin_treshold.addConstraint("-2.0");
		prop_YAxisMin_treshold.addConstraint("2.0");
		prop_YAxisMax_treshold.addConstraint("-2.0");
		prop_YAxisMax_treshold.addConstraint("2.0");
		prop_ZAxisMin_treshold.addConstraint("-2.0");
		prop_ZAxisMin_treshold.addConstraint("2.0");
		prop_ZAxisMax_treshold.addConstraint("-2.0");
		prop_ZAxisMax_treshold.addConstraint("2.0");
		registerProperty(prop_XAxisMin_treshold);
		registerProperty(prop_XAxisMax_treshold);
		registerProperty(prop_YAxisMin_treshold);
		registerProperty(prop_YAxisMax_treshold);
		registerProperty(prop_ZAxisMin_treshold);
		registerProperty(prop_ZAxisMax_treshold);
} 

CubicMaskExtractor::~CubicMaskExtractor() {
  
}

void CubicMaskExtractor::prepareInterface() {
  
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_centerMassPoint", &in_centerMassPoint);
	registerStream("out_mask", &out_mask);
	registerStream("out_filtered_cloud", &out_filtered_cloud);
	
	// Register handlers
	h_filter.setup(boost::bind(&CubicMaskExtractor::filter, this));
	registerHandler("filter", &h_filter);
	addDependency("filter", &in_cloud_xyz);

}

bool CubicMaskExtractor::onInit() {

	return true;
}

bool CubicMaskExtractor::onFinish() {
	return true;
}

bool CubicMaskExtractor::onStop() {
	return true;
}

bool CubicMaskExtractor::onStart() {
	return true;
}

void CubicMaskExtractor::filter() {
  
  // create clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
  cv::Point3d centerMassPoint = in_centerMassPoint.read();
  
  double centerX = (double) centerMassPoint.x;
  double centerY = (double) centerMassPoint.y;
  double centerZ = (double) centerMassPoint.z;
  std::cout<<"centerX"<<centerX<<"centerY"<<centerY<<"centerZ"<<centerZ<<"\n";
  double xL= centerMassPoint.x-prop_XAxisMin_treshold;
  double xH= centerMassPoint.x+prop_XAxisMax_treshold;
  
  double yL= centerMassPoint.y-prop_YAxisMin_treshold;
  double yH= centerMassPoint.y+prop_YAxisMax_treshold;
  
  double zL= centerMassPoint.z-prop_ZAxisMin_treshold;
  double zH= centerMassPoint.z+prop_ZAxisMax_treshold;
  
  cv::Mat mask;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredX (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredY (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredZ (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  
  std::cout << "Limity\n"<<"xL "<< xL<<"xH "<< xH<< "yL " <<yL<<"yH "<< yH <<"zL "<< zL <<"zH" <<zH<<"\n";
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (xL, xH);
  pass.filter (*cloud_filteredX);
  
  pass.setInputCloud (cloud_filteredX);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (yL, yH);
  pass.filter (*cloud_filteredY);
  
  pass.setInputCloud (cloud_filteredY);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (zL, zH);
  pass.filter (*cloud_filteredZ);

   std::cout <<"Koniec fitrlowania!!!\n";
   std::cout <<"Pozostalo po fitrowaniu: " << cloud_filteredZ->points.size ()<<" punktow\n";

   // write to output
   out_filtered_cloud.write(cloud_filteredZ);
   
   // nie moze stworzyc Mata!!!
   // libCubicMaskExtractor.so: undefined symbol: _ZN2cv3Mat6createEiPKii
   
   int size1 = 1028;
   int size2 = 1024;
   tmp_img = cv::Mat(size1, size2, CV_8UC1);
   
   
   /*
    for (int i = 0; i < size1.height; i++)
        {
            const uchar* rgb_p1 = img1.ptr <uchar> (i);
            const uchar* rgb_p2 = img2.ptr <uchar> (i);
            uchar* tmp_p = tmp_img.ptr <uchar> (i);
            int j, k = 0;
	    int val = 0;
            for (j = 0; j < size1.width; j += 3)
            {

            	uchar r1 = rgb_p1[j];
                uchar g1 = rgb_p1[j + 1];
                uchar b1 = rgb_p1[j + 2];

		uchar r2 = rgb_p2[j];
                uchar g2 = rgb_p2[j + 1];
                uchar b2 = rgb_p2[j + 2];
                
		uchar sum1 = r1+g1+b1;
		uchar sum2 = r2+g2+b2;

		if(abs(sum1-sum2)>m_sum_diff || abs(r1-r2)>m_red_diff ||  abs(g1-g2)>m_green_diff || abs(b1-b2)>m_blue_diff)
		{
		  val = 255;
		}
		else
		{
		  val=0;
		}
		tmp_p[k] = val;
		k++;
            }
        }
        std::cout<<"Write\n";
        out_img.write(tmp_img);
   */
   
   /*
     for(int y = 0; y < xyz.rows; y++)
    {
        uchar* rgb_ptr = oLeftRectified.ptr<uchar>(y);
        for(int x = 0; x < xyz.cols; x++)
        {
            cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
            //cv::Vec3f rgbVal = oLeftRectified.at<cv::Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;

            //Get RGB info
            pb = rgb_ptr[3*x];
            pg = rgb_ptr[3*x+1];
            pr = rgb_ptr[3*x+2];

            //Insert info into point cloud structure
            pcl::PointXYZRGB point1;
            point1.x = point[0];
            point1.y = point[1];
            point1.z = -point[2];
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            //uint8_t r = 255, g = 0, b = 0; // Example: Red color
            //uint32_t rgb = ((uint32_t)pr << 16 | (uint32_t)pg << 8 | (uint32_t)pb);
            point1.rgb = *reinterpret_cast<float*>(&rgb);
            cloud->push_back(point1);
        }
    }
    */
    // dziala na wczytanej chmurze !!!
    // mozna zamienic na iterowanie po obrazie 2D i sprawdzanie czy taki punkt jest w obrazie 3D
    /*
    int val = 0;
    for(int y = 0; y < cloud->width; y++)
    {
	  uchar* tmp_p = tmp_img.ptr <uchar> (y);
	  int k = 0;
	  for(int x = 0; x <cloud->height; x++)
	  {
	    
	    // get point coordinates
	    cv::Vec3f point = cloud.at<cv::Vec3f>(y, x);
	    
	    pcl::PointXYZ point1;
            point1.x = point[0];
            point1.y = point[1];
	    
	    // jesli punkt nalezy do przedzialu to przypisz 255
	    if(point1.x>xL && point1.x<xH && point1.y>yL && point1.y<yH)
	    {
		val = 255;
	    }
	    else		// jesli nie, to przypisz 0
	    {
		val=0;
	    }
	    tmp_p[k] = val;
	    k++;	       // iteracja po obrazie o jeden piksel
	  }
    }
    
    */
    
    // zmienic iterowanie po obrazie maski, a nie po chmurze
    // dla kolumny dla wiersza dla odpowiedneigo piksela sprawdzic czy ten piksel sie miesci w przedziale
    /*
    int val = 0;
    // only one row
    uchar* tmp_p = tmp_img.ptr <uchar> (0);
    int k=0;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
	    // jesli punkt nalezy do przedzialu to przypisz 255
	    if(cloud->points[i].x>xL && cloud->points[i].x<xH && cloud->points[i].y>yL && cloud->points[i].y<yH)
	    {
		val = 255;
	    }
	    else		// jesli nie, to przypisz 0
	    {
		val=0;
	    }
	    tmp_p[k] = val;
	    k++;	       // iteracja po obrazie o jeden piksel 
    }*/
    
    int pointCloudNumber = 0;
    for (int i = 0; i < tmp_img.size().height; i++)
        {
            uchar* tmp_p = tmp_img.ptr <uchar> (i);
            int j, k = 0;
	    int val = 0;
            for (j = 0; j < tmp_img.size().width; j++)
            {
		  pointCloudNumber = tmp_img.size().width*i+k;
		  // jesli punkt nalezy do przedzialu to przypisz 255
		  if(cloud->points.size()<pointCloudNumber)
		  {
			if(cloud->points[pointCloudNumber].x>xL && cloud->points[pointCloudNumber].x<xH && cloud->points[pointCloudNumber].y>yL && cloud->points[pointCloudNumber].y<yH)
			{
	      	      val = 255;
			}
	      	  else		// jesli nie, to przypisz 0
			{
				val=0;
			}
			tmp_p[k] = val;
			k++;	       // iteracja po obrazie o jeden piksel 
		  }
		  else
		  {
			std::cout<<"W CHMURZE JEST ZA MALO PUNKTOW!!!\nPunkt nr: " << pointCloudNumber<<"\n";
		  }
	      }
        }
    
   
   // out_mask.write(tmp_img);			// write mask 
   
   /*double xCenter = 1.0; //centerMassPoint.at<double>(0,0);
   double yCenter = -0.5; //centerMassPoint.at<double>(1,0);
   double zCenter = 1.0; //centerMassPoint.at<double>(2,0);
   */
   //std::cout << xCenter <<"  " <<yCenter << " "  <<zCenter<<std::endl;

   
}



} //: namespace CubicMaskExtractor
} //: namespace Processors
