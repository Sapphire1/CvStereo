/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "DepthRainbow.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthRainbow {

DepthRainbow::DepthRainbow(const std::string & name) :
        Base::Component(name)
{
}

DepthRainbow::~DepthRainbow() {
}

void DepthRainbow::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_depth_mono", &in_depth_xyz);
    registerStream("out_depth_rainbow", &out_depth_rainbow);

	// Register handlers
    h_ConvertMonoToRainbow.setup(boost::bind(&DepthRainbow::convertMonoToRainbow, this));
    registerHandler("convertMonoToRainbow", &h_ConvertMonoToRainbow);
    addDependency("convertMonoToRainbow", &in_depth_xyz);
}

bool DepthRainbow::onInit() {
	return true;
}

bool DepthRainbow::onFinish() {
	return true;
}

bool DepthRainbow::onStop() {
	return true;
}

bool DepthRainbow::onStart() {
	return true;
}

typedef struct{uchar r; uchar g; uchar b;} color;

void DepthRainbow::convertMonoToRainbow() {
    cv::Mat data(in_depth_xyz.read());
    cv::Mat depth;
    cv::Mat out;
    float max_val = 0;
    float min_val = 0;
    float delta;
    {
        cv::Vec3f point = data.at<cv::Vec3f>(0, 0);
        max_val = point[2];
        min_val = point[2];
    }
    for (int y = 0; y < data.rows; y++) {
        for (int x = 0; x < data.cols; x++) {
            cv::Vec3f point = data.at<cv::Vec3f>(y, x);
            if (max_val < point[2] && point[2] != 10000) max_val = point[2];
            if (min_val > point[2]) min_val = point[2];
        }
    }
    LOG(LINFO) << "Min value = " << min_val;
    LOG(LINFO) << "Max value = " << max_val;

    delta = (max_val - min_val) / 255;
    LOG(LINFO) << "Converting mono to rainbow";
    try {
        depth.create(data.size(), CV_8U);
        for (int y = 0; y < out.rows; y++) {
            for (int x = 0; x < out.cols; x++) {
                cv::Vec3f point = data.at<cv::Vec3f>(0, 0);
                float floatDepth = point[2];
                unsigned short curDepth = (floatDepth - min_val) / delta;
                depth.at<color>(y, x) = point[2];
            }
        }
        out.create(data.size(), CV_8UC3);
        for (int y = 0; y < out.rows; y++) {
            for (int x = 0; x < out.cols; x++) {
                color col;
                col.r = col.g = col.b = 0;
                cv::Vec3f point = data.at<cv::Vec3f>(0, 0);
                float floatDepth = point[2];
                unsigned short curDepth = (floatDepth - min_val) / delta;
                int lb = curDepth & 0xff;
                switch (curDepth>>8) {
                case 0:
                    col.b = 255;
                    col.g = 255-lb;
                    col.r = 255-lb;
                    break;
                case 1:
                    col.b = 255;
                    col.g = lb;
                    col.r = 0;
                    break;
                case 2:
                    col.b = 255-lb;
                    col.g = 255;
                    col.r = 0;
                    break;
                case 3:
                    col.b = 0;
                    col.g = 255;
                    col.r = lb;
                    break;
                case 4:
                    col.b = 0;
                    col.g = 255-lb;
                    col.r = 255;
                    break;
                case 5:
                    col.b = 0;
                    col.g = 0;
                    col.r = 255-lb;
                    break;
                default:
                    col.r = col.g = col.b = 0;
                    break;
                }
                if (point[2] == 10000) col.r = col.g = col.b = 0;
                out.at<color>(y, x) = col;
            }
        }
        out_depth_rainbow.write(out);
	} catch (...)
	{
		LOG(LERROR) << "Error occured in processing input";
	}
}

} //: namespace DepthRainbow
} //: namespace Processors
