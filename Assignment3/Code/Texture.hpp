//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
      /*  if (u < 0) {
            u = 0.01f;
        }
        if (v < 0) {
            v = 0.01f;
        }
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);*/
        return getColorBilinear(u,v);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        if (u < 0) {
            u = 0.01f;
        }
        if (v < 0) {
            v = 0.01f;
        }
     
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto uMin = std::floor(u_img);
        auto uMax = std::min((int)std::ceil(u_img),width) ;

        auto vMin = std::floor(v_img);
        auto vMax = std::min((int)std::ceil(v_img), height);

        auto color1 = image_data.at<cv::Vec3b>(vMin, uMin);
        auto color2 = image_data.at<cv::Vec3b>(vMax, uMin);

		auto color3 = image_data.at<cv::Vec3b>(vMin, uMax);
		auto color4 = image_data.at<cv::Vec3b>(vMax, uMax);

		float detlaU = (u_img - uMin) / (uMax - uMin);
		auto uDetlaColor1 = (color3 - color1) * detlaU + color1;
		auto uDetlaColor2 = (color4 - color2) * detlaU + color2;

		float detlaV = (v_img - vMin) / (vMax - vMin);
		auto returnColor = (uDetlaColor2 - uDetlaColor1) * detlaV + uDetlaColor1;
		return Eigen::Vector3f(returnColor[0], returnColor[1], returnColor[2]);
	}
};
#endif //RASTERIZER_TEXTURE_H
