#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <string>
#include <iostream>
#include <algorithm>
#include <tclap/CmdLine.h>
#include <stdio.h>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>



using namespace pcl;

#define win32_lean_and_mean             // exclude rarely-used stuff from windows headers


int GenerateHightMap(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudInput, int sizeX, int & sizeY, char modeColor, float percentile, float minColor, float maxColor, float** &map);

/// <summary>
///  Generats Heght Map float Grayscale image, with size X und Y 
/// </summary>
/// <param name="PointCloudInput"> <para>Points Area</para></param>
/// <param name="sizeX"> <para>X size of image</para></param>
/// <param name="sizeY"> <para>output parametr Y size of image</para></param>
/// <param name="modeColor"> <para>l- long hsv angle s - short hsv angle g - grauscale,z leave it as z</para></param>
/// <param name="modeValues"> <para>h- heigthest value on pixel, l lowest value in pixel, a average value on pixel, m not yet</para></param>
/// <param name="minColor"> <para>Color of lowest element, by hsv in grad will be normalized to [0,360]</para></param>
/// <param name="maxColor"> <para>Color of heigthest element, by hsv in grad will be normalized to [0,360]</para></param>
/// <param name="map"> <para>output parametr with Heigh map</para></param>
/// <returns> <para> Backgroung reference </para> </returns>
int GenerateHightMap(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudInput, int sizeX, int & sizeY, char modeColor, char modeValues, float minColor, float maxColor, float** &map);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GenerateHightMap(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudInput, int sizeX, int & sizeY, char modeColor, char modeValues, float minColor, float maxColor, float** &map){
	// minColor-100 means background!!!
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*PointCloudInput, min, max);
	double Xmin = min.x();
	double Ymin = min.y();
	double Zmin = min.z();
	double Xmax = max.x();
	double Ymax = max.y();
	double Zmax = max.z();

	int outSizeX = sizeX;
	double XproPixel = (Xmax - Xmin) / outSizeX;
	int outSizeY = (sizeX * (Ymax - Ymin) / (Xmax - Xmin)) + 1;
	sizeY = outSizeY;
	double YproPixel = (Ymax - Ymin) / outSizeY;
	std::vector<float>** img;
	float **outputimage;
	img = new std::vector<float>*[outSizeY];
	outputimage = new float*[outSizeY];
	for (int i = 0; i < outSizeY; i++){
		img[i] = new std::vector<float>[outSizeX];
		outputimage[i] = new float[outSizeX];
		for (int j = 0; j < outSizeX; j++){
			outputimage[i][j] = Zmin - 100;
		}
	}
	double dZ = Zmax - Zmin;
	for (int i = 0; i < PointCloudInput->size(); i++){
		int x = (PointCloudInput->points[i].x - Xmin) / (XproPixel);
		if (x == outSizeX)x = outSizeX - 1;

		int y = (PointCloudInput->points[i].y - Ymin) / (YproPixel);
		if (y == outSizeY)y = outSizeY - 1;
		float c = PointCloudInput->points[i].z;
		img[y][x].push_back(c);
	}

	for (int y = 0; y < outSizeY; y++){
		for (int x = 0; x < outSizeX; x++){
			if (img[y][x].size() == 0){
				continue;
			}

			if (modeValues == 'a'){
				float av=0;
				for (int i = 0; i < img[y][x].size(); i++){
					av += img[y][x][i];
				}
				outputimage[y][x]= av/img[y][x].size();
				continue;
			}
			if (modeValues == 'l'){
					
						float min = Zmax + 100;
						int minindex = 0;
						for (int j = 0; j < img[y][x].size(); j++){
							if (img[y][x][j] < min){
								min = img[y][x][j];
								minindex = j;
							}
						}
						
					
						outputimage[y][x] = img[y][x][minindex];
						continue;
			}
			if (modeValues == 'h'){
					float max = Zmin-100;
					int maxindex = 0;
					for (int j = 0; j < img[y][x].size(); j++){
						if (img[y][x][j] > max){
							max = img[y][x][j];
							maxindex = j;
						}
					}
					outputimage[y][x] = img[y][x][maxindex];
					continue;
			}
			if (modeValues == 'm'){
					for (int i = 0; i < img[y][x].size(); i++){
						float min = Zmin - 100;
						int minindex = i;
						for (int j = i; j <= img[y][x].size(); j++){
							if (img[y][x][j] < min){
								min = img[y][x][j];
								minindex = j;
							}
						}
						if (minindex != i){
							std::swap(img[y][x][i], img[y][x][minindex]);
						}
					}
					if (img[y][x].size() % 2 == 1){
						outputimage[y][x] = img[y][x][img[y][x].size()/2];
					}
					else{
						outputimage[y][x] = (img[y][x][img[y][x].size() / 2] + img[y][x][img[y][x].size() / 2+1])/2;
					}
			}
			

		}
	}

	if (modeColor == 'z'){
		map = outputimage;
		return Zmin-100;
	}

	
	
	if (modeColor == 's' || modeColor == 'l'){
		while (minColor < 0)minColor += 360;
		while (maxColor < 0)maxColor += 360;
		while (minColor >= 360)minColor -= 360;
		while (maxColor >= 360)maxColor -= 360;
		if (maxColor == minColor){
		
			maxColor += 360;
		}
		else if (maxColor > minColor){
			if (maxColor - minColor <= 180){
				if (modeColor == 's'){
					
				}
				else {
					
					minColor += 360;
				}

			}
			else{
				if (modeColor == 's'){
					
					minColor += 360;
				}
				else {
					
				}
			}
		}
		else{
			if (minColor - maxColor <= 180){
				if (modeColor == 's'){
					
				}
				else {
					
					maxColor += 360;
				}
			}
			else{
				if (modeColor == 's'){
					
					maxColor += 360;
				}
				else {
					
				}
			}
		}
	}

	float dColor = maxColor - minColor;
	for (int y = 0; y < outSizeY; y++){
		for (int x = 0; x < outSizeX; x++){
			if (outputimage[y][x] <= Zmin - 50){
				outputimage[y][x] = (minColor<maxColor ? minColor : maxColor) - 100;
				continue;
			}
			outputimage[y][x] = dColor*(outputimage[y][x] - Zmin) / dZ + minColor;
		}
	}
	map = outputimage;

	return (minColor<maxColor?minColor:maxColor) - 100;
}


