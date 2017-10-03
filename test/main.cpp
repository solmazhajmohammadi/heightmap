#include <iostream>
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h"
#include <fstream>
#include <string>
#include <math.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <tclap/CmdLine.h>


using namespace pcl;
using namespace std;


int main (int argc, char** argv)
{

try{
	TCLAP::CmdLine cmd("Transforming the PLY to Gantry coordinate System", ' ', "0.1");
	TCLAP::ValueArg<std::string> InputFile("i", "input", "plyfile path", true, "", "string");
	TCLAP::ValueArg<std::string> OutputFile("o", "output", "output", true, "", "string");
	TCLAP::ValueArg<float> X("x", "X", "X offset", false, 0.0, "float");
	TCLAP::ValueArg<float> Y("y", "Y", "Y offset", false, 0.0, "float");
	TCLAP::ValueArg<float> Z("z", "Z", "Z offset", false, 0.0 , "float");


	cmd.add(InputFile);
	cmd.add(OutputFile);
	cmd.add(X);
	cmd.add(Y);
	cmd.add(Z);
	cmd.parse(argc, argv);



	// Fill in the cloud data
	PointCloud<PointXYZRGB>::Ptr input_cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud (new PointCloud<PointXYZRGB>);
	pcl::PLYReader reader;
	reader.read (InputFile.getValue(), *input_cloud);


	//apply the transformation

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a translation 
	transform_1 (0,3) = X.getValue();
	transform_1 (1,3) = Y.getValue();
	transform_1 (2,3) = Z.getValue();

	pcl::transformPointCloud (*input_cloud, *transformed_cloud, transform_1);


	PointCloud<pcl::PointXYZRGB> output_cloud = *transformed_cloud;	
	pcl::io::savePLYFileBinary (OutputFile.getValue(), output_cloud );

	return (0);

	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;

	}
}
