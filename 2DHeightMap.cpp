// GenerateHightMap.cpp : Defines the entry point for the console application.

#include "PCLDLL.h"

int main(int argc, char** argv){
	
	try{
		TCLAP::CmdLine cmd("show point cloud\nShowCloud as a part of PCLTools. Powered by LemnaTec", ' ', "0.1");

		TCLAP::ValueArg<std::string> InputFile("i", "input", "file ply with point cloud", true, "", "string");
		TCLAP::ValueArg<std::string> InputFile2("j", "input2", "file ply with point cloud", false, "", "string");
		TCLAP::ValueArg<std::string> OutputFile("o", "output", "file to create", true, "", "string");
		TCLAP::ValueArg<int> ResX("x", "Xresolution", "x resolution of image, y will be count auto", false, 2000, "int");
		TCLAP::ValueArg<char> ColorMode("c", "colorMode", "l long way for HSV color space\ns short way for HSV color space \ng grauscale\nz determinate color as z axis", false, 'l', "char");
		TCLAP::ValueArg<char> ValueMod("v", "valueMode", "l lowest element on the pixel\nh hightest element on pixel \na averege of elements\n m medians", false, 'h', "char");
		TCLAP::ValueArg<float> BackG("b", "background", "background color grayscale 0 to 1", false, 0, "float");
		TCLAP::ValueArg<float> min("", "min", "min color \n for HSV it is in Grad for H\n ", false, 0, "float");
		TCLAP::ValueArg<float> max("", "max", "max color \n for HSV it is in Grad for H\n ", false, 270, "float");
		TCLAP::ValueArg<char> ImageType("t", "type", "b for bmp\nt for tiff", false, 'b', "char");


		cmd.add(InputFile);
		cmd.add(InputFile2);
		cmd.add(OutputFile);
		cmd.add(ResX);
		cmd.add(BackG);
		cmd.add(ColorMode);
		cmd.add(ValueMod);
		cmd.add(min);
		cmd.add(max);
		cmd.add(ImageType);
		cmd.parse(argc, argv);




			pcl::PointCloud<pcl::PointXYZI>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloudin2(new pcl::PointCloud<pcl::PointXYZI>);
			if (pcl::io::loadPLYFile<pcl::PointXYZI>(InputFile.getValue(), *cloudin) == -1) // load the file
			{
				std::cout << "Couldn't read file" << std::endl;
				return -1;
			}



			if (pcl::io::loadPLYFile<pcl::PointXYZI>(InputFile2.getValue(), *cloudin2) == -1) // load the file
			{

			}else{
				*cloudin += *cloudin2;
			}



			int x = ResX.getValue();
			int y;
			float** map;
			float back=GenerateHightMap(cloudin, x, y, ColorMode.getValue(), ValueMod.getValue(), min.getValue(), max.getValue(), map);
			float backbe = BackG.getValue();
			if (ImageType.getValue() == 'b' && ColorMode.getValue()!= 'z'){
				cv::Mat mask(y, x, CV_8UC1);
				for (int i = 0; i < y; i++){
					for (int j = 0; j < x; j++){
						float a = map[i][j];
						if (a <= back + 50){
							mask.at<unsigned char>(i, j) = 0;
						}
						else{
							mask.at<unsigned char>(i, j) = 255;
						}

					}
				}
				cv::imwrite(OutputFile.getValue() + "_mask.bmp", mask);
				if (ColorMode.getValue() == 'g'){
					cv::Mat image(y, x, CV_8UC1);
					for (int i = 0; i < y; i++){
						for (int j = 0; j < x; j++){
							if (map[i][j] <= back + 50){
								image.data[i*x + j] = backbe;
							}
							else{
								image.data[i*x + j] = char( map[i][j]*255);
							}
						}
					}
					cv::imwrite(OutputFile.getValue() + ".bmp", image);
				}
				else{
					cv::Mat image(y, x, CV_8UC3);
					int vi = 0;
					for (int i = 0; i < y; i++)
					{
						for (int j = 0; j < x; j++)
						{

							float H = map[i][j];
							if (H < back + 50)
							{

								image.data[vi++] =(unsigned char) (backbe *255);
								image.data[vi++] =(unsigned char) (backbe * 255);
								image.data[vi++] =(unsigned char) (backbe * 255);
								continue;
							}
							while (H > 360) H -= 360;
							while (H < 0) H += 360;

							int hi = (int)H / 60;
							float V = 1;
							float S = 1;
							float f = (H / 60 - hi);
							float p = V*(1 - S);
							float q = V*(1 - S*f);
							float t = V*(1 - S*(1 - f));
							if (hi == 0 || hi == 6)
							{
								float r = (V * 255);
								float g = (t * 255);
								float b = (p * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
							else if (hi == 1)
							{
								float r = (q * 255);
								float g = (V * 255);
								float b = (p * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
							else if (hi == 2)
							{
								float r = (p * 255);
								float g = (V * 255);
								float b = (t * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
							else if (hi == 3)
							{
								float r = (p * 255);
								float g = (q * 255);
								float b = (V * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
							else if (hi == 4)
							{
								float r = (t * 255);
								float g = (p * 255);
								float b = (V * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
							else if (hi == 5)
							{
								float r = (V * 255);
								float g = (p * 255);
								float b = (q * 255);
								image.data[vi++] =(unsigned char) r;
								image.data[vi++] =(unsigned char) g;
								image.data[vi++] =(unsigned char) b;
							}
						}
					}
					cv::imwrite(OutputFile.getValue() + ".bmp", image);
				}

			}
			/*else{
				fstream f;
				f.open(OutputFile.getValue() + ".tiff", ios::out | ios::binary);

				
				f.write((const char *)&x, sizeof(int));
				f.write((const char *)&y, sizeof(int));
				f.write((const char *)&back, sizeof(float));
				f.write((const char *)&backbe, sizeof(float));
				char mode;
				if (ColorMode.getValue() == 'g'){
					mode = 'g';
				}
				else if (ColorMode.getValue() == 'z'){
					mode = 'z';
				}
				else{
					mode = 'c';
				}

				cv::Mat mask(y, x, CV_8UC1);
				f.write(&mode, sizeof(unsigned char));
				for (int i = 0; i < y; i++){
					for (int j = 0; j < x; j++){
						f.write((const char *)&map[i][j], sizeof(float));
						float a = map[i][j];
						if (a <= back + 50){
							mask.at<unsigned char>(i, j) = 0;
						}
						else{
							mask.at<unsigned char>(i, j) = 255;
						}

					}
				}

				f.close();
				if (ColorMode.getValue() != 'z'){
					cv::imwrite(OutputFile.getValue() + "_mask.bmp", mask);
					// for release path mast be changed

					std::string command = "C:\\src\\sandbox\\KirillKozlov\\PCLTools\\TiffFromBinFile\\bin\\Debug\\TiffFromBinFile.exe " + OutputFile.getValue();
					system(command.c_str());
				}
			}*/
	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;

	}




}


