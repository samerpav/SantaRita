#include <boost/config/warning_disable.hpp>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include <iostream>
//#include <vector>
//#include <ctime>
#include <conio.h>
//#include <iterator>
//#include <functional>
//#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <fstream>

// global variables!!!
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
float resolution = 0.01f;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

void myviewer(pcl::PointCloud<pcl::PointXYZ>::Ptr c, std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel) {
   
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (c, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();	

	//viewer->addLine(pcl::PointXYZ(0, 0.1, 0), pcl::PointXYZ(0, 0.1, 10), "line1", 0);	

	for (size_t i=0; i < voxel.size() ; ++i)
		viewer->addSphere ( pcl::PointXYZ(voxel[i].x, voxel[i].y, voxel[i].z) , 0.1, "sphere" + boost::lexical_cast<std::string>(i));

	while (!viewer->wasStopped ())    {
		//viewer->spin();
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void ConvertPtsToPcd(std::string filename)
{
	//std::ifstream fs("C:\\clouds\\indoor-5m.pts", std::ios::in );
	//std::ifstream fs("c:\\400k.pts", std::ios::in );
	//std::ifstream fs("c:\\tree-63k-z.pts", std::ios::in );
	//std::ifstream fs("c:\\test.pts", std::ios::in );
	//if (!fs.is_open () || fs.fail ()) {
	//	std::cout << "failed to open pts file!" << std::endl;
	//	return (-1);
	//}
	//std::string contents((std::istreambuf_iterator<char>(fs)), 
	//std::istreambuf_iterator<char>());
	//std::cout << "read OK" << std::endl;

	//std::vector<std::string> vPoints;
	//std::cout << "split started" << std::endl;
	//boost::split(vPoints, contents, boost::is_any_of("\n"), boost::token_compress_on);
	//std::cout << "splite finished" << std::endl;

	//std::vector<std::string> p;
	//for (size_t i=0; i < vPoints.size(); ++i) {
	//	std::string line = vPoints.at(i);
	//	
	//	//boost::trim (line);
	//	boost::split (p, line, boost::is_any_of (std::string ( "\t\r ")), boost::token_compress_on);

	//	if (p.size() < 3)
	//		continue;
	//	
	//	cloud->push_back (pcl::PointXYZ ( atof(p.at(0).c_str()) , atof(p.at(1).c_str()), atof(p.at(2).c_str()) )); 

	//	if (i % 1000 == 0) 
	//		std::cout << i << std::endl;
	//}
	//cloud->is_dense = true;
	//fs.close();
	//std::cout << "points added" << std::endl;
	//pcl::io::savePCDFileBinary("c:\\test400k.pcd", *cloud);
	//return 0;
}

Eigen::Vector3f FindPickPoint(Eigen::Vector3f v_origin, Eigen::Vector3f v_dir)
{
	srand ((unsigned int) time (NULL));

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile("c:\\test.pcd", *cloud);
	//float resolution = 0.1f;
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	//octree.setInputCloud (cloud);
	//octree.addPointsFromInputCloud ();

	//Eigen::Vector3f v_origin = Eigen::Vector3f( 0.849, -2.379, 0.445);
	//Eigen::Vector3f v_dir = Eigen::Vector3f( 0.019, 0.569, -0.822 );

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> intersectedVoxels;
	std::cout << "# of Voxel found = " << octree.getIntersectedVoxelCenters(v_origin, v_dir, intersectedVoxels) << std::endl;

	float dMin = 0.002;
	Eigen::Vector3f bestPoint(Eigen::Vector3f::Zero());

	for (size_t i=0; i < intersectedVoxels.size(); ++i) {
		
		// print voxel center coordinate
		//std::cout << mypoints[i].x << ' ' << mypoints[i].y << ' ' << mypoints[i].z << std::endl;		
		
		std::vector<int> pointIdxVec;

		if (octree.voxelSearch (intersectedVoxels[i], pointIdxVec)) {
			std::cout << "       # of pts in voxel = " << pointIdxVec.size() << std::endl;
			
			for (size_t i = 0; i < pointIdxVec.size (); ++i) {

				double d = pcl::sqrPointToLineDistance(
													   Eigen::Vector4f(cloud->points[pointIdxVec[i]].x,
																	   cloud->points[pointIdxVec[i]].y,
																	   cloud->points[pointIdxVec[i]].z,
																	   0),
													   Eigen::Vector4f(v_origin[0], v_origin[1], v_origin[2], 0),
													   Eigen::Vector4f(v_dir[0], v_dir[1], v_dir[2],  0)
													  );

				std::cout << "    " << cloud->points[pointIdxVec[i]].x << " " 
									<< cloud->points[pointIdxVec[i]].y << " " 
									<< cloud->points[pointIdxVec[i]].z << " "
									<< "dist = " << d << std::endl;

				if (d < dMin) {
					dMin = d;
					bestPoint = Eigen::Vector3f (cloud->points[pointIdxVec[i]].x,
												cloud->points[pointIdxVec[i]].y,
												cloud->points[pointIdxVec[i]].z);

				}
				//cloudPick->push_back( pcl::PointXYZ(cloud->points[pointIdxVec[i]].x, cloud->points[pointIdxVec[i]].y, cloud->points[pointIdxVec[i]].z) );
				//std::cout << "    " << cloud->points[pointIdxVec[i]].x << " " << cloud->points[pointIdxVec[i]].y << " " << cloud->points[pointIdxVec[i]].z << std::endl;
			} // for
		} // if
	}

	return bestPoint;

	//myviewer (cloud, voxelPoints);
	//_getch();
}
