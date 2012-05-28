#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <octree_search.h>

using boost::asio::ip::tcp;

const int max_length = 64;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

void session(socket_ptr sock)
{
  try
  {
    for (;;)
    {
		char data[max_length];

		boost::system::error_code error;
      
		size_t length = sock->read_some(boost::asio::buffer(data), error);

		if (error == boost::asio::error::eof)
			break; // Connection closed cleanly by peer.
		else if (error)
			throw boost::system::system_error(error); // Some other error.

		std::ostringstream ss;
		ss.write(data, length);
		std::string queryPickRay = ss.str();
		
		std::vector<std::string> vecParam;
		boost::split(vecParam, queryPickRay, boost::is_any_of(","), boost::token_compress_on);

		/*
		0: cloudID,
		1,2,3: rayOriginX, rayOriginY, rayOriginZ
		4,5,6: rayDirectionX, rayDirectionY, rayDirectionZ
		*/
		Eigen::Vector3f origin = Eigen::Vector3f ( ::atof(vecParam[1].c_str()),
												   ::atof(vecParam[2].c_str()),
												   ::atof(vecParam[3].c_str()) );
		Eigen::Vector3f dir = Eigen::Vector3f( ::atof(vecParam[4].c_str()),
												 ::atof(vecParam[5].c_str()),
												 ::atof(vecParam[6].c_str()) );

		Eigen::Vector3f pickPoint = FindPickPoint(origin, dir);

		std::stringstream ssTmp;
		ssTmp << boost::lexical_cast<std::string>( pickPoint[0] ) << "," <<
				 boost::lexical_cast<std::string>( pickPoint[1] ) << "," <<
				 boost::lexical_cast<std::string>( pickPoint[2] );
		std::string s = ssTmp.str();
		
		boost::asio::write (*sock, boost::asio::buffer(s) );
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception in thread: " << e.what() << "\n";
  }
}

void server(boost::asio::io_service& io_service, short port)
{
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  {
    socket_ptr sock(new tcp::socket(io_service));
    a.accept(*sock);
    boost::thread t(boost::bind(session, sock));
  }
}

int main(int argc, char* argv[])
{
  try
  {
    //if (argc != 2)
    //{
    //  std::cerr << "Usage: blocking_tcp_echo_server <port>\n";
    //  return 1;
    //}

	extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	extern pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
	
	pcl::io::loadPCDFile("c:\\test.pcd", *cloud);
	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();

    boost::asio::io_service io_service;

	//server(io_service, std::atoi(argv[1]));
	server( io_service, 9000 );
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}