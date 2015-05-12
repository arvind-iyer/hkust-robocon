#include "stdafx.h"

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
	#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>


boost::mutex mutex;

void keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* )
{
	if( event.getKeyCode() && event.keyDown() ){
		std::cout << "Key : " << event.getKeyCode() << std::endl;
	}
}

void mouse_callback( const pcl::visualization::MouseEvent& event, void* )
{
	if( event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton ){
		std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
	}
}

int _tmain( int argc, _TCHAR* argv[] )
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
	viewer->registerKeyboardCallback( &keyboard_callback, "keyboard" );
	viewer->registerMouseCallback( &mouse_callback, "mouse" );
	
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
	boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> function =
		[&cloud]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &ptr ){
		boost::mutex::scoped_lock lock( mutex );
		cloud = ptr;
	};

	pcl::Grabber* grabber = new pcl::Kinect2Grabber();

	boost::signals2::connection connection = grabber->registerCallback( function );

	grabber->start();

	while( !viewer->wasStopped() ){
		viewer->spinOnce();

		if( cloud ){
			if( !viewer->updatePointCloud( cloud, "cloud" ) ){
				viewer->addPointCloud( cloud, "cloud" );
				viewer->resetCameraViewpoint( "cloud" );
			}
		}

		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
	}

	grabber->stop();

	return 0;
}