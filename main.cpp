#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/real_sense_2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, pcl::PointXYZRGBA
typedef pcl::PointXYZRGB PointType;

int main( int argc, char* argv[] )
{
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->addCoordinateSystem( 0.1 );
    viewer->setCameraPosition( 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0 );

    // Point Cloud Color Hndler
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ){
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGB ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBAField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBAField<PointType>() );
        handler = color_handler;
    }
    else{
        throw std::runtime_error( "This PointType is unsupported." );
    }

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */

            cloud = ptr->makeShared();
        };

    // RealSense2Grabber
    boost::shared_ptr<pcl::RealSense2Grabber> grabber = boost::make_shared<pcl::RealSense2Grabber>();
    //boost::shared_ptr<pcl::RealSense2Grabber> grabber = boost::make_shared<pcl::RealSense2Grabber>( "746112061315" ); // specific device (serial number)
    //boost::shared_ptr<pcl::RealSense2Grabber> grabber = boost::make_shared<pcl::RealSense2Grabber>( "../file.bag" ); // bag file
    
    // Set Frame Size and FPS
    //grabber->setDeviceOptions( 640, 480, 30 );

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            handler->setInputCloud( cloud );
            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
                viewer->addPointCloud( cloud, *handler, "cloud" );
            }
        }
    }

    // Stop Grabber
    grabber->stop();
    
    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}
