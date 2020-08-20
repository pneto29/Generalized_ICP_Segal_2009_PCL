#include "validationlib.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


int main (int argc, char** argv)
{
    if (argc < 3) {
        printf("Missing parameters\n");
        printf("Usage: gicp <target> <source>\n");

        return -1;
    }

    clock_t tempo;
    tempo = clock();
    
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);
    PointCloud::Ptr cloud_icp(new PointCloud);
    PointCloud::Ptr cloud_regist(new PointCloud);

    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_out) == -1) {
        PCL_ERROR ("Couldn't read file model \n");
        return (-1);
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_in) == -1) {
        PCL_ERROR ("Couldn't read file shape \n");
        return (-1);
    }
    

          
           pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
           approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
           approximate_voxel_filter.setInputCloud (cloud_in);
           approximate_voxel_filter.filter (*cloud_in);

           pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter2;
           approximate_voxel_filter2.setLeafSize (0.2, 0.2, 0.2);
           approximate_voxel_filter2.setInputCloud (cloud_out);
           approximate_voxel_filter2.filter (*cloud_out);

    int iteration = atoi(argv[3]);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setMaximumIterations(iteration);
    icp.align(*cloud_icp);

    Eigen::Matrix4f rotation_matrix = icp.getFinalTransformation();
    double rms = computeCloudRMSE(cloud_in, cloud_out, std::numeric_limits<double>::max());
    double elem1 = rotation_matrix(0, 0);
    double elem2 = rotation_matrix(1, 1);
    double elem3 = rotation_matrix(2, 2);
    double angle123 = (elem1 + elem2 + elem3 - 1) / 2.0;
    double rot_angle = (acos(angle123) * 180.0) / PI;
    
    std::cout << rotation_matrix << std::endl;
    std::cout << "RMSE: " << rms << std::endl;
  //  std::cout << "ANGL: " << rot_angle << std::endl;
    std::cout << "TIME: " << (clock() - tempo) / (double)CLOCKS_PER_SEC << std::endl;
    
 //   return 0;

    //---------------------------
    //Visualization
    //---------------------------

    int sizePoints = 2;


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("REGISTRO"));

    // Adicionando ambas as nuvens de pontos
    viewer1->setBackgroundColor(255,255,255);

    viewer1->addPointCloud(cloud_out, "cloud_out");
    viewer1->addPointCloud(cloud_in, "cloud_in");

    //Configurando cloud_out
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_in");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 255, 0,"cloud_out");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("REGISTRO"));

    // Adicionando ambas as nuvens de pontos
    viewer2->setBackgroundColor(255,255,255);

    viewer2->addPointCloud(cloud_out, "cloud_out");
    viewer2->addPointCloud(cloud_icp, "cloud_regist");

    //Configurando cloud_out
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_regist");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_regist");
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 255, 0,"cloud_out");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!viewer2->wasStopped())
    {
        viewer2->spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    system("pause");

}

