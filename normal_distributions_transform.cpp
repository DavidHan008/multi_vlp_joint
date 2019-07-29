#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ> cloud_1;
    pcl::PointCloud<pcl::PointXYZ> cloud_2;
    pcl::PointCloud<pcl::PointXYZ> cloud_3;
    pcl::PointCloud<pcl::PointXYZ> cloud_result;
    pcl::PointCloud<pcl::PointXYZ> cloud_1_trans;
    pcl::PointCloud<pcl::PointXYZ> cloud_2_trans;
    pcl::PointCloud<pcl::PointXYZ> cloud_3_trans;

    pcl::console::TicToc tt;
    std::cerr<<"Reader...\n",tt.tic();

    pcl::PCDReader reader1;
    reader1.read("center.pcd",cloud_1);
    pcl::PCDReader reader2;
    reader2.read("left.pcd",cloud_2);
    pcl::PCDReader reader3;
    reader3.read("right.pcd",cloud_3);

    std::cerr<<"Done  "<<tt.toc()<<"  ms\n"<<std::endl;
    /* Reminder: how transformation matrices work :

          |-------> This column is the translation
   | 1 0 0 x |  \
   | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
   | 0 0 1 z |  /
   | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

   METHOD #1: Using a Matrix4f
   This is the "manual" method, perfect to understand but error prone !
   */
    //处理第一个点云数据
    //定义变换矩阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    transform_1 (0,3)=-0.04760347490526639;//x
    transform_1 (1,3)=3.933374978330232;//y
    transform_1 (2,3)=-0.1733476578374331;//z
    //输入的四元数 转化成旋转矩阵
    tf::Quaternion quaternion_1(-0.00557460918759057,0.003013038448977041,-0.009800698769672609,0.9999318934984206);//x,y,z,w
    tf::Matrix3x3 Matrix_1;
    Matrix_1.setRotation(quaternion_1);
    tf::Vector3 v1_1,v1_2,v1_3;
    v1_1=Matrix_1[0];
    v1_2=Matrix_1[1];
    v1_3=Matrix_1[2];
    transform_1 (0,0)=v1_1[0];
    transform_1 (0,1)=v1_1[1];
    transform_1 (0,2)=v1_1[2];
    transform_1 (1,0)=v1_2[0];
    transform_1 (1,1)=v1_2[1];
    transform_1 (1,2)=v1_2[2];
    transform_1 (2,0)=v1_3[0];
    transform_1 (2,1)=v1_3[1];
    transform_1 (2,2)=v1_3[2];
    //输入的cloud_1，经过transform_1变换之后，输出cloud_1_trans，平移之后的点云
    pcl::transformPointCloud (cloud_1, cloud_1_trans, transform_1);

    //处理第二个点云数据
    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
    transform_2 (0,3)=-0.9114749222525654;//x
    transform_2 (1,3)=-0.2044065801246115;//y
    transform_2 (2,3)=0.285;//z
    tf::Quaternion quaternion_2(0.004911912854527661,0.01235642422757478,0.742849173394299,0.6693267494130384);//x,y,z,w
    tf::Matrix3x3 Matrix_2;
    Matrix_2.setRotation(quaternion_2);
    tf::Vector3 v2_1,v2_2,v2_3;
    v2_1=Matrix_2[0];
    v2_2=Matrix_2[1];
    v2_3=Matrix_2[2];
    transform_2 (0,0)=v2_1[0];
    transform_2 (0,1)=v2_1[1];
    transform_2 (0,2)=v2_1[2];
    transform_2 (1,0)=v2_2[0];
    transform_2 (1,1)=v2_2[1];
    transform_2 (1,2)=v2_2[2];
    transform_2 (2,0)=v2_3[0];
    transform_2 (2,1)=v2_3[1];
    transform_2 (2,2)=v2_3[2];
    pcl::transformPointCloud (cloud_2, cloud_2_trans, transform_2);

    //处理第三个点云数据
    Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
    transform_3 (0,3)=0.8877439387257221;//x
    transform_3 (1,3)=-0.2386284956028188;//y
    transform_3 (2,3)=0.19;//z
    tf::Quaternion quaternion_3(-0.002047146471783097,0.008139579096743448,-0.7311369261493741,0.682179119926941);//x,y,z,w
    tf::Matrix3x3 Matrix_3;
    Matrix_3.setRotation(quaternion_3);
    tf::Vector3 v3_1,v3_2,v3_3;
    v3_1=Matrix_3[0];
    v3_2=Matrix_3[1];
    v3_3=Matrix_3[2];
    transform_3 (0,0)=v3_1[0];
    transform_3 (0,1)=v3_1[1];
    transform_3 (0,2)=v3_1[2];
    transform_3 (1,0)=v3_2[0];
    transform_3 (1,1)=v3_2[1];
    transform_3 (1,2)=v3_2[2];
    transform_3 (2,0)=v3_3[0];
    transform_3 (2,1)=v3_3[1];
    transform_3 (2,2)=v3_3[2];
    pcl::transformPointCloud (cloud_3, cloud_3_trans, transform_3);

    //三个点云数据叠加
    cloud_result=cloud_1_trans+cloud_2_trans;
    cloud_result=cloud_result+cloud_3_trans;
    //cloud_3=cloud_1_trans+cloud_1;
    std::cerr<<"The point cloud_1 has:  "<<cloud_1.points.size()<<"  points data."<<std::endl;
    std::cerr<<"The point cloud_2 has:  "<<cloud_2.points.size()<<"  points data."<<std::endl;
    std::cerr<<"The point cloud_3 has:  "<<cloud_3.points.size()<<"  points data."<<std::endl;
    std::cerr<<"The point cloud_result has:  "<<cloud_result.points.size()<<"  points data."<<std::endl;

    pcl::PCDWriter writer;
    writer.write("3vlp_result.pcd",cloud_result);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader_3;
    reader_3.read("3vlp_result.pcd",*cloud_4);

    std::cerr<<"SorFilter...\n",tt.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_4);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloud_filtered);
    std::cerr<<"Done  "<<tt.toc()<<" ms.\n"<<std::endl;

    std::cerr<<"visualization...\n",tt.tic();
    pcl::visualization::CloudViewer viewer1("3D Viewer1");
    viewer1.showCloud(cloud_filtered);
    while(!viewer1.wasStopped())
    {}
    std::cerr<<"Done  "<<tt.toc()<<"  ms.\n"<<std::endl;

    return 0;

}