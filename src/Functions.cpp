#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "Functions.h"
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace std;

void
Print(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

    std::set<int> rgbVals;


    for (size_t i = 0; i < cloud->points.size (); ++i)
//        std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << cloud->points[i].rgb << std::endl;
        rgbVals.insert (cloud->points[i].rgb);

   std::cout<< rgbVals.size() << std::endl;
}

void
write(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::string filename){

    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cout << "Saved " << cloud->points.size () << " data points to "<< filename << "\n"<<std::endl;

}

float
RandomFloat(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}

PointXYZRGB
averagePoints(PointXYZRGB p1, PointXYZRGB p2){

    PointXYZRGB avg;

    avg.x = (p1.x + p2.x)/2;
    avg.y = (p1.y + p2.y)/2;
    avg.z = (p1.z + p2.z)/2;


    return avg;
}

//cout<<"  "<<euclideanDistance(Boundries[first]->points[i],Boundries[sec]->points[j])<<endl;
//if (euclideanDistance(Boundries[first]->points[i],Boundries[sec]->points[i]) < 0.1){

//    Boundries[first]->points[i] = averagePoints(Boundries[first]->points[i], Boundries[sec]->points[j]);

//    Boundries[sec]->points[i] = averagePoints(Boundries[first]->points[i], Boundries[sec]->points[j]);
//}

void
FixDuplicatePoints(const vector < PointCloud<PointXYZRGB>::Ptr> &Boundries){

    PointCloud<PointXYZRGB> all_corners;
    all_corners.points.resize( 4 * Boundries.size() );

    int count = 0;

    for(int first = 0; first < Boundries.size();++first){
        for(int i=0; i < 4;++i){
            all_corners.points[count].x = Boundries[first]->points[i].x;
            all_corners.points[count].y = Boundries[first]->points[i].y;
            all_corners.points[count].z = Boundries[first]->points[i].z;
            ++count;
        }

    }//outer
    PointCloud<PointXYZRGB>::Ptr all_corners_Ptr (new PointCloud<PointXYZRGB> (all_corners));


    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (all_corners_Ptr);

     std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.3);
    ec.setSearchMethod (tree);
    ec.setInputCloud (all_corners_Ptr);
    ec.extract (cluster_indices);

    cout<<"----------------->"<<cluster_indices.size()<<endl;

    for (int i = 0; i < cluster_indices.size(); ++i){

        Eigen::Vector4f centerV;
        compute3DCentroid(*all_corners_Ptr, cluster_indices[i],centerV);
        cout<<centerV[0]<<" "<<centerV[1]<<" "<<centerV[2]<<endl;
        PointXYZRGB center;
        center.x = centerV[0];
        center.y = centerV[1];
        center.z = centerV[2];




        for(int j = 0; j < all_corners_Ptr->points.size();j++){
            if (euclideanDistance(center, all_corners_Ptr->points[j]) < 0.3 ){
                cout<<"changed  "<<euclideanDistance(center, all_corners_Ptr->points[j])<<endl;
                all_corners_Ptr->points[j].x = center.x;
                all_corners_Ptr->points[j].y = center.y;
                all_corners_Ptr->points[j].z = center.z;
            }
        }
    }


    for(int j = 0; j < all_corners_Ptr->points.size();j++){

        }

}





void
CreateCornerFile(const vector < PointCloud<PointXYZRGB>::Ptr> &Boundries, string filename){
    FixDuplicatePoints(Boundries);

    ofstream myfile;
    myfile.open (filename + "-Corners.obj");
    int vertCount = 0;
    for(int j = 0; j < Boundries.size();++j){
        myfile << "o Object."<<to_string(j)<<"\n";
        for (int i = 0; i < Boundries[j]->points.size(); ++i){
                myfile<<"v "<<Boundries[j]->points[i].x<<"  "<<Boundries[j]->points[i].y<<"  "<<Boundries[j]->points[i].z<<"\n";
//                cout<<Boundries[j]->points[i].x<<"  "<<Boundries[j]->points[i].y<<"  "<<Boundries[j]->points[i].z<<endl;
            }
        myfile<<"l "<<vertCount + 1<<" "<< vertCount + 2<<" "<< vertCount + 4<<" "<< vertCount + 3<<"\n";
        myfile<<"\n";
        vertCount += 4;
    }

    myfile.close();
}
