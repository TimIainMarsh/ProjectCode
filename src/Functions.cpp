#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "Functions.h"

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
    std::cout << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;

}

float
RandomFloat(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}


void
CreateCornerFile(const vector < PointCloud<PointXYZRGB>::Ptr> &Boundries, string filename){
    ofstream myfile;


    myfile.open (filename + "-Corners.obj");
    int vertCount = 0;
    for(int j = 0; j < Boundries.size();++j){
        myfile << "o Object."<<to_string(j)<<"\n";
        for (int i = 0; i < Boundries[j]->points.size(); ++i){
                myfile<<"v "<<Boundries[j]->points[i].x<<"  "<<Boundries[j]->points[i].y<<"  "<<Boundries[j]->points[i].z<<"\n";
//                cout<<Boundries[j]->points[i].x<<"  "<<Boundries[j]->points[i].y<<"  "<<Boundries[j]->points[i].z<<endl;
            }
        myfile<<"f "<<vertCount + 1<<" "<< vertCount + 3<<" "<< vertCount + 4<<" "<< vertCount + 2<<"\n";
        myfile<<"\n";
        vertCount += 4;
    }

    myfile.close();
}
