#include "MatrixReaderWriter.h"
#include "PlaneEstimation.h"
#include "PLYWriter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <vector>


using namespace cv;


#define THERSHOLD 0.2  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

#define RANSAC_ITER  200    //RANSAC iteration

#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

int main(int argc, char** argv){
    
    if (argc!=3){
        printf("Usage:\n PlanRansac input.xyz output.ply\n");
        exit(EXIT_FAILURE);
    }
    
    MatrixReaderWriter mrw(argv[1]);
    
    int num=mrw.rowNum;
    
    cout<< "Rows:" << num <<endl;
    cout<< "Cols:" << mrw.columnNum << endl;

    //Read data from text file
    
    vector<Point3f> points;
    
    for (int idx=0;idx<num;idx++){
       double x=mrw.data[3*idx];
       double y=mrw.data[3*idx+1];
       double z=mrw.data[3*idx+2];
       
       float distFromOrigo=sqrt(x*x+y*y+z*z);


//First filter: minimal work distance for a LiDAR limited.        
       
       if (distFromOrigo>FILTER_LOWEST_DISTANCE){
           Point3f newPt;
           newPt.x=x;
           newPt.y=y;
           newPt.z=z;
           points.push_back(newPt);
           }
       
    }
    
    
    //Number of points:

    num=points.size();
    
    
    
    //Estimate plane parameters without robustification
    
    float* plane=EstimatePlaneImplicit(points);
    printf("Plane fitting results for thw whole data:\nA:%f B:%f C:%f D:%f\n",plane[0],plane[1],plane[2],plane[3]);
    
    delete[] plane;
    
    
    //RANSAC-based robust estimation
  // 1st optimal plane
    
        float* planeParams = EstimatePlaneRANSAC(points, THERSHOLD, RANSAC_ITER);

       
        printf("Plane params RANSAC:\n A:%f B:%f C:%f D:%f \n", planeParams[0], planeParams[1], planeParams[2], planeParams[3]);
  
        //Compute differences of the fitted plane in order to separate inliers from outliers

        RANSACDiffs differences = PlanePointRANSACDifferences(points, planeParams, THERSHOLD);


        delete[]  planeParams;

        //--------
        vector<Point3f> inlierPts;
        vector<Point3f> outlierPts;

        for (int idx = 0;idx < num;idx++) {
            if (differences.isInliers.at(idx)) {
                inlierPts.push_back(points.at(idx));
            }
            else {
                //compute the outliers
                outlierPts.push_back(points.at(idx));
            }
        }
            //RANSAC-based robust estimation

       // printf("pionts %f - inliers %f - outliers%f \n ", num, inlierPts.size(), outlierPts.size());

        //2'nd optimal plane
        // find the second optimal plane from the outliers points
        float* planeParams2 = EstimatePlaneRANSAC(outlierPts, THERSHOLD, RANSAC_ITER);


       
        //Compute differences of the fitted plane in order to separate inliers from outliers

        RANSACDiffs differences2 = PlanePointRANSACDifferences(points, planeParams2, THERSHOLD);
        // RANSACDiffs differences2 = PlanePointRANSACDifferences(points, p2, THERSHOLD);
        // RANSACDiffs differences3 = PlanePointRANSACDifferences(points, p3, THERSHOLD);

        delete[]  planeParams2;
        //---------
       
        vector<Point3f> inlierPts2;
        vector<Point3f> outlierPts2;

        for (int idx = 0;idx < outlierPts.size();idx++) {
            if (differences2.isInliers.at(idx)) {
                inlierPts2.push_back(outlierPts.at(idx));
            }
            else {
                //compute the outliers
                outlierPts2.push_back(outlierPts.at(idx));
            }
        }
        //printf("pionts - inliers - outliers:\n ", outlierPts.size(), inlierPts2.size(), outlierPts2.size());

        //3'rd optimal plane
        float* planeParams3 = EstimatePlaneRANSAC(outlierPts2, THERSHOLD, RANSAC_ITER);


        //printf("Plane params RANSAC:\n A:%f B:%f C:%f D:%f \n", planeParams3[0], planeParams3[1], planeParams3[2], planeParams3[3]);
                
        RANSACDiffs differences3 = PlanePointRANSACDifferences(points, planeParams3, THERSHOLD);
        delete[]  planeParams3;
       
        //--------
   // float* planes = SeqRANSAC(points, THERSHOLD, RANSAC_ITER);


    //printf("Planes seqRANSAC:\n A:%f B:%f C:%f D:%f \n", planes, planes[0], planes+1, planes[1]);
    //for (int i = 0;i < 3;i++) {
     //   RANSACDiffs differences = PlanePointRANSACDifferences(points, planes, THERSHOLD);
     //   RANSACDiffs differences2 = PlanePointRANSACDifferences(points, planes+1, THERSHOLD);
     //   RANSACDiffs differences3 = PlanePointRANSACDifferences(points, planes+2, THERSHOLD);
   // }
   
    
    vector<Point3i> colorsRANSAC;
    
    for (int idx=0;idx<num;idx++){
        Point3i newColor;
        // color first plane by green
        if (differences.isInliers.at(idx)){
           newColor.x=0;
           newColor.y=255; 
           newColor.z=0;
        }
         
        else if (differences2.isInliers.at(idx)) {
            // color second plane by blue
            newColor.x = 0;
            newColor.y = 0;
            newColor.z = 255;
        }
        else if (differences3.isInliers.at(idx)) {
            // color third plane by pink
            newColor.x = 255;
            newColor.y = 190;
            newColor.z = 210;
        }
        else{
           newColor.x=255;
           newColor.y=0;
           newColor.z=0;
        }
        
        colorsRANSAC.push_back(newColor);

            
    }
            
    //Write results into a PLY file. 
    //It can be isualized by open-source 3D application Meshlab (www.meshlab.org)

    WritePLY(argv[2],points,colorsRANSAC);
  
      
}
