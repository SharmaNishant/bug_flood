//
// Created by shivamthukral on 26/5/16.
//


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include<iostream>
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>


using namespace cv;
using namespace std;

struct coordinates{
    int x;
    int y;
    int height;
    int width;
};

struct Position
{
    float x, y;
};

char sequence[500];
char sequence_image[500];
char sequence__final_image[500];
//char sequence_cfg_data[500];

vector<coordinates> Points;

Position getNewRandomStartPosition()
{
    Position pos;
    pos.x = rand() % 10;
    pos.y = rand() % 100;
    return pos;
}

Position getNewRandomEndPosition()
{
    Position pos;
    pos.x = (rand() % 10) + 90;
    pos.y = (rand() % 10);
    return pos;
}



bool checkInsideObstacles(vector<vector<float> > binaryMap, Position point)
{
    //bound check
    if(point.x<2||point.x>binaryMap.size()-2||point.y<2||point.y>binaryMap[0].size()-2)
        return true;

    if(binaryMap[point.y][point.x]==1||binaryMap[point.y+1][point.x]==1||binaryMap[point.y][point.x+1]==1||
            binaryMap[point.y+1][point.x+1]==1||binaryMap[point.y-1][point.x]==1||binaryMap[point.y][point.x-1]==1||
            binaryMap[point.y-1][point.x-1]==1||binaryMap[point.y-1][point.x+1]==1||binaryMap[point.y+1][point.x-1]==1)
        return true;
    else
        return false;
}

Position getSource(vector<vector<float> > binaryMap)
{
    Position source;
    while(true) {
        source = getNewRandomStartPosition();
        if (!checkInsideObstacles(binaryMap, source)) {
            break;
        }
    }
    return source;
}

Position getGoal(vector<vector<float> > binaryMap)
{
    Position goal;
    while(true) {
        goal = getNewRandomEndPosition();
        if (!checkInsideObstacles(binaryMap, goal))
        {
            //cout<<goal.x<<" "<<goal.y<<" "<<binaryMap[goal.x][goal.y]<<endl;
            break;
        }
    }
    return goal;
}



int main(int argc, char* argv[])
{

    if(argc<3)
    {
        cout<<"Enter filename and obstacle number\n";
        exit(0);
    }

    int frameWidth =0,frameHeight = 0,obsNo = atoi(argv[1]);
    frameHeight = 100;
    frameWidth = 100;

    int maxHeight = 6 * frameHeight/obsNo;
    int maxWidth =  6 * frameWidth/obsNo;
    int minDist=5;

    srand(time(NULL));


    for(int z = 1 ; z <= 1000 ;z++)
    {
        cv::Mat img =(Mat) cv::Mat::zeros(frameWidth,frameHeight,CV_8UC1);
        Points.clear();
        snprintf(sequence, 500, "sg_%d.txt", z);
        snprintf(sequence_image, 500, "map_%d.txt", z);
        snprintf(sequence__final_image, 500, "image_%d.jpg", z);
        //snprintf(sequence_cfg_data, 500, "cfg_data_%d.cfg", z);


        string sg_filename(argv[2]);
        sg_filename.append(sequence);

        string map_filename(argv[2]);
        map_filename.append(sequence_image);

        string image_filename(argv[2]);
        image_filename.append(sequence__final_image);


        cout<<sg_filename<<endl;
        cout<<map_filename<<endl;
        cout<<image_filename<<endl;
       // cout<<sequence_cfg_data<<endl;

        ofstream fout(sg_filename);
        ofstream fout_image(map_filename);
       // ofstream fout_cfg(sequence_cfg_data);

        int i = 0;
        while(i<obsNo)
        {
            coordinates tempPoint;
            tempPoint.x = frameWidth * ((float)rand()/(float)RAND_MAX);
            tempPoint.y = frameHeight * ((float)rand()/(float)RAND_MAX);

            tempPoint.height = 4 + maxHeight * ((float)rand()/(float)RAND_MAX);
            tempPoint.width = 4 + maxWidth * ((float)rand()/(float)RAND_MAX);

            if((tempPoint.x<5)||(tempPoint.y<5)||(tempPoint.y + tempPoint.height) > frameHeight-10 ||
               (tempPoint.x + tempPoint.width) > frameWidth-10){
               // cout<<"YES"<<endl;
            }

            else
            {
                int check = 0;
                for(int j = 0;j<i;j++)
                {
                    coordinates checkPoint = Points[j];

                    Rect tempRect(checkPoint.x,checkPoint.y,checkPoint.width,checkPoint.height);
                    Rect tempRect2(tempPoint.x,tempPoint.y,tempPoint.width,tempPoint.height);

                    Rect intersect = tempRect & tempRect2;
                    Rect orArea = tempRect | tempRect2;

                    if((intersect.area() > 0) || (orArea.area() < (tempRect2.area() + tempRect.area()) ))
                    {
                        //intersect
                        check = 1;
                        break;
                    }

                    //check for virtual rectangle
                    if(check != 1)
                    {
                        Rect currentRect(tempPoint.x,tempPoint.y,tempPoint.width,tempPoint.height);
                        Rect virtualRect(checkPoint.x-minDist,checkPoint.y-minDist,checkPoint.width+(2*minDist),checkPoint.height+(2*minDist));
                        Rect intersect = currentRect & virtualRect;
                        Rect orArea = currentRect | virtualRect;

                        if((intersect.area() > 0) || (orArea.area() < (tempRect2.area() + tempRect.area()) ))
                        {
                            //intersect
                            check = 2;
                            break;
                        }
                    }
                }

                if(check == 1)
                {
                    check = 0;
                    //if intersection merge the two rectangle
                    i++;
                    Points.push_back(tempPoint);

                    for(int x = tempPoint.x ; x < (tempPoint.x+tempPoint.width) ;x++)
                    {
                        for(int y = tempPoint.y ;y < (tempPoint.y+tempPoint.height);y++)
                        {
                            img.at<uchar>(x,y) = 255;

                        }
                    }
                }

                else if(check == 0)
                {
                    i++;
                    Points.push_back(tempPoint);

                    for(int x = tempPoint.x ; x < (tempPoint.x+tempPoint.width) ;x++)
                    {

                        for(int y = tempPoint.y ;y < (tempPoint.y+tempPoint.height);y++)
                        {
                            img.at<uchar>(x,y) = 255;
                        }
                    }
                }
                else if(check==2)
                {
                    //Do nothing ignore
                    check=0;
                }
            }
        }
        //cfg file config added
       // fout_cfg<<"discretization(cells): "<<(float) img.rows<<" "<<img.cols<<endl;
        //fout_cfg<<"obsthresh: 1"<<endl;

        vector<vector<float> > binaryMap(img.rows,vector<float>(img.cols));
        //output the zero one matrix
        fout_image<<(float) img.rows<<" "<<img.cols<<endl;
        for(int t=0;t<img.rows;t++)
        {
            for(int s=0;s<img.cols;s++)
            {
                fout_image<<(float) (img.at<uchar>(t,s)/255)<<" ";
                binaryMap[t][s]=(float) (img.at<uchar>(t,s)/255);
            }

            fout_image<<endl;
        }

        //Testing
        cv::Mat img1 =(Mat) cv::Mat::zeros(frameWidth,frameHeight,CV_8UC1);

        for(int i=0;i<img1.rows;i++)
        {
            for(int j=0;j<img1.cols;j++)
            {
                if(binaryMap[i][j]==1)
                    img1.at<uchar>(i,j)=255;
            }
        }


        Position source = getSource(binaryMap);
        Position goal = getGoal(binaryMap);

        fout<<source.y<<" "<<source.x<<"\n";
        fout<<goal.y<<" "<<goal.x<<"\n";
        //fout_cfg<<"start(cells): "<<source.y<<" "<<source.x<<"\n";
        //fout_cfg<<"end(cells): "<<goal.y<<" "<<goal.x<<"\n";
       // fout_cfg<<"environment:\n";
        for(int t=0;t<img.rows;t++)
        {
            for(int s=0;s<img.cols;s++)
            {
              //  fout_cfg<<(float) (img.at<uchar>(t,s)/255)<<" ";
                binaryMap[t][s]=(float) (img.at<uchar>(t,s)/255);
            }

            //fout_cfg<<endl;
        }


        circle(img,Point(source.x,source.y),1,100);
        circle(img,Point(goal.x,goal.y),1,255);

        /// Show in a window
        namedWindow( "Image", CV_WINDOW_NORMAL);
        imshow( "Image", img );
        cvWaitKey(10);
        //string str(sequence__final_image);
        imwrite(image_filename, img );

        namedWindow( "Image-test", CV_WINDOW_NORMAL);
        imshow( "Image-test", img1 );
        cvWaitKey(10);


        fout.close();
        fout_image.close();
        //fout_cfg.close();
    }

    return 0;
}


