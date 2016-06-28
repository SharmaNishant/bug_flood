//
// Created by shivamthukral on 19/6/16.
//

#ifndef VISIBILITYGRAPHSNEWCOMMIT_OBSTACLEGENERATION_H
#define VISIBILITYGRAPHSNEWCOMMIT_OBSTACLEGENERATION_H

#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <queue>
#include <string>



using namespace std;


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

struct myPoint{
    int x;
    int y;
};

struct myLine{
    myPoint start;
    myPoint end;
};

vector<queue<myLine> > findObstacles(char* fileName,vector<vector<int> > &bm);
vector<queue<myLine> > obstacles(vector<myLine> edges)
{
    vector<queue<myLine> > obs;
    while(edges.empty()==false) {
        queue<myLine> obstacle1;
       // cout << endl;
        obstacle1.push(edges[0]);

        while (!(obstacle1.front().start.x == obstacle1.back().end.x &&
                 obstacle1.front().start.y == obstacle1.back().end.y)) {
            //cout << "YES";
            myLine l1 = obstacle1.back();
           // cout << l1.start.x << "," << l1.start.y << " " << l1.end.x << "," << l1.end.y << endl;

            for (int i = 0; i < edges.size(); i++) {
                if (!(l1.start.x == edges[i].start.x && l1.start.y == edges[i].start.y && l1.end.x == edges[i].end.x &&
                      l1.end.y == edges[i].end.y)) {
                    if (l1.end.x == edges[i].start.x && l1.end.y == edges[i].start.y) {
                        obstacle1.push(edges[i]);
                        edges.erase(edges.begin() + i);

                    }
                }
            }
        }

        //add the last line and remove it from the edges list

        edges.erase(edges.begin() + 0);
        obs.push_back(obstacle1);

    }

    return obs;

}

vector<string> getSourceGoal(char* fileName,vector<int> &sg_int)
{
    vector<string> res;
    ifstream infile(fileName);
    if(!infile.is_open())
    {
        cout<<"Cannot Open Source Goal file File. Exiting.....";
        exit(-1);
    }

    std::string line;
    vector<string> splittedLine;

    //read source
    getline(infile, line);
    splittedLine = split(line, ' ');

    string source = "polygon(("+to_string(stoi(splittedLine[1]))+" " +to_string(stoi(splittedLine[0]))+","+
            to_string(stoi(splittedLine[1]))+" " +to_string(stoi(splittedLine[0]))+"))";
    sg_int.push_back(stoi(splittedLine[1]));
    sg_int.push_back(stoi(splittedLine[0]));


    //read goal
    getline(infile, line);
    splittedLine = split(line, ' ');
    string goal = "polygon(("+to_string(stoi(splittedLine[1]))+" " +to_string(stoi(splittedLine[0]))+","+
                    to_string(stoi(splittedLine[1]))+" " +to_string(stoi(splittedLine[0]))+"))";

    sg_int.push_back(stoi(splittedLine[1]));
    sg_int.push_back(stoi(splittedLine[0]));

    infile.close();

    res.push_back(source);
    res.push_back(goal);
    return res;
}

vector<string> getObstacles(char* fileName,vector<vector<int> > &bm )
{
    vector<string> obs;

    vector<queue<myLine> > obstacles = findObstacles(fileName,bm);

    for(int i=0;i<obstacles.size();i++)
    {
        queue<myLine> obstacle = obstacles[i];
        myLine l= obstacle.front();
        obstacle.pop();
        string str="polygon((";
        str=str+to_string(l.start.x)+" "+to_string(l.start.y)+","+to_string(l.end.x)+" "+to_string(l.end.y)+",";
        while(obstacle.empty()!=true)
        {
            myLine l= obstacle.front();
            obstacle.pop();

            if(obstacle.size()!=0)
                str=str+to_string(l.end.x)+" "+to_string(l.end.y)+",";
            else
                str=str+to_string(l.end.x)+" "+to_string(l.end.y)+"))";
            //cout<<l.start.x<<" "<<l.start.y<<","<<l.end.x<<" "<<l.end.y<<endl;
        }
       // cout<<str<<endl;
        obs.push_back(str);
    }

    return obs;
}

vector<queue<myLine> > findObstacles(char * fileName,vector<vector<int> > &BM)
{
    ifstream fin(fileName);

    if(!fin.is_open())
    {
        cout<<"Cannot open";
        exit(0);
    }

    std::string line1;
    vector<vector<int> > binaryMap;
    std::getline(fin, line1);
    while (std::getline(fin, line1))
    {
        //cout<<line1<<endl;
        vector<string> splitted = split(line1,' ');
        vector<int> res;
        for(int i=0;i<splitted.size();i++)
        {
            res.push_back(stoi(splitted[i]));
        }
        binaryMap.push_back(res);
    }

    vector<vector<int> > bm(102,vector<int>(102));
    for(int i=1;i<bm.size()-1;i++)
    {
        for(int j=1;j<bm[0].size()-1;j++)
        {
            bm[j][i]=binaryMap[i-1][j-1];
        }
    }

    myPoint p1,p2;
    p1.x=-1;
    p1.y=-1;
    p2.x=-1;
    p2.y=-1;
    vector<myLine> edges;
    for(int i=1;i<bm.size()-1;i++)
    {
        for(int j=1;j<bm[0].size()-1;j++)
        {

            if(bm[i][j] == 1)
            {
                int dir = 0;
                if(bm[i-1][j]==0)
                    dir = -1;
                if(bm[i+1][j]==0)
                    dir = 1;

                if(dir != 0){
                    p1.x=i;
                    p1.y=j;

                    if(bm[i][j-1]==1 && bm[i+dir][j-1]==1)
                        p1.y--;

                    while( bm[i][j] == 1  &&  bm[i+dir][j] == 0)
                        j++;
                    j--;

                    if(bm[i][j+1]==1&& bm[i+dir][j + 1] == 1)
                        j++;

                    p2.x=i;
                    p2.y=j;           //remember to change this
                    if(dir==1)
                    {
                        int temp=p1.y;
                        p1.y=p2.y;
                        p2.y=temp;
                    }

                    //cout<<p1.x<<","<<p1.y<<" "<<p2.x<<","<<p2.y<<endl;

                    myLine l_i;
                    l_i.start=p1;
                    l_i.end=p2;
                    edges.push_back(l_i);
                }
            }
        }
    }

    //cout<<"======================="<<endl;
    p1.x=-1;
    p1.y=-1;
    p2.x=-1;
    p2.y=-1;
   // cout<<bm[0].size()<<endl;

    for(int j=1;j<bm[0].size()-1;j++)
    {
        for(int i=1;i<bm.size()-1;i++) {

            if (bm[i][j] == 1) {
                int dir = 0;
                if (bm[i][j - 1] == 0)
                    dir = -1;
                if (bm[i][j + 1] == 0)
                    dir = 1;

                if (dir != 0) {
                    p1.x = i;
                    p1.y = j;

                    if(bm[i-1][j]==1 && bm[i-1][j+dir]==1)
                        p1.x--;

                    while (bm[i][j] == 1 && bm[i][j + dir] == 0)
                        i++;
                    i--;

                    if(bm[i+1][j]==1&& bm[i+1][j + dir] == 1)
                        i++;

                    p2.x = i;
                    p2.y = j;           //remember to change this

                    if(dir==-1)
                    {
                        int temp=p1.x;
                        p1.x=p2.x;
                        p2.x=temp;
                    }

                    //cout << p1.x << "," << p1.y << " " << p2.x << "," << p2.y << endl;

                    myLine l_i;
                    l_i.start = p1;
                    l_i.end = p2;
                    edges.push_back(l_i);
                }
            }


        }
    }


    /*Mat image;
    image = imread("/home/shivamthukral/ur_ws/src/bug_flood/obstacles/10_Obstacles/image_1.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        exit(0);
    }

    for(int i=0;i<edges.size();i++)
    {
        // line(image,Point(edges[i].start.x-1,edges[i].start.y-1),Point(edges[i].end.x-1,edges[i].end.y-1),190);
    }

    namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);*/

    vector<queue<myLine> > obs = obstacles(edges);
    BM=binaryMap;
    return obs;
}

#endif //VISIBILITYGRAPHSNEWCOMMIT_OBSTACLEGENERATION_H
