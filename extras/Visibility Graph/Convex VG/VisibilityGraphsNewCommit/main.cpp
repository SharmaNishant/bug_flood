/*
 * main.cpp
 *
 *  Created on: Oct 2, 2013
 *      Author: nut
 */
#include <iostream>
#include "obstacles.h"
#include "boostHelper.h"
#include "obstacleController.h"
#include "drawing.h"
#include "utility.h"
#include <vector>
#include "VisibilityGraphController.h"
#include "VisibilityGraph.h"
#include "graphutility.h"
#include "dijkstra.h"
#include <time.h>
#include <fstream>
#include "obstacleGeneration.h"

clock_t startTime;
void findShortestPath(VisibilityGraph* vg, double sourceX, double sourceY,
                      double destX, double destY);
void drawObs(Obstacle* obs, Point* ori);
void drawAndWriteFileVisEdges(vector<Line*> visEdges);


int main(int argc, char* argv[]) {

    // Check the number of parameters
    if (argc < 3) {
        // Tell the user how to run the program
        std::cerr << "Input binary File name and source goal file !"<< std::endl;
        return 1;
    }
    vector<int> sg_int;
    vector<vector<int> > bm;

    //vector<string> res = getObstacles(argv[1],bm);
    vector<string> res = getConvexObstacles(argv[1]);
    vector<string> sg = getSourceGoal(argv[2],sg_int);


    startTime = clock();
    vector<Obstacle*> obsList;
    Obstacle *obs;

    //Create a list of test obstacle
   /* Obstacle* obs = createObstacle(
            "polygon((40 140,40 300,100 280,120 200,100 140,40 140))");
    obsList.push_back(obs);

    obs = createObstacle("polygon((200 40,280 40,280 200,180 200,200 40))");
    obsList.push_back(obs);

    = createObstacle("polygon((30 330,180 350,120 450,30 330))");
    obsList.push_back(obs);

    obs = createObstacle("polygon((400 320,520 320,500 520,400 530,400 320))");
    obsList.push_back(obs);

    obs = createObstacle("polygon((40 20,40 20))"); //For data point
    obsList.push_back(obs);

    obs = createObstacle("polygon((440 20,440 20))"); //For data point
    obsList.push_back(obs);

    obs = createObstacle("polygon((550 100,550 100))"); //For data point
    obsList.push_back(obs);

    obs=createObstacle("polygon((250 400,320 400,320 500,250 400))");
    obsList.push_back(obs);

    obs=createObstacle("polygon((60 20,250 20,250 200,60 200,60 20))");
    obsList.push_back(obs);


    obs=createObstacle("polygon((140 350,190 350,190 250,140 250,140 350))");
    obsList.push_back(obs);

    obs=createObstacle("polygon((200 20,500 20,500 200,400 200,400 80,300 80,300 200,200 200 200 20))");
    obsList.push_back(obs);
*/

  /*  obs = createObstacle("polygon((40 20,40 20))"); //For data point
    obsList.push_back(obs);

    obs = createObstacle("polygon((440 20,440 20))"); //For data point
    obsList.push_back(obs);

    obs=createObstacle("polygon((100 150,150 150,150 50,100 50,100 150))");
    obsList.push_back(obs);
    obs=createObstacle("polygon((250 400,320 400,320 500,250 400))");
    obsList.push_back(obs);*/

     obs = createObstacle(sg[0]); //For data point source
     obsList.push_back(obs);
     obs = createObstacle(sg[1]); //For data point goal
     obsList.push_back(obs);

     for(int k=0;k<res.size();k++)
     {
         obs = createObstacle(res.at(k)); //For data point
         obsList.push_back(obs);
     }

    obsList.at(10)->print();

    //Create the initial Visibility graph
    VisibilityGraph* visGraph = new VisibilityGraph(obsList);
    visGraph->print();
    VisibilityGraphController* vg = new VisibilityGraphController(visGraph);

    vector<Line*> visEdges = vg->constructVisGraph();


    vector<Line*> obsSide = visGraph->obsSides;

    //removeing inside edges

/*    for(int q=0;q<visGraph->edges.size();q++)
    {
        Line * l=visGraph->edges.at(q);
        int x1=(int) l->a->x*scale;
        int y1=(int)l->a->y*scale;
        int x2=(int)l->b->x*scale;
        int y2=(int)l->b->y*scale;

        int  x = (int) floor((x1+x2)/2);
        int  y = (int) floor((y1+y2)/2);
      //  cout<<"------->"<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
        if(bm[y][x]==1)
        {
           /// cout<<"*****"<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
            visGraph->edges.erase(visGraph->edges.begin()+q);
            q--;
        }
    }*/


//adding the obs as well
    for(int z=2;z<obsList.size();z++)
    {
        vector<Line*> obsEdges = obsList[z]->edges;
        visGraph->edges.insert(visGraph->edges.end(),obsEdges.begin(),obsEdges.end());
    }


    //Necessary Drawing
    //visGraph->print();
    visEdges.clear();
    vector<Line *> visEdges1 = visGraph->edges;
    obsSide = visGraph->obsSides;
    vector<Obstacle*> visObs = visGraph->obstacles;
    //Draw the Obstacle Sides id in the image
/*    for (int i = 0; i < obsSide.size(); i++) {
        Line* l = obsSide[i];
        drawText(((l->a->x) + (l->b->x)) / 2, ((l->a->y) + (l->b->y)) / 2,
                 l->id, WHITE);
        drawEdge(l,WHITE);
    }*/

    //Write the edges in a file so that Dijkstra can use it
    drawAndWriteFileVisEdges(visEdges1);

    //This point is not used , i created it for testign purpose
/*
    Point* ori = new Point(440, 120);
    for (int i = 0; i < visObs.size(); i++) {
        //obsList[i]->print();
        drawObs(visObs[i], ori);

    }
*/

    //Find the shortest path from two points using Dijkstra
    findShortestPath(visGraph,sg_int[0],sg_int[1],sg_int[2],sg_int[3]);
    //findShortestPath(visGraph, 40, 20, 550, 100);
    //findShortestPath(visGraph,400,530 , 650, 400);
    /* Calculate the time */
    printf("%f\n",
           ((double) clock() - startTime) / CLOCKS_PER_SEC);

   // displayImage();

    for(int k=0;k<obsList.size();k++)
    {
        for(int j=0;j<obsList[k]->edges.size();j++)
        {
            delete obsList[k]->edges[j];
        }

        for(int j=0;j<obsList[k]->vertices.size();j++)
        {
            delete obsList[k]->vertices[j];
        }

        delete obsList[k];

    }
    delete vg;
    delete visGraph;

    return 0;
}

void drawAndWriteFileVisEdges(vector<Line*> visEdges) {
    //Remove existing test.txt file
    if (remove("/home/shivamthukral/Desktop/test.txt") != 0)
        perror("Error deleting file");
    else
       // puts("File successfully deleted");

    for (int i = 0; i < visEdges.size(); i++) {
        drawEdge(visEdges[i], BLUE);
        fileWrite(visEdges[i]->a, visEdges[i]->b);
    }
}
void drawObs(Obstacle* o, Point* ori) {

    vector<Point*> vertexList = getVertices(o);
    int size = vertexList.size();
    CImg<double> points(size, 2);
    double ps[size * 2];
    int i = 0;
    for (std::vector<Point*>::iterator it = vertexList.begin();
         it < vertexList.end(); ++it) {
        ps[i] = (*it)->x;
        i++;
        ps[i] = (*it)->y;
        i++;
        drawCircle((*it)->x, (*it)->y, 1.5, WHITE);
        drawText((*it)->x, (*it)->y, (*it)->id, RED);
        //Draw Lines for angle sorting
        // drawLine((*it)->x,(*it)->y,ori->x,ori->y);
    }

    double *iterator = ps;
    cimg_forX(points,i)
    {
        points(i, 0) = *(iterator++)*scale;
        points(i, 1) = *(iterator++)*scale;
    }
    drawPolygon(points);

}

void findShortestPath(VisibilityGraph* vg, double sourceX, double sourceY,
                      double destX, double destY) {
    int numOfPoints = vg->nodes.size();
    int numOfEdges = vg->edges.size();
    int sourcePointId = searchPointByCoord(vg->nodes, sourceX, sourceY)->id;
    int destPointId = searchPointByCoord(vg->nodes, destX, destY)->id;
   /* printf("\nFinding shortest path from %d -> %d\n", sourcePointId,
           destPointId);*/
    Point* start;
    Point* goal;

    vector<int> path=initiateDijkstra(numOfPoints, numOfEdges, false, sourcePointId,
                     destPointId);

    //Print the Shortest Path
    /*printf("The Shortest Path is :");*/
    /*for(int j=0;j<path.size()-1;j++) {
        printf("%d ", path[j]);

        if (path[j] != -1) {
            start = getPointById(vg->nodes, path[j]);
            goal = getPointById(vg->nodes, path[j + 1]);
            // Visualize:
           *//* drawCircle(start->x, start->y, 2, GREEN);
            drawCircle(goal->x, goal->y, 2, GREEN);
            drawLine(start->x, start->y, goal->x, goal->y, GREEN);*//*
        }
    }*/

}

