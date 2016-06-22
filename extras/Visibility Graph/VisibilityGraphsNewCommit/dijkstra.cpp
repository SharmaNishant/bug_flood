/*
 * File:   main.cpp
 * Author: artur
 *
 * Created on 28 kwiecień 2012, 15:48
 */

#include <cstdlib>
#include<cstdio>
#include<vector>
#include<set>
#include "dijkstra.h"
#include <math.h>
#include <climits>
// Program to find Dijkstra's shortest path using STL set
// Program to find Dijkstra's shortest path using STL set
#include<bits/stdc++.h>
# define INF 0x3f3f3f3f




using namespace std;

const int infty = 1000000000; // limit
int verticesNum;
int edgesNum;
vector< vector< pair<int, double> > > comparisionMatrix;
vector<double> pathWeight;
int *myPrev;
int *smallestPath;

void printPath(int dest);


struct cmp // the condition of the mound
{
    // if a is smaller than b

    bool operator() (const int &a, const int &b) {
        if (pathWeight[a] < pathWeight[b]) return true;
        if (pathWeight[a] > pathWeight[b]) return false;
        return a<b;
    }
};

set<int, cmp> kopiec; // ;-)

void prim_dijkstra(int s) //s starting point
{
    int v, u;
    double c;

    pathWeight.clear();
    pathWeight.resize(verticesNum, infty);
    pathWeight[s] = 0;

	myPrev = new int[verticesNum];
	smallestPath=new int[verticesNum];


    kopiec.clear();
    for (int i = 0; i < verticesNum; i++) { //throw vertices, k to the mound
        kopiec.insert(i);
        myPrev[i]=-1;
        smallestPath[i]=-1;
    }

    while (!kopiec.empty()) //go to the mound effect
    {
        u = *(kopiec.begin()); // weź wierzchołek najbliżej drzewa MST
        kopiec.erase(kopiec.begin());

        for (int i = 0; i < comparisionMatrix[u].size(); i++) //połączenia danego wierzchołka i koszty
        {
            v = comparisionMatrix[u][i].first;

            c = comparisionMatrix[u][i].second;
            if (pathWeight[u] + c < pathWeight[v]) // w alg. Prima jest tutaj c < pathWeight[v]
            {
                // uaktualniamy wagę wierzchołka v - poprawnośc przez indukcję, dla co raz większego drzewa mst wybieramy krawędzie o najmiejszej wadze
                kopiec.erase(kopiec.find(v));
                pathWeight[v] = pathWeight[u] + c; // w alg. Prima jest tutaj pathWeight[v] = c ;
                kopiec.insert(v);
                myPrev[v]=u;
                //	mstClosingVertices[v] = u; // domykam krawędź
            }

        }
    }

}

// This class represents a directed graph using
// adjacency list representation
class Graph
{
    int V;    // No. of vertices

    // In a weighted graph, we need to store vertex
    // and weight pair for every edge
    list< pair<int, int> > *adj;

public:
    Graph(int V);  // Constructor

    // function to add an edge to graph
    void addEdge(int u, int v, int w);

    // prints shortest path from s
    vector<int> shortestPath(int s,int d);
};

// Allocates memory for adjacency list
Graph::Graph(int V)
{
    this->V = V;
    adj = new list< pair<int, int> >[V];
}

void Graph::addEdge(int u, int v, int w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

// Prints shortest paths from src to all other vertices
vector<int> Graph::shortestPath(int src,int dest)
{
    // Create a set to store vertices that are being
    // prerocessed
    set< pair<int, int> > setds;

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<int> dist(V, INF);
    vector<int> parent(V,-1);
    vector<int> path;

    // Insert source itself in Set and initialize its
    // distance as 0.
    setds.insert(make_pair(0, src));
    dist[src] = 0;

    /* Looping till all shortest distance are finalized
       then setds will become empty */
    while (!setds.empty())
    {
        // The first vertex in Set is the minimum distance
        // vertex, extract it from set.
        pair<int, int> tmp = *(setds.begin());
        setds.erase(setds.begin());

        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = tmp.second;

        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, int> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            int weight = (*i).second;

            //  If there is shorter path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                /*  If distance of v is not INF then it must be in
                    our set, so removing it and inserting again
                    with updated less distance.
                    Note : We extract only those vertices from Set
                    for which distance is finalized. So for them,
                    we would never reach here.  */
                if (dist[v] != INF)
                    setds.erase(setds.find(make_pair(dist[v], v)));

                // Updating distance of v
                dist[v] = dist[u] + weight;
                parent[v]=u;
                setds.insert(make_pair(dist[v], v));
            }
        }
    }

    // Print shortest distances stored in dist[]
 /*   printf("Vertex   Distance from Source\n");
    for (int i = 0; i < V; ++i)
        printf("%d \t\t %d\n", i, dist[i]);*/
    printf("%d   ", dist[1]);

    int root=dest;
    while(root!=-1)
    {
       // cout<<root<<"============>";
        path.push_back(root);
        root=parent[root];

    }

    return path;

}


vector<int> initiateDijkstra(int numVertice,int numEdges,bool directed,int source,int destination) {
    int a, b, src, dest;
    double c;
    FILE *input = fopen("/home/shivamthukral/Desktop/test.txt", "r+");
    verticesNum = numVertice;
    edgesNum=numEdges;
   // printf("Vertice Num %d , Edge Num %d\n",verticesNum,edgesNum);

    Graph g(numVertice);

    //  making above shown graph


    for (int i = 0; i < edgesNum; i++) {
        fscanf(input, "%d %d %lf", &a, &b, &c); // c = Cost edge from a to b
        //cout<<a<<" "<<b<<" "<<c<<endl;
        g.addEdge(a, b, (int)floor(c));
    }

    vector<int> p=g.shortestPath(source,destination);


    printf("\n");
    return p;
}

/*
 * Prints the shortest path from the source to dest.
 *
 * dijkstra(int) MUST be run at least once BEFORE
 * this is called
 */
void printPath(int dest) {
	static int index=0;
	if (myPrev[dest] != -1)
		printPath(myPrev[dest]);
	printf("%d ", dest);
	smallestPath[index]=dest;
	index++;
}
int * getShortestPath(){
	return smallestPath;
}

