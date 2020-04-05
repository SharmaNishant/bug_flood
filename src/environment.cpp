//
// Created by nishant on 15/5/16.
//

#include <bug_flood/environment.h>
#include <bug_flood/helper_functions.h>

/**
#ifdef OBSTACLE_IO_DEBUG

#endif
 */

Environment::Environment(string sourceGoal, int length, int width)
    : map(length, width) {
  ReadSourceGoal(sourceGoal);
}

Environment::Environment(string sourceGoal, string mapFile) : map(mapFile) {
  ReadSourceGoal(sourceGoal);
  generateObstacleLineMap();
}

Point Environment::getSource() { return this->source; }

Point Environment::getGoal() { return this->goal; }

int Environment::getEnvironmentLength() { return this->map.getRowSize(); }

int Environment::getEnvironmentWidth() { return this->map.getColSize(); }

void Environment::getEnvironmentDimensions(int &length, int &width) {
  length = getEnvironmentLength();
  width = getEnvironmentWidth();
}

vector<Point> Environment::getObstructedLocations(int &rowSize, int &colSize) {
  return this->map.getObstructedLocations(rowSize, colSize);
}

bool Environment::isObstructed(Point location) {
  //	Point loc = *(Point*)location;
  return this->map.isObstructed(location);
}

void Environment::ReadSourceGoal(string sourceGoal) {
  ifstream infile(sourceGoal);
  if (!infile.is_open()) {
    cout << "Cannot Open Source Goal file File. Exiting.....";
    exit(-1);
  }

  std::string line;
  vector<string> splittedLine;

  // read source
  getline(infile, line);
  splittedLine = split(line, ' ');
  this->source.x = stoi(splittedLine[0]);
  this->source.y = stoi(splittedLine[1]);
  this->source.z = 0;

  // read goal
  getline(infile, line);
  splittedLine = split(line, ' ');
  this->goal.x = stoi(splittedLine[0]);
  this->goal.y = stoi(splittedLine[1]);
  this->goal.z = 0;
  infile.close();
}

double Environment::isVisited(double row, double col) {
  //	return this->map.isVisited(row,col); //Not using map based visited
  //information anymore
  /*
   * New approach
   */
  for (VisitInfo &point : this->visited) {
    if (point.location.x == row && point.location.y == col)
      return point.cost;
  }
  return -1;
}

double Environment::isVisited(Point location) {
  //	return this->map.isVisited(row,col); //Not using map based visited
  //information anymore
  /*
   * new approach
   */
  for (VisitInfo &point : this->visited) {
    if (point.location.x == location.x && point.location.y == location.y)
      return point.cost;
  }
  return -1;
}

void Environment::setVisited(double row, double col, double cost) {
  //	this->map.setVisited(row,col);
  Point point;
  point.x = row;
  point.y = col;
  point.z = 0;
  this->setVisited(point, cost);
}

void Environment::setVisited(Point location, double cost) {
  bool found = false;
  for (int i = 0; i < this->visited.size(); ++i) {
    if (visited[i].location.x == location.x &&
        visited[i].location.y == location.y) {
      found = true;
      if (cost < visited[i].cost) {
        visited[i].cost = cost;
      }
    }
  }

  if (!found) {
    VisitInfo visitInfo;
    visitInfo.location = location;
    visitInfo.cost = cost;
    this->visited.push_back(visitInfo);
  }
}

bool Environment::getObstacleIntersection(Point start, Point end,
                                          Point &intersection, double &distance,
                                          int &boundaryID, Point &location) {
  Line line;
  line.start = start;
  line.end = end;
  return this->getObstacleIntersection(line, intersection, distance, boundaryID,
                                       location);
}

bool Environment::getObstacleIntersection(Line line, Point &intersection,
                                          double &distance, int &boundaryID,
                                          Point &location) {
  bool result = false;
  double minDistance = INT_MAX;
  Point minIntersection;
  int minBoundaryID = -1;
  for (auto &obsLine : this->lines) {
    if (obsLine.first == boundaryID)
      continue; // skip test for same line
    bool tresult = IsIntersecting(line, obsLine.second, intersection, distance);

    if (location.x == intersection.x && location.y == intersection.y) {
      // set which direction it should move in now
      continue;
    }

    if (distance < minDistance) {
      result = true;
      minDistance = distance;
      minIntersection = intersection;
      minBoundaryID = obsLine.first;
    }
  }

  // set the min distance once we are done
  distance = minDistance;
  intersection = minIntersection;
  boundaryID = minBoundaryID;
  return result;
}

bool Environment::getNextBoundaryLine(Point location, int &boundaryID,
                                      Point &tempGoal) {
  // copy boundary id for static reference
  int boundary = boundaryID;
  for (auto &obsLine : this->lines) {
    if (obsLine.first == boundary) {
      continue;
    }
    if (obsLine.second.start.x == location.x &&
        obsLine.second.start.y == location.y) {
      // set which direction it should move in now
      tempGoal = obsLine.second.end;
      boundaryID = obsLine.first;
      return true;
    }
    if (obsLine.second.end.x == location.x &&
        obsLine.second.end.y == location.y) {
      // set which direction it should move in now
      tempGoal = obsLine.second.start;
      boundaryID = obsLine.first;
      return true;
    }
  }
  return false;
}

Line Environment::getLine(int id) {
  ObstacleLines::iterator it;
  it = this->lines.find(id);
  if (it != this->lines.end()) {
    return it->second;
  } else {
    assert(!"TRYING TO ACCESS A NON EXISTING LINE!!! EXITING");
  }
}

Line GenLine(double oneX, double oneY, double twoX, double twoY) {
  Line line;
  line.start.x = oneX;
  line.start.y = oneY;
  line.start.z = 0;
  line.end.x = twoX;
  line.end.y = twoY;
  line.end.z = 0;
  return line;
}

void Environment::generateObstacleLineMap() {

#ifdef CONVEX
  ifstream fin("/tmp/bug_flood_lines.txt");
  if (!fin.is_open()) {
    cout << "CANNOT OPEN LINES FILE" << endl;
    exit(-1);
  }
  string line;

  int i = 0;
  while (getline(fin, line)) {
    vector<string> splitted = split(line, ' ');
    this->lines[++i] = GenLine(stoi(splitted[0]), stoi(splitted[1]),
                               stoi(splitted[2]), stoi(splitted[3]));
    this->lines[++i] = GenLine(stoi(splitted[2]), stoi(splitted[3]),
                               stoi(splitted[4]), stoi(splitted[5]));
    this->lines[++i] = GenLine(stoi(splitted[4]), stoi(splitted[5]),
                               stoi(splitted[6]), stoi(splitted[7]));
    this->lines[++i] = GenLine(stoi(splitted[6]), stoi(splitted[7]),
                               stoi(splitted[8]), stoi(splitted[9]));

    // cout<<i<<endl;
  }
#else
  // create a temp copy of map
  vector<vector<bool>> binaryMap(this->map.getRowSize() + 2,
                                 vector<bool>(this->map.getColSize() + 2));

  for (int i = 1; i < binaryMap.size() - 1; i++) {
    for (int j = 1; j < binaryMap[0].size() - 1; j++) {
      binaryMap[i][j] = this->map.at(i - 1, j - 1);
    }
  }
  // padding
  for (int i = 0; i < binaryMap.size(); i++) {
    binaryMap[i][0] = false;
    binaryMap[i][binaryMap[0].size() - 1] = false;
  }

  for (int j = 0; j < binaryMap[0].size(); j++) {
    binaryMap[0][j] = false;
    binaryMap[binaryMap.size() - 1][j] = false;
  }

  Point p1, p2;
  p1.x = -1;
  p1.y = -1;
  p2.x = -1;
  p2.y = -1;
  vector<Line> edges;
  // horizontal edges
  for (int i = 1; i < binaryMap.size() - 1; i++) {
    for (int j = 1; j < binaryMap[0].size() - 1; j++) {

      if (binaryMap[i][j] == 1) {
        int dir = 0;
        if (binaryMap[i - 1][j] == 0)
          dir = -1;
        if (binaryMap[i + 1][j] == 0)
          dir = 1;

        if (dir != 0) {
          p1.x = i;
          p1.y = j;
          while (binaryMap[i][j] == 1 && binaryMap[i + dir][j] == 0)
            j++;

          // j--;

          p2.x = i;
          p2.y = j; // remember to change this
          if (dir == 1) {
            p2.x++;
            p1.x++;
          }
          // cout<<p1.y-1<<","<<p1.x-1<<" "<<p2.y-1<<","<<p2.x-1<<endl;
          Line l_i;
          l_i.start = p1;
          l_i.end = p2;
          edges.push_back(l_i);
        }
      }
    }
  }

  // verticle edges
  p1.x = -1;
  p1.y = -1;
  p2.x = -1;
  p2.y = -1;
  // cout<<binaryMap[0].size()<<endl;

  for (int j = 1; j < binaryMap[0].size() - 1; j++) {
    for (int i = 1; i < binaryMap.size() - 1; i++) {

      if (binaryMap[i][j] == 1) {
        int dir = 0;
        if (binaryMap[i][j - 1] == 0)
          dir = -1;
        if (binaryMap[i][j + 1] == 0)
          dir = 1;

        if (dir != 0) {
          p1.x = i;
          p1.y = j;
          while (binaryMap[i][j] == 1 && binaryMap[i][j + dir] == 0)
            i++;

          // i--;

          p2.x = i;
          p2.y = j; // remember to change this
          if (dir == 1) {
            p2.y++;
            p1.y++;
          }
          // cout << p1.y -1<< "," << p1.x-1 << " " << p2.y-1 << "," << p2.x-1
          // << endl;

          Line l_i;
          l_i.start = p1;
          l_i.end = p2;
          edges.push_back(l_i);
        }
      }
    }
  }

  for (int i = 0; i < edges.size(); i++) {
    this->lines[i + 1] = GenLine(edges[i].start.y - 1, edges[i].start.x - 1,
                                 edges[i].end.y - 1, edges[i].end.x - 1);
  }

  /*	this->lines[1] 	= GenLine(6  ,2  ,10 ,2); //there
          this->lines[2] 	= GenLine(6  ,2  ,6  ,4); //there
          this->lines[3] 	= GenLine(6  ,4  ,10 ,4);
          this->lines[4] 	= GenLine(10 ,4  ,10 ,2);

          this->lines[5] 	= GenLine(4  ,6  ,4  ,13); //there
          this->lines[6] 	= GenLine(4  ,13 ,13 ,13);
          this->lines[7] 	= GenLine(13 ,13 ,13 ,6);
          this->lines[9] 	= GenLine(13 ,6  ,11 ,6); //there
          this->lines[10] = GenLine(11 ,6  ,11 ,11); // there
          this->lines[11] = GenLine(11 ,11 ,6  ,11); //there
          this->lines[12] = GenLine(6  ,11 ,6  ,6);
          this->lines[8] 	= GenLine(6  ,6  ,4  ,6); //There*/

#endif
}

ObstacleLines Environment::getObstacleLines() { return this->lines; }
