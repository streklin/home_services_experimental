#ifndef _INCL_GRAPH
#define _INCL_GRAPH

#include <vector>
#include <string>
#include "Vertex.cpp"

using namespace std;

class Graph {
private:
  vector<int> adjacency[100];
  vector<Vertex*> vertexList;

  bool isEdge(int v1, int v2);
public:
  Graph();
  void addVertex(Vertex* v);
  void addEdge(int v1, int v2);
  Vertex* getClosestVertex(float x, float y);
  Vertex* getClosestVertexInBall(float x, float y, float epsilon);
  vector<Vertex*> getConnectedVertices(int v1);
  Vertex* getVertexByIndex(int index);
  Vertex* getVertexByLabel(string label);
  int getSize();
  vector<Vertex*> findPath(Vertex* start, Vertex* goal);
};

Graph::Graph() {}

void Graph::addVertex(Vertex* v) {
  if (this->getSize() >= 100) {
    throw "Graph is full";
  }

  this->vertexList.push_back(v);
}

bool Graph::isEdge(int v1, int v2) {
  ROS_INFO("isEdge");

  for(int i = 0; i < this->adjacency[v1].size(); i++) {
    if (this->adjacency[v1][i] == v2) return true;
  }

  return false;
}

void Graph::addEdge(int v1, int v2) {
  ROS_INFO("addEdge");

  if (this->isEdge(v1, v2)) return;

  this->adjacency[v1].push_back(v2);
  this->adjacency[v2].push_back(v1);
}

Vertex* Graph::getClosestVertex(float x, float y) {
  float distance = 10000000;
  Vertex* result = NULL;

  if (this->vertexList.size() == 0) return result;

  for(int i = 0; i < this->vertexList.size(); i++) {
    Vertex* v = this->vertexList[i];
    float d = pow(v->getX() - x, 2) + pow(v->getY() - y, 2);

    if (d < distance) {
      distance = d;
      result = v;
    }
  }

  return result;
}

Vertex* Graph::getClosestVertexInBall(float x, float y, float epsilon) {

  Vertex* closest = this->getClosestVertex(x,y);

  if (closest == NULL) return NULL;

  float distance = sqrt( pow(x - closest->getX(), 2) + pow(y - closest->getY(), 2));

  if (distance > epsilon) return NULL;

  return closest;

}

vector<Vertex*> Graph::getConnectedVertices(int v1) {
  vector<Vertex*> result;

  for(int i = 0; i < this->adjacency[v1].size(); i++) {
    int vIdx = this->adjacency[v1][i];
    Vertex* v = this->vertexList[vIdx];
    result.push_back(v);
  }

  return result;
}

int Graph::getSize() {
  return this->vertexList.size();
}

Vertex* Graph::getVertexByIndex(int index) {
  if (index < 0) return NULL;

  for(int i = 0; i < this->vertexList.size(); i++) {
    if (this->vertexList[i]->getIndex() == index) {
      return this->vertexList[i];
    }
  }

  return NULL;
}

Vertex* Graph::getVertexByLabel(string label) {
  for(int i = 0; i < this->vertexList.size(); i++) {
    if (this->vertexList[i]->getLabel() == label) {
      return this->vertexList[i];
    }
  }

  return NULL;
}

vector<Vertex*> Graph::findPath(Vertex* start, Vertex* goal) {
  vector<Vertex*> path;

  vector<Vertex*> stack;
  vector<Vertex*> visited;

  start->setParent(-1);
  stack.push_back(start);

  bool goalReached = false;
  while(stack.size() > 0 && !goalReached) {

    Vertex* next = stack.back();
    stack.pop_back();

    visited.push_back(next);

    if (goal->getIndex() == next->getIndex()) {
      goalReached = true;
      continue;
    }

    vector<Vertex*> frontier = this->getConnectedVertices(next->getIndex());

    for(int i = 0; i < frontier.size(); i++) {
      Vertex* fv = frontier[i];

      bool found = false;
      for(int j = 0; j < visited.size(); j++) {
        Vertex* vv = visited[j];

        if (fv->getIndex() == vv->getIndex()) {
          found = true;
          break;
        }
      }

      if (!found) {
        fv->setParent(next->getIndex());
        stack.push_back(fv);
      }
    }
  }

  // extract the path
  Vertex* dest = visited.back();

  while(dest->getParent() > 0) {
    path.push_back(dest);
    dest = this->getVertexByIndex(dest->getParent());
  }

  return path;

}

#endif
