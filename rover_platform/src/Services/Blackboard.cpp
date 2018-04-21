#ifndef _INCL_BLACKBOARD
#define _INCL_BLACKBOARD

#include <string>
#include "../Graph/Graph.cpp"
#include "../Graph/Vertex.cpp"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

/*
Blackboard operations needed:

vector<Vertex*> getConnectedVertices(int v1);
Vertex* getVertexByIndex(int index);
Vertex* getVertexByLabel(string label);
int getSize();
*/


class Blackboard {
private:
  Graph* g;
  Vertex* currentLocation;
  string createError(string errorMsg);
public:
  Blackboard();
  string queryCurrentLocation();
  void addVertex(string query);
  void addEdge(string query);
  void setCurrentLocation(string query);
  string getClosestVertex(string query);
  string getClosestVertexInBall(string query);
};

Blackboard::Blackboard() {
  this->g = new Graph();
  this->currentLocation = NULL;
}

string Blackboard::createError(string errorMsg) {
  json response = {
    {"Error", errorMsg}
  };

  return response.dump();
}

string Blackboard::queryCurrentLocation() {
  if (this->currentLocation == NULL) return this->createError("There is no current vertex!");
  return this->currentLocation->getJson();
}

void Blackboard::addVertex(string query) {
  auto vertexData = json::parse(query);

  float x = vertexData["Vertex"]["x"];
  float y = vertexData["Vertex"]["y"];
  int index = vertexData["Vertex"]["index"];

  Vertex* v = new Vertex(x,y,index);
  this->g->addVertex(v);
}

void Blackboard::setCurrentLocation(string query) {
  auto locationData = json::parse(query);

  int index = locationData["Location"]["index"];
  Vertex* v = this->g->getVertexByIndex(index);
  this->currentLocation = v;
}

void Blackboard::addEdge(string query) {
  auto edgeData = json::parse(query);

  int v1 = edgeJson["Edge"]["v1"];
  int v2 = edgeJson["Edge"]["v2"];

  this->g->addEdge(v1, v2);
}

string Blackboard::getClosestVertex(string query) {
  auto locationData = json::parse(query);

  float x = locationData["Location"]["x"];
  float y = locationData["Location"]["y"];

  Vertex* v = this->g->getClosestVertex(x, y);
  return v->getJson();
}

string Blackboard::getClosestVertexInBall(string query) {
  auto ballData = json::parse(query);

  float x = ballData["Ball"]["x"];
  float y = balldata["Ball"]["y"];
  float epsilon = ballData["Ball"]["epsilon"];

  Vertex* v = this->g->getClosestVertexInBall(x, y, epsilon);
  if (v != NULL) return v->getJson();

  return this->createError("EMPTY");
}

#endif
