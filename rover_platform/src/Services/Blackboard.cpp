#ifndef _INCL_BLACKBOARD
#define _INCL_BLACKBOARD

#include <vector>
#include <string>
#include <iostream>
#include "../Graph/Graph.cpp"
#include "../Graph/Vertex.cpp"
#include "../utilities/json.hpp"

using namespace std;
using json = nlohmann::json;

class Blackboard {
private:
  Graph* g;
  Vertex* currentLocation;

  string createError(string errorMsg);

  void addEdge(string query);
  void setCurrentLocation(string query);
  void setCurrentLabel(string query);
  void saveDataToCurrentLocation(string query);

  string addVertex(string query);
  string queryCurrentLocation();
  string getClosestVertex(string query);
  string getClosestVertexInBall(string query);
  string getConnectedVertices(string query);
  string getVertexByIndex(string query);
  string getVertexByLabel(string query);
  string getSize();
  string getCurrentLabel();
  string findPath(string query);
  string getDataFromCurrentLocation();

public:
  Blackboard();
  string performQuery(string query);
};

Blackboard::Blackboard() {
  this->g = new Graph();
  this->currentLocation = new Vertex(0,0,0);
}

string Blackboard::createError(string errorMsg) {
  json response = {
    {"Error", errorMsg}
  };

  return response.dump();
}

string Blackboard::queryCurrentLocation() {
  cout << "queryCurrentLocation" << endl;
  if (this->currentLocation == NULL) return this->createError("There is no current vertex!");

  json v = this->currentLocation->getJson();
  cout << v.dump() << endl;

  return this->currentLocation->getJson();
}

string Blackboard::addVertex(string query) {
  auto vertexData = json::parse(query);

  float x = vertexData["Vertex"]["x"];
  float y = vertexData["Vertex"]["y"];
  int index = this->g->getSize();

  Vertex* v = new Vertex(x,y,index);
  this->g->addVertex(v);

  return v->getJson();
}

void Blackboard::setCurrentLocation(string query) {
  auto locationData = json::parse(query);

  int index = locationData["Location"]["index"];
  Vertex* v = this->g->getVertexByIndex(index);
  this->currentLocation = v;
}

void Blackboard::addEdge(string query) {
  auto edgeData = json::parse(query);

  int v1 = edgeData["Edge"]["v1"];
  int v2 = edgeData["Edge"]["v2"];

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
  float y = ballData["Ball"]["y"];
  float epsilon = ballData["Ball"]["epsilon"];

  Vertex* v = this->g->getClosestVertexInBall(x, y, epsilon);
  if (v != NULL) return v->getJson();

  return this->createError("EMPTY");
}

string Blackboard::getConnectedVertices(string query) {
  auto vertexData = json::parse(query);

  int index = vertexData["Vertex"]["index"];

  vector<Vertex*> result = this->g->getConnectedVertices(index);

  json response = json::array();
  for(int i = 0; i < result.size(); i++) {
    json obj = result[i]->getJson();
    response.insert(response.end(), obj);
  }

  return response.dump();
}

string Blackboard::getVertexByIndex(string query) {
  auto vertexData = json::parse(query);

  int index = vertexData["Vertex"]["index"];
  Vertex* result = this->g->getVertexByIndex(index);

  if (result == NULL) return this->createError("EMPTY");

  return result->getJson();
}

string Blackboard::getVertexByLabel(string query) {
  auto vertexData = json::parse(query);

  string label = vertexData["Vertex"]["label"];
  Vertex* result = this->g->getVertexByLabel(label);

  if (result == NULL) return this->createError("EMPTY");

  return result->getJson();
}

string Blackboard::getSize() {
  json response = {
    "size", this->g->getSize()
  };

  return response.dump();
}

void Blackboard::setCurrentLabel(string query) {
  cout << "setCurrentLabel" << endl;

  auto vertexLabel = json::parse(query);
  if (this->currentLocation == NULL) return;
  this->currentLocation->setLabel(vertexLabel["Vertex"]["label"]);
}

string Blackboard::findPath(string query) {
  auto pathQuery = json::parse(query);

  float x = pathQuery["Vertex"]["x"];
  float y = pathQuery["Vertex"]["y"];
  int index = pathQuery["Vertex"]["index"];

  Vertex* goal = new Vertex(x, y, index);

  vector<Vertex*> result = this->g->findPath(this->currentLocation, goal);

  cout << "Generating Path JSON" << endl;

  json response = json::array();
  for(int i = 0; i < result.size(); i++) {
    json obj = result[i]->getJson();
    response.insert(response.end(), obj);
  }

  return response.dump();

}

void Blackboard::saveDataToCurrentLocation(string query) {
  if (this->currentLocation == NULL) return;
  this->currentLocation->data = query;
}

string Blackboard::getDataFromCurrentLocation() {
  if (this->currentLocation == NULL) {
    return this->createError("EMPTY");
  }

  return this->currentLocation->data;
}

string Blackboard::performQuery(string query) {
  auto queryData = json::parse(query);

  string command = queryData["Command"];

  cout << "Command: " << command << endl;

  if (command == "addVertex") return this->addVertex(query);
  if (command == "addEdge") this->addEdge(query);
  if (command == "setCurrentLocation") this->setCurrentLocation(query);
  if (command == "queryCurrentLocation") return this->queryCurrentLocation();
  if (command == "getClosestVertex") return this->getClosestVertex(query);
  if (command == "getClosestVertexInBall") return this->getClosestVertexInBall(query);
  if (command == "getConnectedVertices") return this->getConnectedVertices(query);
  if (command == "getVertexByIndex") return this->getVertexByIndex(query);
  if (command == "getVertexByLabel") return this->getVertexByLabel(query);
  if (command == "getSize") return this->getSize();
  if (command == "setCurrentLabel") this->setCurrentLabel(query);
  if (command == "findPath") return this->findPath(query);
  if (command == "saveDataToCurrentLocation") this->saveDataToCurrentLocation(query);
  if (command == "getDataFromCurrentLocation") return this->getDataFromCurrentLocation();


  return "OK";

}

#endif
