#ifndef _INCL_VERTEX
#define _INCL_VERTEX

#include <string>
#include "../utilities/json.hpp"

using namespace std;
using json = nlohmann::json;

class Vertex {
private:
  float x;
  float y;
  int index;
  int parent; // used for book keeping during search
  bool explored;
  string label;

public:

  string data;

  Vertex();
  Vertex(float x, float y, int index);
  Vertex(float x, float y, int index, int parent, string label);

  int getIndex();
  float getX();
  float getY();
  bool inNhd(float x, float y, float epsilon);
  bool isExplored();
  void markExplored();
  void setParent(int parent);
  int getParent();
  void setLabel(string label);
  string getLabel();
  string getJson();

  static Vertex* createVertexFromJson(json data);
};

Vertex::Vertex() {
  this->x = 0;
  this->y = 0;
  this->index = -1;
  this->explored = false;
}

Vertex::Vertex(float x, float y, int index) {
  this->x = x;
  this->y = y;
  this->index = index;
  this->explored = false;
}

Vertex::Vertex(float x, float y, int index, int parent, string label) {
  this->x = x;
  this->y = y;
  this->index = index;
  this->parent = parent;
  this->label = label;
  this->explored = false;
}

int Vertex::getIndex() {
  return this->index;
}

float Vertex::getX() {
  return this->x;
}

float Vertex::getY() {
  return this->y;
}

bool Vertex::inNhd(float x, float y, float epsilon) {
  float distance = sqrt( pow(x - this->x, 2) + pow(y - this->y, 2) );
  return distance < epsilon;
}

bool Vertex::isExplored() {
  return this->explored;
}

void Vertex::markExplored() {
  this->explored = true;
}

void Vertex::setParent(int parent) {
  this->parent = parent;
}

int Vertex::getParent() {
  return this->parent;
}

void Vertex::setLabel(string label) {
  this->label = label;
}

string Vertex::getLabel() {
  return this->label;
}

string Vertex::getJson() {
  json response = {
    {"Vertex", {
      {"x", this->x},
      {"y", this->y},
      {"index", this->index},
      {"parent", this->parent},
      {"label", this->label}
    }}
  };

  return response.dump();
}

Vertex* Vertex::createVertexFromJson(json data) {
  float x = data["Vertex"]["x"];
  float y = data["Vertex"]["y"];
  int index = data["Vertex"]["index"];
  int parent = data["Vertex"]["parent"];
  string label = data["Vertex"]["label"];


  return new Vertex(x, y, index, parent, label);
}


#endif
