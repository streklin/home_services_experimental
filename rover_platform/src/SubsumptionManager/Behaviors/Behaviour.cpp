#ifndef _INCL_BEHAVIOR
#define _INCL_BEHAVIOR

#include <vector>
#include <string>

using namespace std;

class Behavior {
protected:
  int id;
  string name;
  bool isActive;
public:
  vector<Behavior*> children;

  Behavior();
  Behavior(int id, string name);
  Behavior* getChildByName(string name);
  void registerChildBehavior(Behavior* child);
  string getName();
  int getId();
  bool active();
  void turnOn();
  void turnOff();
  virtual void activate();
  virtual void deactivate();
};

Behavior::Behavior() {}

Behavior::Behavior(int id, string name) {
  this->id = id;
  this->name = name;
  this->isActive = false;
}

string Behavior::getName() {
  return this->name;
}

Behavior* Behavior::getChildByName(string name) {
  for(int index = 0; index < this->children.size(); index++) {
    if (this->children[index]->getName() == name) {
      return this->children[index];
    }
  }

  return NULL;
}

void Behavior::registerChildBehavior(Behavior* child) {
  this->children.push_back(child);
}

void Behavior::turnOn() {
  this->isActive = true;
}

void Behavior::turnOff() {
  this->isActive = false;
}

int Behavior::getId() {
  return this->id;
}

bool Behavior::active() {
  return this->isActive;
}

void Behavior::activate() {}

void Behavior::deactivate() {}

#endif
