#ifndef _INCL_SUBSUMPTION
#define _INCL_SUBSUMPTION

#include <algorithm>
#include <vector>
#include <string>
#include "Behaviors/Behaviour.cpp"

using namespace std;

// from stack overflow: https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
vector<string> split(const string &text, char sep) {
  vector<string> tokens;
  size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

class SubsumptionManager {
private:
  vector<Behavior*> behaviors;

  vector<string> pathStringToVector(string path);
  Behavior* findBehaviorByPathVector(vector<string> path);
  Behavior* getRootBehaviorByName(string name);
  Behavior* findRootActiveBehavior(vector<string> path);
public:
  SubsumptionManager();
  void registerBehavior(string parentPath, Behavior* child);
  void activateBehavior(string path);
  void deActivateBehavior(string path);
};

SubsumptionManager::SubsumptionManager() {}

vector<string> SubsumptionManager::pathStringToVector(string path) {
  return split(path, '/');
}

Behavior* SubsumptionManager::getRootBehaviorByName(string name) {
  for(int index = 0; index < this->behaviors.size(); index++) {
      if(this->behaviors[index]->getName() == name) {
        return this->behaviors[index];
      }
  }
  return NULL;
}

Behavior* SubsumptionManager::findBehaviorByPathVector(vector<string> path) {

  if (path.size() == 0) return NULL;

  reverse(path.begin(), path.end());
  Behavior* result = this->getRootBehaviorByName(path.back());
  if (result == NULL) return NULL;

  path.pop_back();

  while(path.size() > 0) {
    result = result->getChildByName(path.back());
    if (result == NULL) return NULL;
    path.pop_back();
  }

  return result;
}

void SubsumptionManager::registerBehavior(string parentPath, Behavior* child) {
  vector<string> pathVector = this->pathStringToVector(parentPath);
  Behavior* parent = this->findBehaviorByPathVector(pathVector);

  if (parent == NULL) {
    // register as a top level behaviors
    this->behaviors.push_back(child);
    return;
  }

  parent->registerChildBehavior(child);
}

void SubsumptionManager::activateBehavior(string path) {

  vector<string> pathVector = this->pathStringToVector(path);
  Behavior* b = this->findBehaviorByPathVector(pathVector);

  b->turnOn();

  for(int i = 0; i < b->children.size(); i++) {
    b->children[i]->deactivate();
  }

  b->activate();

}

Behavior* SubsumptionManager::findRootActiveBehavior(vector<string> path) {

  while(path.size() > 0) {
    Behavior* b = this->findBehaviorByPathVector(path);
    if (b->active()) return b;
    path.pop_back();
  }

  return NULL;
}

void SubsumptionManager::deActivateBehavior(string path) {

  vector<string> pathVector = this->pathStringToVector(path);
  Behavior* b = this->findBehaviorByPathVector(pathVector);

  b->turnOff();

  pathVector.pop_back();
  Behavior* top = this->findRootActiveBehavior(pathVector);

  if (top != NULL) {
    return;
  }

  b->deactivate();

  for(int i = 0; i < b->children.size(); i++) {
    if (b->children[i]->active()) {
      b->children[i]->activate();
    }
  }

}

#endif
