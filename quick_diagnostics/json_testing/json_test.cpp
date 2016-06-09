#include "json.hpp"
#include <string>
#include <stdio.h>
using json = nlohmann::json;

int main(int argc, char** argv) {
  json j = {
    {"x", 1},
    {"y", 2},
    {"z", 3},
    {"parent", {"child", 4}}
  };
  printf("num children %d\n", (int) j.size());

  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    json element = *it;
    printf("key: %s", it.key().c_str());    
  }
}
