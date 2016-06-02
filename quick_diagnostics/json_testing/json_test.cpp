#include "json.hpp"
#include <string>
#include <stdio.h>
using json = nlohmann::json;

int main(int argc, char** argv) {
  json j = json::parse("{\"name\": \"sample name\",\"foo\": {\"bar\": \"bar_val\"}}");
  printf("num children %d\n", (int) j.size());
  printf("name: %s\n", j["name"].get<std::string>().c_str());

  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    json element = *it;
    if (element.is_primitive()) {
      printf("Primitive element\n");
    } else {
      printf("Not primitive element\n");
    }
  }
}
