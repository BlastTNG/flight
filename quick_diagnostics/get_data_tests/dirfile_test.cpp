#include <unistd.h>
#include <stdio.h>
#include <getdata/dirfile.h>
#include "string.h"

// TODO: test the QString::fromNumber func, see if it gives something other than 0
// TODO: could try to debug in qt creator, just need to see error messages, if any!

using namespace std;

int main(int argc, char** argv) {
  GetData::Dirfile* dirfile = new GetData::Dirfile("/Users/matthewriley/Documents/BLAST/sample.dirfile");
  if (dirfile->Error() != GD_E_OK) {
    printf("Error: %s\n", dirfile->ErrorString());
    return 0;
  }
  
  for (int j = 0; j < 100; ++j) {
    printf("------------------------------------------\n\n");
    for (int i = 10; i < 50; ++i) {
      string fieldStr("roach2_kid00");
      fieldStr = fieldStr + to_string(i) + "_i";

      double buffer[1];
      int numFrames = dirfile->NFrames();
      const char* field = fieldStr.c_str();
      int numSamplesRead = dirfile->GetData(field, numFrames - 1, 0, 0, 1, GetData::Float64, (void*) buffer); 

      printf("Num samples read: %d", numSamplesRead);
      printf("roach2_kid%04d_i: %f\n", i, buffer[0]);

      dirfile->Flush(field);

      if (dirfile->Error() != GD_E_OK) {
        printf("Error: %s\n", dirfile->ErrorString());
      }
    }
    usleep(1000 * 1000 * 10);
  }
}
