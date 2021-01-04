#include <iostream>
#include <chrono>
#include <omp.h>
#include <vector>
using namespace std;

using namespace std::chrono;

void Hello(void);

int main() {
  std::vector<int> vec(1000000000);

  auto start = high_resolution_clock::now();
  //#pragma omp parallel for
  for(int i = 0 ; i< vec.size(); ++i){
    vec[i] = 4;
  }

  auto stop = high_resolution_clock::now();

  auto duration = duration_cast<microseconds>(stop - start);

  cout << "Time taken by function: "
       << (double)duration.count()/1e6 << " seconds" << endl;

  return 0;
}
