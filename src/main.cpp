#include <iostream>
#include "test.hpp"

using namespace std;
extern const char* git_version_short;

int main() {
    cout << "SimEdit " << git_version_short << endl;   

    gravity_test();

    return 0;
}