#include <iostream>

using namespace std;
extern const char* git_version_short;

int main() {
    cout << "SimEdit " << git_version_short << endl;   

    return 0;
}