#include "util.hpp"
#include <signal.h>

using namespace std;

string stringf (string format, ...) {
    char buf[2048];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 2048, format.c_str(), args);
    va_end(args);
    return std::string(buf);
}

bool ends_with(const string& text, const string& ending) {
    if (text.length() >= ending.length()) {
        return (0 == text.compare (text.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

string replace(string text, const string& from , const string& to) {
    size_t pos = 0;
    while ((pos = text.find(from, pos)) != string::npos) {
         text.replace(pos, from.length(), to);
         pos += to.length();
    }
    return text;
}

const double infinity = numeric_limits<double>::max();

void segfault() {
     raise(SIGSEGV);
}

void ERROR(const string& msg) {
    cerr << "ERROR: " << msg << endl;
    exit(1);
}