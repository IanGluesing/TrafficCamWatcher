#include <iostream>
#include <fstream>
#include <string>
#include <thread>

#include "camera_server.h"

int main() {

    camera_server c;

    c.start();
    
    return 0;
}
