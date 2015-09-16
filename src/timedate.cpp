#include <iostream>
#include <ctime>
#include <timedate.h>
using namespace std;

void displayTime(){
    // current date/time based on current system
    time_t now = time(0);

    // convert now to string form
    char* dt = ctime(&now);

    cout << dt << endl;

}
