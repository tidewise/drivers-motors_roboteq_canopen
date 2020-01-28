#include <iostream>
#include <fstream>
#include <motors_roboteq_canopen/SerialCommandWriter.hpp>

using namespace std;
using namespace motors_roboteq_canopen;

void usage(ostream& io) {
    io << "motors_roboteq_canopen_cfg URI PATH [--reset]\n"
       << "send configuration commands contained by the file at PATH to the\n"
       << "Roboteq controller reachable at URI, through the serial interface\n"
       << "\n"
       << "If --reset is given, reset the controller after applying the\n"
       << "configuration\n"
       << flush;
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        usage(cerr);
        exit(1);
    }
    else if (argc == 4 && string(argv[3]) != "--reset") {
        cerr << "--reset must be given last\n\n" << endl;
        usage(cerr);
        exit(1);
    }

    string uri = argv[1];
    string path = argv[2];
    bool reset = (argc == 4);

    ifstream file(path);
    if (!file) {
        cerr << path << " does not exist" << endl;
        exit(1);
    }

    SerialCommandWriter writer;
    writer.setLogStream(cout);
    writer.openURI(uri);
    writer.executeCommands(file);

    if (reset) {
        writer.sendCommand("%RESET 321654987");
    }
}
