#include "Communicator.h"
#include "Log.h"

using namespace argosServer;

int main(int argc, char **argv) {
  if(argc < 3) {
    std::cout << "Use: " << argv[0] << " <port> <iface>" << std::endl;
    return 1;
  }

  Log::setColouredOutput(isatty(fileno(stdout)));

  Communicator com(atoi(argv[1]), argv[2]);
  com.run();

  return 0;
}
