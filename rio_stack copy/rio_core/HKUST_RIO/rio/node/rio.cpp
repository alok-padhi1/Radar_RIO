#include "zmqWarper.hpp"

#ifdef DEBUG
#define BACKWARD_HAS_BFD 1
#define BACKWARD_HAS_DW 1
#include <backward.hpp>
backward::SignalHandling sh;
#endif

int main(int argc, char **argv) {
  std::shared_ptr<RIO> rio = std::make_shared<RIO>();
  
  // Keep the main thread alive while ZMQ loop runs in background
  while (true) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}