#ifndef SRC_PRINTEVENTHANDLER_H
#define SRC_PRINTEVENTHANDLER_H

//#include <folly/io/async/EventBase.h>
#include <atomic>
#include <future>
#include <memory>

class PrintEventHandler {
public:
  using Ptr = std::shared_ptr<PrintEventHandler>;

  PrintEventHandler();

  void Print(const std::string &text);

  void Print(const std::string &author, const std::string &text);

  void Stop();

  virtual ~PrintEventHandler();

private:
//  folly::EventBase base_;
  std::atomic_bool is_running_{};
  std::shared_future<void> future_;

};


#endif //SRC_PRINTEVENTHANDLER_H
