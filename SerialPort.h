#ifndef __SERIALPORT_H
#define __SERIALPORT_H

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include "windows.h"
#elif __linux__

#elif __APPLE__

#else
#error "Unsupported platform"
#endif

class SerialPort {
 public:
  enum class Event {
    SERIAL_PORT_ERROR,
    SERIAL_PORT_TX_EMPTY,
    SERIAL_PORT_RX_FULL,
    SERIAL_PORT_RX_VALID
  };

  SerialPort();
  ~SerialPort();

  bool isOpen();
  int open();
  int close();
  int write(const uint8_t* data, uint32_t data_size, uint32_t timeout);
  int read(uint8_t* buffer, uint32_t buffer_size, uint32_t timeout);
  uint32_t getBaudrate(void);
  int setBaudrate(uint32_t baudrate);
  const std::string getPort(void);
  bool setPort(const std::string portName);
  int setDTR(bool state);
  int setRTS(bool state);
  bool setEventCallback(std::function<void(Event)> callback);
  int purgeBuffer(void);
  size_t getReadBytes(void);
  static std::vector<std::string> getSerialList(void);

 private:
  std::function<void(Event)> m_callback;
  std::string m_port = "";
  uint32_t m_baudrate = 115200;
  std::deque<uint8_t> read_buffer;
  std::thread* m_thread = nullptr;

  bool m_suspend = false;
  std::mutex m_mutex;
  std::condition_variable m_cond;
  void read_thread(void);
  bool m_thread_stop = true;
  void thread_create(void);
  void thread_destroy(void);
  void thread_suspend(void);
  void thread_resume(void);

  void emitEvent(Event event);

#ifdef _WIN32
  HANDLE m_hCom = NULL, m_hThreadC = NULL;
  OVERLAPPED ov_write = {0, 0, 0, 0, 0}, ov_read = {0, 0, 0, 0, 0};
#elif __linux__

#elif __APPLE__

#endif
};

#endif
