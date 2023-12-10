#ifndef __SERIALPORT_H
#define __SERIALPORT_H

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "BaseType.h"

class SerialPort {
 public:
  enum class Event {
    SERIAL_PORT_ERROR,
    SERIAL_PORT_TX_EMPTY,
    SERIAL_PORT_RX_FULL,
    SERIAL_PORT_RX_VALID
  };

  SerialPort(std::function<void(Event)> callback);
  ~SerialPort();

  bool isOpen();
  int open();
  int close();
  int write(const u8* data, u32 data_size, u32 timeout);
  int read(u8* buffer, u32 buffer_size, u32 timeout);

  u32 getBaudrate(void);
  int setBaudrate(u32 baudrate);
  const wchar_t* getPort(void);
  bool setPort(const wchar_t* portName);

  int setDTR(bool state);
  int setRTS(bool state);

  bool setEventCallback(std::function<void(Event)> callback);

  int purgeBuffer(void);
  size_t getReadBytes(void);
  size_t getWriteBytes(void);
  static bool getSerialList(std::vector<wchar_t*>& serial_list);
  static long getTickCount(void);

 private:
  std::function<void(Event)> m_callback;
  std::string m_port = "";
  uint32_t m_baudrate = 115200;
  std::deque<uint8_t> read_buffer;
};

#endif
