#include "SerialPort.h"

#include <chrono>
#include <cstdio>

using namespace std;

SerialPort::SerialPort() {}

SerialPort::~SerialPort(void) {
  printf("%s", "destructing");
  thread_destroy();
  if (isOpen()) {
    if (close()) {
      printf("%ws release failed.", m_port);
    }
  }
  printf("%s", "destructed");
}

bool SerialPort::isOpen() {
#if defined(Q_OS_WIN)
  return (m_hCom != nullptr) && (m_hCom != INVALID_HANDLE_VALUE);
#elif defined(Q_OS_LINUX)

#elif defined(Q_OS_MACOS)

#endif
}

int SerialPort::open() {
  printf("open %ls", m_port);
  if (isOpen()) {
    printf("%ls has been opened", m_port);
    return 0;
  }
  int ret = ERROR_SUCCESS;
#ifdef _WIN32
  char port[64];
  if (snprintf(port, sizeof(port), "\\\\.\\%ls", m_port) < 0) {
    return -1;
  }
  m_hCom = CreateFile(port, GENERIC_READ | GENERIC_WRITE,
                      0,              //  must be opened with exclusive-access
                      0,              //  default security attributes
                      OPEN_EXISTING,  //  must use OPEN_EXISTING
                      FILE_FLAG_OVERLAPPED,  // overlapped I/O
                      0);  //  hTemplate must be nullptr for comm devices

  if (m_hCom == INVALID_HANDLE_VALUE || m_hCom == nullptr) {
    ret = GetLastError();
    printf("CreateFile failed with error %d.", GetLastError());
    goto end;
  }
  // Set buffer size
  if (!SetupComm(m_hCom, 1024 * 2, 1024 * 2)) {
    ret = GetLastError();
    printf("SetupComm failed with error %d.", ret);
    goto end;
  }
  // Set serial communication parameter
  DCB dcb;
  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(m_hCom, &dcb)) {
    ret = GetLastError();
    printf("GetCommState failed with error %d.", ret);
    goto end;
  }

  dcb.BaudRate = m_baudrate;  //  baud rate
  dcb.ByteSize = 8;           //  data size, xmit and rcv
  dcb.Parity = NOPARITY;      //  parity bit
  dcb.StopBits = ONESTOPBIT;  //  stop bit
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  if (!SetCommState(m_hCom, &dcb)) {
    ret = GetLastError();
    printf("SetCommState failed with error %d.", ret);
    goto end;
  }
  // Set event
  // if (!SetCommMask(m_hCom, EV_BREAK | EV_CTS | EV_DSR | EV_ERR | EV_RING |
  //                              EV_RLSD | EV_RXCHAR | EV_RXFLAG | EV_TXEMPTY))
  //                              {
  //   ret = GetLastError();
  //   printf("SetCommMask failed with error %d.", ret);
  //   goto end;
  // }

  // Set timeout
  COMMTIMEOUTS timeouts;
  memset(&timeouts, 0, sizeof(timeouts));
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
  timeouts.ReadTotalTimeoutConstant = 100;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  if (!SetCommTimeouts(m_hCom, &timeouts)) {
    ret = GetLastError();
    printf("SetCommTimeouts failed with error %d.", ret);
    goto end;
  }
  if (m_hThreadC == nullptr || m_hThreadC == INVALID_HANDLE_VALUE) {
    m_hThreadC = CreateEvent(nullptr, true, false, nullptr);
    if (m_hThreadC == nullptr || m_hThreadC == INVALID_HANDLE_VALUE) {
      ret = GetLastError();
      printf("CreateEvent failed with error %d.", ret);
      goto end;
    }
  }
  if (ov_write.hEvent == nullptr) {
    CloseHandle(ov_write.hEvent);
    memset(&ov_write, 0, sizeof(ov_write));
    ov_write.hEvent = CreateEvent(0, true, 0, 0);
  }
  if (ov_read.hEvent == nullptr) {
    CloseHandle(ov_read.hEvent);
    memset(&ov_read, 0, sizeof(ov_read));
    ov_read.hEvent = CreateEvent(0, true, 0, 0);
  }
#elif __linux__

#elif __APPLE__

#endif
end:
  if (ret == ERROR_SUCCESS) {
    if (m_thread == nullptr) {
      thread_create();
    } else {
      thread_resume();
    }
    printf("Serial port %ls init finished.", m_port);
  }
  return ret;
}

int SerialPort::close() {
  int ret = ERROR_SUCCESS;
  if (!isOpen()) {
    printf("%ls hasn't been opened", m_port);
    return 0;
  }
  thread_suspend();
  if (!PurgeComm(m_hCom, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR |
                             PURGE_RXABORT)) {
    printf("%s failed:%d", "PurgeComm", GetLastError());
  }
  if (!CancelIo(m_hCom)) {
    printf("%s failed:%d", "CancelIo", GetLastError());
  }
  read_buffer.clear();
  // if (ov_write.hEvent != nullptr) {
  //   if (CloseHandle(ov_write.hEvent)) {
  //     ov_write.hEvent = nullptr;
  //   } else {
  //     printf("%s close failed:%d", "ov_write", GetLastError());
  //   }
  // }
  // if (ov_read.hEvent != nullptr) {
  //   if (CloseHandle(ov_read.hEvent)) {
  //     ov_read.hEvent = nullptr;
  //   } else {
  //     printf("%s close failed:%d", "ov_read", GetLastError());
  //   }
  // }
  if (m_hCom != nullptr && m_hCom != INVALID_HANDLE_VALUE) {
    if (!CloseHandle(m_hCom)) {
      ret = GetLastError();
      printf("%ls close failed:%d", m_port, ret);
      goto end;
    } else {
      m_hCom = nullptr;
    }
  } else {
    printf("%ls has closed ", m_port);
    return ERROR_SUCCESS;
  }
end:
  if (ret == ERROR_SUCCESS) {
    printf("%ls closed", m_port);
  } else {
    printf("%ls close failed", m_port);
  }
  return ret;
}

int SerialPort::write(const uint8_t* data, uint32_t bytes2write,
                      uint32_t timeout) {
  DWORD written_size, ret;
  if (WriteFile(m_hCom, data, bytes2write, nullptr, &ov_write)) {
    goto get_result;
  } else {
    ret = GetLastError();
    if (ret == ERROR_IO_PENDING) {
      ret = WaitForSingleObject(ov_write.hEvent, timeout);
      switch (ret) {
        case WAIT_OBJECT_0:
          goto get_result;
        case WAIT_FAILED:
        case WAIT_TIMEOUT:
          goto error;
      }
    } else {
      goto error;
    }
  }
get_result:
  ResetEvent(ov_write.hEvent);
  if (GetOverlappedResult(m_hCom, &ov_write, &written_size, true)) {
    return written_size;
  } else {
    goto error;
  }
error:
  ResetEvent(ov_write.hEvent);
  ret = GetLastError();
  printf("write error:%d", ret);
  emitEvent(Event::SERIAL_PORT_ERROR);
  return -1;
}

int SerialPort::read(uint8_t* buffer, uint32_t bytes2read, uint32_t timeout) {
  auto end = chrono::steady_clock::now() + chrono::milliseconds(timeout);
  uint32_t index = 0;
  while (index < bytes2read) {
    if (chrono::steady_clock::now() >= end) {
      goto end;
    }
    if (!read_buffer.empty()) {
      buffer[index] = read_buffer.front();
      read_buffer.pop_front();
      index++;
    }
  }
end:
  return index;
}

uint32_t SerialPort::getBaudrate(void) { return m_baudrate; }

int SerialPort::setBaudrate(uint32_t baudrate) {
  int ret = ERROR_SUCCESS;
  m_baudrate = baudrate;
  if (isOpen()) {
    DCB dcb;
    if (!GetCommState(m_hCom, &dcb)) {
      ret = GetLastError();
      printf("GetCommState failed with error %d.", ret);
      goto end;
    }
    dcb.BaudRate = m_baudrate;
    if (!SetCommState(m_hCom, &dcb)) {
      ret = GetLastError();
      printf("SetCommState failed with error %d.", ret);
      goto end;
    }
  }
end:
  if (ret == ERROR_SUCCESS) {
    printf("%ls baudrete change to %d", m_port, m_baudrate);
  }
  return ret;
}

const string SerialPort::getPort(void) { return m_port; }

bool SerialPort::setPort(const string portName) {
  int ret = ERROR_SUCCESS;
  if (isOpen()) {
    ret = close();
  }
  if (ret != ERROR_SUCCESS) {
    return false;
  }
  m_port = portName;
  return true;
}

int SerialPort::setDTR(bool state) {
  int ret = ERROR_SUCCESS;
  if (!EscapeCommFunction(m_hCom, state ? SETDTR : CLRDTR)) {
    ret = GetLastError();
    printf("EscapeCommFunction failed with error %d.", ret);
  }
  if (ret == ERROR_SUCCESS) {
    printf("set DTR %s", state ? "1" : "0");
  }
  return ret;
}

int SerialPort::setRTS(bool state) {
  int ret = ERROR_SUCCESS;
  if (!EscapeCommFunction(m_hCom, state ? SETRTS : CLRRTS)) {
    ret = GetLastError();
    printf("EscapeCommFunction failed with error %d.", ret);
  }
  if (ret == ERROR_SUCCESS) {
    printf("set RTS %s", state ? "1" : "0");
  }
  return ret;
}

bool SerialPort::setEventCallback(function<void(Event)> callback) {
  m_callback = callback;
  return true;
}

int SerialPort::purgeBuffer() {
  int ret = ERROR_SUCCESS;
  if (!PurgeComm(m_hCom, PURGE_RXCLEAR)) {
    ret = GetLastError();
    printf("PurgeComm failed with error %d.", ret);
  }
  read_buffer.clear();
  return ret;
}

size_t SerialPort::getReadBytes(void) { return read_buffer.size(); }

vector<string> SerialPort::getSerialList(void) {
  vector<string> serial_list;
  // TODO optimise memory usage
  HKEY hkey;
  int result, i = 0;
  if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Hardware\\DeviceMap\\SerialComm"),
                   0, KEY_READ, &hkey)) {
    goto exit;
  }
  char portName[128];
  char commName[32];
  DWORD dwLong, dwSize;
  do {
    dwSize = sizeof(portName) / sizeof(char);
    dwLong = dwSize;
    result = RegEnumValue(hkey, i, portName, &dwLong, nullptr, nullptr,
                          (LPBYTE)commName, &dwSize);
    if (ERROR_SUCCESS != result) {
      if (ERROR_NO_MORE_ITEMS == result) {
        break;
      } else {
        goto exit;
      }
    }

    serial_list.push_back(string(commName));
    i++;
  } while (1);
exit:
  return serial_list;
}

void SerialPort::read_thread(void) {
  // Init overlapped
  uint8_t buffer[1024];
  uint32_t error_line;
  DWORD ret, read_size;
  HANDLE event_list[2];
  event_list[0] = ov_read.hEvent;
  event_list[1] = m_hThreadC;
  while (1) {
    unique_lock<mutex> locker(m_mutex);
    while (m_suspend) {
      printf("%s", "thread suspended");
      ResetEvent(m_hThreadC);
      m_cond.wait(locker);
    }
    if (m_thread_stop) {
      goto exit;
    }
    if (!ReadFile(m_hCom, buffer, sizeof(buffer), nullptr, &ov_read)) {
      ret = GetLastError();
      if (ret != ERROR_IO_PENDING) {
        error_line = __LINE__;
        goto error;
      }
    }
    ret = WaitForMultipleObjects(2, event_list, false, INFINITE);
    switch (ret) {
      case WAIT_OBJECT_0:
        ResetEvent(ov_read.hEvent);
        GetOverlappedResult(m_hCom, &ov_read, &read_size, true);
        for (uint32_t i = 0; i < read_size; i++) {
          // LOG_V("%02x ", buffer[i]);
          read_buffer.push_back(buffer[i]);
        }
        if (read_buffer.size() != 0) {
          emitEvent(Event::SERIAL_PORT_RX_VALID);
        }
        break;
      case WAIT_OBJECT_0 + 1:
        ResetEvent(m_hThreadC);
        break;
      default:
        if (ret == WAIT_FAILED) {
          error_line = __LINE__;
          goto error;
        } else {
          printf("WaitForMultipleObjects ret:%d", ret);
        }
        break;
    }
    continue;
  error:
    ret = GetLastError();
    printf("read error[Line%d]:%d", error_line, ret);
    emitEvent(Event::SERIAL_PORT_ERROR);
    close();
  }
exit:
  if (!CloseHandle(m_hThreadC)) {
    printf("%s close failed:%d", "m_hThreadC", GetLastError());
  } else {
    m_hThreadC = nullptr;
  }
  printf("%s", "read thread exited.");
}

void SerialPort::thread_create(void) {
  if (m_thread != nullptr) {
    thread_destroy();
  }
  m_thread = new thread(std::bind(SerialPort::read_thread, this), nullptr);
}

void SerialPort::thread_destroy(void) {
  if (m_thread != nullptr) {
    printf("%s", "request destroying thread.");
    m_thread_stop = true;
    thread_resume();
    m_thread->join();
    m_thread_stop = false;
    if (m_thread != nullptr) {
      delete m_thread;
      m_thread = nullptr;
    }
  }
}

void SerialPort::thread_suspend(void) {
  if (m_thread != nullptr) {
    printf("%s", "m_suspend thread.");
    // unique_lock<mutex> locker(m_mutex);
    SetEvent(m_hThreadC);
    m_suspend = true;
  }
}

void SerialPort::thread_resume(void) {
  if (m_thread != nullptr) {
    printf("%s", "resume thread.");
    unique_lock<mutex> locker(m_mutex);
    m_suspend = false;
    m_cond.notify_one();
  }
}

void SerialPort::emitEvent(Event event) {
  if (event != Event::SERIAL_PORT_RX_VALID) {
    printf("event %d", event);
  }
  m_callback(event);
}
