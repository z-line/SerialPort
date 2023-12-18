#include "SerialPort.h"

#include <cstdio>

#include "Logger.h"

using namespace std;

SerialPort::SerialPort(std::function<void(uint8_t*, size_t)> callback)
    : m_callback(callback) {}

SerialPort::~SerialPort(void) {
  thread_destroy();
  if (isOpen()) {
    if (close()) {
      LOG_E() << m_port << " release failed.";
    }
  }
}

bool SerialPort::isOpen() {
#if defined(_WIN32)
  return (m_hCom != nullptr) && (m_hCom != INVALID_HANDLE_VALUE);
#elif defined(__linux__)

#elif defined(__APPLE__)

#endif
}

int SerialPort::open() {
  if (isOpen()) {
    LOG_D() << m_port << " has been opened";
    return 0;
  }
  int ret = ERROR_SUCCESS;
#ifdef _WIN32
  char port[64];
  if (snprintf(port, sizeof(port), "\\\\.\\%s", m_port.c_str()) < 0) {
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
    LOG_E() << "CreateFile failed with error " << ret;
    goto end;
  }
  // Set buffer size
  if (!SetupComm(m_hCom, 1024 * 2, 1024 * 2)) {
    ret = GetLastError();
    LOG_E() << "SetupComm failed with error " << ret;
    goto end;
  }
  // Set serial communication parameter
  DCB dcb;
  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(m_hCom, &dcb)) {
    ret = GetLastError();
    LOG_E() << "GetCommState failed with error " << ret;
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
    LOG_E() << "SetCommState failed with error " << ret;
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
    LOG_E() << "SetCommTimeouts failed with error " << ret;
    goto end;
  }
  if (m_hThreadC == nullptr || m_hThreadC == INVALID_HANDLE_VALUE) {
    m_hThreadC = CreateEvent(nullptr, true, false, nullptr);
    if (m_hThreadC == nullptr || m_hThreadC == INVALID_HANDLE_VALUE) {
      ret = GetLastError();
      LOG_E() << "CreateEvent failed with error " << ret;
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
    LOG_E() << "Serial port " << m_port << " init finished.";
  }
  return ret;
}

int SerialPort::close() {
  int ret = ERROR_SUCCESS;
  if (!isOpen()) {
    LOG_W() << m_port << "hasn't been opened";
    return 0;
  }
  thread_suspend();
  if (!PurgeComm(m_hCom, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR |
                             PURGE_RXABORT)) {
    LOG_W() << "PurgeComm failed " << GetLastError();
  }
  if (!CancelIo(m_hCom)) {
    LOG_W() << "CancelIo failed " << GetLastError();
  }
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
      LOG_E() << m_port << "close failed: " << ret;
      goto end;
    } else {
      m_hCom = nullptr;
    }
  } else {
    LOG_D() << m_port << " has closed";
    return ERROR_SUCCESS;
  }
end:
  if (ret == ERROR_SUCCESS) {
    LOG_D() << m_port << " closed";
  } else {
    LOG_E() << m_port << " close failed";
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
  LOG_E() << "write error: " << ret;
  return -1;
}

uint32_t SerialPort::getBaudrate(void) { return m_baudrate; }

int SerialPort::setBaudrate(uint32_t baudrate) {
  int ret = ERROR_SUCCESS;
  m_baudrate = baudrate;
  if (isOpen()) {
    DCB dcb;
    if (!GetCommState(m_hCom, &dcb)) {
      ret = GetLastError();
      LOG_E() << "GetCommState failed with error " << ret;
      goto end;
    }
    dcb.BaudRate = m_baudrate;
    if (!SetCommState(m_hCom, &dcb)) {
      ret = GetLastError();
      LOG_E() << "SetCommState failed with error " << ret;
      goto end;
    }
  }
end:
  if (ret == ERROR_SUCCESS) {
    LOG_D() << m_port << " baudrate change to " << baudrate;
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
    LOG_E() << "EscapeCommFunction failed with error " << ret;
  }
  if (ret == ERROR_SUCCESS) {
    LOG_V() << "set DTR " << (state ? "1" : "0");
  }
  return ret;
}

int SerialPort::setRTS(bool state) {
  int ret = ERROR_SUCCESS;
  if (!EscapeCommFunction(m_hCom, state ? SETRTS : CLRRTS)) {
    ret = GetLastError();
    LOG_E() << "EscapeCommFunction failed with error " << ret;
  }
  if (ret == ERROR_SUCCESS) {
    LOG_V() << "set RTS " << (state ? "1" : "0");
  }
  return ret;
}

bool SerialPort::setEventCallback(function<void(uint8_t*, size_t)> callback) {
  m_callback = callback;
  return true;
}

int SerialPort::purgeBuffer() {
  int ret = ERROR_SUCCESS;
  if (!PurgeComm(m_hCom, PURGE_RXCLEAR)) {
    ret = GetLastError();
    LOG_E() << "PurgeComm failed with error " << ret;
  }
  return ret;
}

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
      LOG_D() << "thread suspended";
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
        if (read_size > 0) {
          m_callback(buffer, read_size);
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
          LOG_W() << "WaitForMultipleObjects ret: " << ret;
        }
        break;
    }
    continue;
  error:
    ret = GetLastError();
    LOG_E() << "read error[Line" << error_line << "]: " << ret;
    close();
  }
exit:
  if (!CloseHandle(m_hThreadC)) {
    LOG_W() << "m_hThreadC close failed: " << GetLastError();
  } else {
    m_hThreadC = nullptr;
  }
  LOG_D() << "read thread exited.";
}

void SerialPort::thread_create(void) {
  if (m_thread != nullptr) {
    thread_destroy();
  }
  m_thread_stop = false;
  m_thread = new thread(std::bind(&SerialPort::read_thread, this), nullptr);
}

void SerialPort::thread_destroy(void) {
  if (m_thread != nullptr) {
    LOG_D() << "request destroying thread.";
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
    LOG_D() << "m_suspend thread.";
    // unique_lock<mutex> locker(m_mutex);
    SetEvent(m_hThreadC);
    m_suspend = true;
  }
}

void SerialPort::thread_resume(void) {
  if (m_thread != nullptr) {
    LOG_D() << "resume thread.";
    unique_lock<mutex> locker(m_mutex);
    m_suspend = false;
    m_cond.notify_one();
  }
}
