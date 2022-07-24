/*----------------------------------------------------------------------------/
  Lovyan HAL library - Hardware Abstraction Layer library .

Original Source:
 https://github.com/lovyan03/LovyanHAL/

Licence:
 [BSD](https://github.com/lovyan03/LovyanHAL/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)
/----------------------------------------------------------------------------*/
#pragma once

#include "init.hpp"
#include "../../use_with_host/for_host.hpp"

#include <WinSock2.h>

#ifndef LHAL_DEFAULT_CONNECTION_NAME
#define LHAL_DEFAULT_CONNECTION_NAME nullptr
#endif

namespace lhal
{
  class LHAL : public LHAL_Host
  {
    class TransportCom : public internal::ITransportLayer
    {
      HANDLE _com = INVALID_HANDLE_VALUE;
      char _target[16];
    public:

      error_t init(const char* target);
      error_t connect(void) override;
      void disconnect(void) override;

      int read(void) override;
      int write(const uint8_t* data, size_t len) override;
    };

    class TransportSock : public internal::ITransportLayer
    {
      SOCKET _sock = 0;
    public:

      error_t init(const char* target);
      int read(void) override;
      int write(const uint8_t* data, size_t len) override;
    };

    TransportCom _tl_com;
    TransportSock _tl_sock;

  public:

    LHAL(const char* target = LHAL_DEFAULT_CONNECTION_NAME);
  };
}
