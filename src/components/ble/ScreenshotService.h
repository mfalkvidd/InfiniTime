#pragma once

#include <cstdint>

#define min // workaround: nimble's min/max macros conflict with libstdc++
#define max
#include <host/ble_gap.h>
#include <host/ble_uuid.h>
#undef max
#undef min

namespace Pinetime {
  namespace System {
    class SystemTask;
  }

  namespace Controllers {
    class ScreenshotService {
    public:
      explicit ScreenshotService(Pinetime::System::SystemTask& systemTask);

      void Init();
      int OnServiceData(uint16_t connectionHandle, uint16_t attributeHandle, ble_gatt_access_ctxt* context);
      void OnScreenshotResult(const char* path, bool success);

    private:
      Pinetime::System::SystemTask& systemTask;

      struct ble_gatt_chr_def characteristicDefinition[3];
      struct ble_gatt_svc_def serviceDefinition[2];

      uint16_t triggerCharacteristicHandle {};
      uint16_t statusCharacteristicHandle {};
      uint16_t connectionHandle = BLE_HS_CONN_HANDLE_NONE;

      bool busy = false;
      char status[80] = "idle";

      bool IsConnectionSecure(uint16_t connectionHandle) const;
      void SetStatus(const char* newStatus);
      void NotifyStatus();
    };
  }
}
