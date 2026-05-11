#include "components/ble/ScreenshotService.h"
#include <cstring>
#include <cstdio>
#include <nrf_log.h>
#include "systemtask/SystemTask.h"

using namespace Pinetime::Controllers;

namespace {
  // 0006yyxx-78fc-48fe-8e23-433b3a1942d0
  constexpr ble_uuid128_t CharUuid(uint8_t x, uint8_t y) {
    return ble_uuid128_t {.u = {.type = BLE_UUID_TYPE_128},
                          .value = {0xd0, 0x42, 0x19, 0x3a, 0x3b, 0x43, 0x23, 0x8e, 0xfe, 0x48, 0xfc, 0x78, x, y, 0x06, 0x00}};
  }

  // 00060000-78fc-48fe-8e23-433b3a1942d0
  constexpr ble_uuid128_t BaseUuid() {
    return CharUuid(0x00, 0x00);
  }

  constexpr ble_uuid128_t screenshotServiceUuid {BaseUuid()};
  constexpr ble_uuid128_t screenshotTriggerCharUuid {CharUuid(0x01, 0x00)};
  constexpr ble_uuid128_t screenshotStatusCharUuid {CharUuid(0x02, 0x00)};

  int ScreenshotServiceCallback(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    auto* service = static_cast<ScreenshotService*>(arg);
    return service->OnServiceData(conn_handle, attr_handle, ctxt);
  }
}

ScreenshotService::ScreenshotService(Pinetime::System::SystemTask& systemTask)
  : systemTask {systemTask},
    characteristicDefinition {{.uuid = &screenshotTriggerCharUuid.u,
                               .access_cb = ScreenshotServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
                               .val_handle = &triggerCharacteristicHandle},
                              {.uuid = &screenshotStatusCharUuid.u,
                               .access_cb = ScreenshotServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &statusCharacteristicHandle},
                              {0}},
    serviceDefinition {
      {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &screenshotServiceUuid.u, .characteristics = characteristicDefinition},
      {0},
    } {
}

void ScreenshotService::Init() {
  int res = 0;
  res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);

  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);
}

int ScreenshotService::OnServiceData(uint16_t connectionHandle, uint16_t attributeHandle, ble_gatt_access_ctxt* context) {
  if (attributeHandle == statusCharacteristicHandle) {
    if (!IsConnectionSecure(connectionHandle)) {
      return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    int res = os_mbuf_append(context->om, status, strlen(status));
    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (attributeHandle != triggerCharacteristicHandle || context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return 0;
  }

  if (!IsConnectionSecure(connectionHandle)) {
    NRF_LOG_INFO("[Screenshot] rejected request from unbonded connection");
    return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
  }

  this->connectionHandle = connectionHandle;
  if (busy) {
    SetStatus("busy");
    NotifyStatus();
    return 0;
  }

  busy = true;
  SetStatus("busy");
  NotifyStatus();
  systemTask.PushMessage(Pinetime::System::Messages::ScreenshotRequested);

  return 0;
}

void ScreenshotService::OnScreenshotResult(const char* path, bool success) {
  if (success) {
    snprintf(status, sizeof(status), "ok:%s", path);
  } else {
    SetStatus("err");
  }

  busy = false;
  NotifyStatus();
  systemTask.PushMessage(Pinetime::System::Messages::EnableSleeping);
}

bool ScreenshotService::IsConnectionSecure(uint16_t connectionHandle) const {
  struct ble_gap_conn_desc desc {};
  int res = ble_gap_conn_find(connectionHandle, &desc);
  if (res != 0) {
    return false;
  }

  return desc.sec_state.encrypted && desc.sec_state.bonded;
}

void ScreenshotService::SetStatus(const char* newStatus) {
  strncpy(status, newStatus, sizeof(status) - 1);
  status[sizeof(status) - 1] = '\0';
}

void ScreenshotService::NotifyStatus() {
  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  auto* om = ble_hs_mbuf_from_flat(status, strlen(status));
  if (om == nullptr) {
    return;
  }

  ble_gattc_notify_custom(connectionHandle, statusCharacteristicHandle, om);
}
