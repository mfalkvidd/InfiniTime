#include "components/ble/AppleNotificationCenterClient.h"
#include "components/ble/NotificationManager.h"
#include "systemtask/SystemTask.h"
#include <algorithm>

using namespace Pinetime::Controllers;

int OnDiscoveryEventCallback(uint16_t conn_handle, const struct ble_gatt_error* error, const struct ble_gatt_svc* service, void* arg) {
  auto client = static_cast<AppleNotificationCenterClient*>(arg);
  return client->OnDiscoveryEvent(conn_handle, error, service);
}

int OnANCSCharacteristicDiscoveredCallback(uint16_t conn_handle,
                                           const struct ble_gatt_error* error,
                                           const struct ble_gatt_chr* chr,
                                           void* arg) {
  auto client = static_cast<AppleNotificationCenterClient*>(arg);
  return client->OnCharacteristicsDiscoveryEvent(conn_handle, error, chr);
}

int OnANCSDescriptorDiscoveryEventCallback(uint16_t conn_handle,
                                           const struct ble_gatt_error* error,
                                           uint16_t chr_val_handle,
                                           const struct ble_gatt_dsc* dsc,
                                           void* arg) {
  auto client = static_cast<AppleNotificationCenterClient*>(arg);
  return client->OnDescriptorDiscoveryEventCallback(conn_handle, error, chr_val_handle, dsc);
}

int NewAlertSubcribeCallback(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg) {
  auto client = static_cast<AppleNotificationCenterClient*>(arg);
  return client->OnNewAlertSubcribe(conn_handle, error, attr);
}

int OnControlPointWriteCallback(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg) {
  auto client = static_cast<AppleNotificationCenterClient*>(arg);
  return client->OnControlPointWrite(conn_handle, error, attr);
}

AppleNotificationCenterClient::AppleNotificationCenterClient(Pinetime::System::SystemTask& systemTask,
                                                             Pinetime::Controllers::NotificationManager& notificationManager)
  : systemTask {systemTask}, notificationManager {notificationManager} {
}

bool AppleNotificationCenterClient::OnDiscoveryEvent(uint16_t connectionHandle, const ble_gatt_error* error, const ble_gatt_svc* service) {
  if (service == nullptr && error->status == BLE_HS_EDONE) {
    if (isDiscovered) {
      NRF_LOG_INFO("ANCS Discovery found, starting characteristics discovery");
      ble_gattc_disc_all_chrs(connectionHandle, ancsStartHandle, ancsEndHandle, OnANCSCharacteristicDiscoveredCallback, this);
    } else {
      NRF_LOG_INFO("ANCS not found");
      MaybeFinishDiscovery(connectionHandle);
    }
    return true;
  }

  if (service != nullptr && ble_uuid_cmp(&ancsUuid.u, &service->uuid.u) == 0) {
    NRF_LOG_INFO("ANCS discovered : 0x%x - 0x%x", service->start_handle, service->end_handle);

    ancsStartHandle = service->start_handle;
    ancsEndHandle = service->end_handle;
    isDiscovered = true;
  }
  return false;
}

int AppleNotificationCenterClient::OnCharacteristicsDiscoveryEvent(uint16_t connectionHandle,
                                                                   const ble_gatt_error* error,
                                                                   const ble_gatt_chr* characteristic) {
  if (error->status != 0 && error->status != BLE_HS_EDONE) {
    NRF_LOG_INFO("ANCS Characteristic discovery ERROR");
    MaybeFinishDiscovery(connectionHandle);
    return 0;
  }

  if (characteristic == nullptr && error->status == BLE_HS_EDONE) {
    NRF_LOG_INFO("ANCS Characteristic discovery complete");
    if (lastCharacteristicValueHandle != 0) {
      AssignCharacteristicEndHandle(lastCharacteristicValueHandle, ancsEndHandle);
      lastCharacteristicValueHandle = 0;
    }
    if (isCharacteristicDiscovered) {
      StartNextDescriptorDiscovery(connectionHandle);
    } else {
      MaybeFinishDiscovery(connectionHandle);
    }
  } else if (characteristic != nullptr) {
    if (lastCharacteristicValueHandle != 0) {
      AssignCharacteristicEndHandle(lastCharacteristicValueHandle, characteristic->def_handle - 1);
    }

    if (ble_uuid_cmp(&notificationSourceChar.u, &characteristic->uuid.u) == 0) {
      NRF_LOG_INFO("ANCS Characteristic discovered: Notification Source");
      notificationSourceHandle = characteristic->val_handle;
      isCharacteristicDiscovered = true;
    } else if (ble_uuid_cmp(&controlPointChar.u, &characteristic->uuid.u) == 0) {
      NRF_LOG_INFO("ANCS Characteristic discovered: Control Point");
      controlPointHandle = characteristic->val_handle;
      isControlCharacteristicDiscovered = true;
    } else if (ble_uuid_cmp(&dataSourceChar.u, &characteristic->uuid.u) == 0) {
      NRF_LOG_INFO("ANCS Characteristic discovered: Data Source");
      dataSourceHandle = characteristic->val_handle;
      isDataCharacteristicDiscovered = true;
    }
    lastCharacteristicValueHandle = characteristic->val_handle;
  }
  return 0;
}

int AppleNotificationCenterClient::OnDescriptorDiscoveryEventCallback(uint16_t connectionHandle,
                                                                      const ble_gatt_error* error,
                                                                      uint16_t characteristicValueHandle,
                                                                      const ble_gatt_dsc* descriptor) {
  if (error->status == 0 && descriptor != nullptr) {
    if (ble_uuid_cmp(&clientCharacteristicConfigDescriptorUuid.u, &descriptor->uuid.u) != 0) {
      return 0;
    }

    if (characteristicValueHandle == notificationSourceHandle && notificationSourceDescriptorHandle == 0) {
      NRF_LOG_INFO("ANCS Notification Source CCCD discovered : %d", descriptor->handle);
      notificationSourceDescriptorHandle = descriptor->handle;
      isDescriptorFound = true;
    } else if (characteristicValueHandle == dataSourceHandle && dataSourceDescriptorHandle == 0) {
      NRF_LOG_INFO("ANCS Data Source CCCD discovered : %d", descriptor->handle);
      dataSourceDescriptorHandle = descriptor->handle;
      isDataDescriptorFound = true;
    }
    return 0;
  }

  if (error->status != BLE_HS_EDONE) {
    char errorStr[55];
    snprintf(errorStr, sizeof(errorStr), "ANCS Descriptor discovery ERROR: %d", error->status);
    NRF_LOG_INFO(errorStr);
  }

  if (characteristicValueHandle == notificationSourceHandle && notificationSourceDescriptorHandle != 0 && !isNotificationSourceSubscribed) {
    SubscribeToDescriptor(connectionHandle, notificationSourceDescriptorHandle);
    return 0;
  }

  if (characteristicValueHandle == dataSourceHandle && dataSourceDescriptorHandle != 0 && !isDataSourceSubscribed) {
    SubscribeToDescriptor(connectionHandle, dataSourceDescriptorHandle);
    return 0;
  }

  StartNextDescriptorDiscovery(connectionHandle);
  return 0;
}

int AppleNotificationCenterClient::OnNewAlertSubcribe(uint16_t connectionHandle,
                                                      const ble_gatt_error* error,
                                                      ble_gatt_attr* /*attribute*/) {
  if (error->status == 0) {
    if (pendingSubscriptionDescriptorHandle == notificationSourceDescriptorHandle) {
      NRF_LOG_INFO("ANCS Notification Source subscribe OK");
      isNotificationSourceSubscribed = true;
    } else if (pendingSubscriptionDescriptorHandle == dataSourceDescriptorHandle) {
      NRF_LOG_INFO("ANCS Data Source subscribe OK");
      isDataSourceSubscribed = true;
    } else {
      NRF_LOG_INFO("ANCS subscribe OK");
    }
  } else {
    NRF_LOG_INFO("ANCS New alert subscribe ERROR");
  }

  pendingSubscriptionDescriptorHandle = 0;
  subscriptionsDone = isNotificationSourceSubscribed && (!isDataCharacteristicDiscovered || isDataSourceSubscribed);
  StartNextDescriptorDiscovery(connectionHandle);

  return 0;
}

int AppleNotificationCenterClient::OnControlPointWrite(uint16_t /*connectionHandle*/,
                                                       const ble_gatt_error* error,
                                                       ble_gatt_attr* /*attribute*/) {
  if (error->status == 0) {
    NRF_LOG_INFO("ANCS Control Point write OK");
  } else {
    char errorStr[55];
    snprintf(errorStr, sizeof(errorStr), "ANCS Control Point ERROR: %d", error->status);
    NRF_LOG_INFO(errorStr);
  }
  return 0;
}

void AppleNotificationCenterClient::MaybeFinishDiscovery(uint16_t connectionHandle) {
  if (discoveryCompleteNotified) {
    return;
  }

  if (!isDiscovered || !isCharacteristicDiscovered) {
    discoveryCompleteNotified = true;
    onServiceDiscovered(connectionHandle);
    return;
  }

  subscriptionsDone = isNotificationSourceSubscribed && (!isDataCharacteristicDiscovered || isDataSourceSubscribed);
  const bool descriptorDiscoveryDone =
    isNotificationSourceDescriptorDiscoveryComplete && (!isDataCharacteristicDiscovered || isDataSourceDescriptorDiscoveryComplete);
  if (subscriptionsDone || descriptorDiscoveryDone) {
    discoveryCompleteNotified = true;
    onServiceDiscovered(connectionHandle);
  }
}

void AppleNotificationCenterClient::StartNextDescriptorDiscovery(uint16_t connectionHandle) {
  if (pendingSubscriptionDescriptorHandle != 0) {
    return;
  }

  if (isCharacteristicDiscovered && !isNotificationSourceDescriptorDiscoveryComplete) {
    isNotificationSourceDescriptorDiscoveryComplete = true;
    const uint16_t endHandle = notificationSourceEndHandle != 0 ? notificationSourceEndHandle : ancsEndHandle;
    int rc =
      ble_gattc_disc_all_dscs(connectionHandle, notificationSourceHandle, endHandle, OnANCSDescriptorDiscoveryEventCallback, this);
    if (rc == 0) {
      return;
    }
    NRF_LOG_INFO("ANCS Notification Source descriptor discovery start ERROR: %d", rc);
  }

  if (isDataCharacteristicDiscovered && !isDataSourceDescriptorDiscoveryComplete) {
    isDataSourceDescriptorDiscoveryComplete = true;
    const uint16_t endHandle = dataSourceEndHandle != 0 ? dataSourceEndHandle : ancsEndHandle;
    int rc = ble_gattc_disc_all_dscs(connectionHandle, dataSourceHandle, endHandle, OnANCSDescriptorDiscoveryEventCallback, this);
    if (rc == 0) {
      return;
    }
    NRF_LOG_INFO("ANCS Data Source descriptor discovery start ERROR: %d", rc);
  }

  MaybeFinishDiscovery(connectionHandle);
}

void AppleNotificationCenterClient::SubscribeToDescriptor(uint16_t connectionHandle, uint16_t descriptorHandle) {
  uint8_t value[2] {1, 0};
  pendingSubscriptionDescriptorHandle = descriptorHandle;
  int rc = ble_gattc_write_flat(connectionHandle, descriptorHandle, value, sizeof(value), NewAlertSubcribeCallback, this);
  if (rc != 0) {
    NRF_LOG_INFO("ANCS subscribe start ERROR: %d", rc);
    pendingSubscriptionDescriptorHandle = 0;
    StartNextDescriptorDiscovery(connectionHandle);
  }
}

void AppleNotificationCenterClient::AssignCharacteristicEndHandle(uint16_t characteristicValueHandle, uint16_t endHandle) {
  if (characteristicValueHandle == notificationSourceHandle) {
    notificationSourceEndHandle = endHandle;
  } else if (characteristicValueHandle == dataSourceHandle) {
    dataSourceEndHandle = endHandle;
  }
}

void AppleNotificationCenterClient::OnNotification(ble_gap_event* event) {
  if (event->notify_rx.attr_handle == notificationSourceHandle || event->notify_rx.attr_handle == notificationSourceDescriptorHandle) {
    NRF_LOG_INFO("ANCS Notification received");

    AncsNotification ancsNotif;

    os_mbuf_copydata(event->notify_rx.om, 0, 1, &ancsNotif.eventId);
    os_mbuf_copydata(event->notify_rx.om, 1, 1, &ancsNotif.eventFlags);
    os_mbuf_copydata(event->notify_rx.om, 2, 1, &ancsNotif.category);
    // Can be used to see how many grouped notifications are present
    // os_mbuf_copydata(event->notify_rx.om, 3, 1, &categoryCount);
    os_mbuf_copydata(event->notify_rx.om, 4, 4, &ancsNotif.uuid);

    // bool silent = (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::Silent)) != 0;
    //  bool important = eventFlags & static_cast<uint8_t>(EventFlags::Important);
    // bool preExisting = (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::PreExisting)) != 0;
    //  bool positiveAction = eventFlags & static_cast<uint8_t>(EventFlags::PositiveAction);
    //  bool negativeAction = eventFlags & static_cast<uint8_t>(EventFlags::NegativeAction);

    // If notification was removed, we remove it from the notifications map
    if (ancsNotif.eventId == static_cast<uint8_t>(EventIds::Removed) && notifications.contains(ancsNotif.uuid)) {
      notifications.erase(ancsNotif.uuid);
      NRF_LOG_INFO("ANCS Notification removed: %d", ancsNotif.uuid);
      return;
    }

    // If the notification is pre-existing, or if it is a silent notification, we do not add it to the list
    if (notifications.contains(ancsNotif.uuid) || (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::Silent)) != 0 ||
        (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::PreExisting)) != 0) {
      return;
    }

    // If new notification, add it to the notifications
    if (ancsNotif.eventId == static_cast<uint8_t>(EventIds::Added) &&
        (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::Silent)) == 0) {
      notifications.insert({ancsNotif.uuid, ancsNotif});
    } else {
      // If the notification is not added, we ignore it
      NRF_LOG_INFO("ANCS Notification not added, ignoring: %d", ancsNotif.uuid);
      return;
    }

    // The 6 is from TotalNbNotifications in NotificationManager.h + 1
    while (notifications.size() > 100) {
      notifications.erase(notifications.begin());
    }

    // if (notifications.contains(ancsNotif.uuid)) {
    //   notifications[ancsNotif.uuid] = ancsNotif;
    // } else {
    //   notifications.insert({ancsNotif.uuid, ancsNotif});
    // }

    // Request ANCS more info
    if (controlPointHandle == 0 || dataSourceHandle == 0) {
      return;
    }

    // The +4 is for the "..." at the end of the string
    uint8_t titleSize = maxTitleSize + 4;
    uint8_t subTitleSize = maxSubtitleSize + 4;
    uint8_t messageSize = maxMessageSize + 4;
    BYTE request[14];
    request[0] = 0x00; // Command ID: Get Notification Attributes
    request[1] = static_cast<uint8_t>(ancsNotif.uuid & 0xFF);
    request[2] = static_cast<uint8_t>((ancsNotif.uuid >> 8) & 0xFF);
    request[3] = static_cast<uint8_t>((ancsNotif.uuid >> 16) & 0xFF);
    request[4] = static_cast<uint8_t>((ancsNotif.uuid >> 24) & 0xFF);
    request[5] = 0x01; // Attribute ID: Title
    // request[6] = 0x00;
    request[6] = static_cast<uint8_t>(titleSize & 0xFF);
    request[7] = static_cast<uint8_t>((titleSize >> 8) & 0xFF);
    request[8] = 0x02; // Attribute ID: Subtitle
    request[9] = static_cast<uint8_t>(subTitleSize & 0xFF);
    request[10] = static_cast<uint8_t>((subTitleSize >> 8) & 0xFF);
    request[11] = 0x03; // Attribute ID: Message
    request[12] = static_cast<uint8_t>(messageSize & 0xFF);
    request[13] = static_cast<uint8_t>((messageSize >> 8) & 0xFF);

    ble_gattc_write_flat(event->notify_rx.conn_handle, controlPointHandle, request, sizeof(request), OnControlPointWriteCallback, this);
  } else if (event->notify_rx.attr_handle == dataSourceHandle || event->notify_rx.attr_handle == dataSourceDescriptorHandle) {
    uint16_t titleSize;
    uint16_t subTitleSize;
    uint16_t messageSize;
    uint32_t notificationUid;

    os_mbuf_copydata(event->notify_rx.om, 1, 4, &notificationUid);
    os_mbuf_copydata(event->notify_rx.om, 6, 2, &titleSize);
    os_mbuf_copydata(event->notify_rx.om, 8 + titleSize + 1, 2, &subTitleSize);
    os_mbuf_copydata(event->notify_rx.om, 8 + titleSize + 1 + 2 + subTitleSize + 1, 2, &messageSize);

    AncsNotification ancsNotif;
    ancsNotif.uuid = 0;

    // Check if the notification is in the session
    if (notifications.contains(notificationUid)) {
      if (notifications[notificationUid].isProcessed) {
        // If the notification is already processed, we ignore it
        NRF_LOG_INFO("Notification with UID %d already processed, ignoring", notificationUid);
        return;
      }
    } else {
      // If the notification is not in the session, we ignore it
      NRF_LOG_INFO("Notification with UID %d not found in session, ignoring", notificationUid);
      return;
    }

    if (notifications.contains(notificationUid)) {
      ancsNotif = notifications[notificationUid];
    } else {
      // If the Notification source didn't add it earlier, then don't process it
      NRF_LOG_INFO("Notification with UID %d not found in notifications map, ignoring datasource", notificationUid);
      return;
    }

    std::string decodedTitle = DecodeUtf8String(event->notify_rx.om, titleSize, 8);

    std::string decodedSubTitle = DecodeUtf8String(event->notify_rx.om, subTitleSize, 8 + titleSize + 1 + 2);

    std::string decodedMessage = DecodeUtf8String(event->notify_rx.om, messageSize, 8 + titleSize + 1 + 2 + subTitleSize + 1 + 2);

    // Debug event ids ands flags by putting them at front of message (in int format)
    // decodedMessage = std::to_string(ancsNotif.uuid) + " " + decodedMessage;

    NRF_LOG_INFO("Decoded Title: %s", decodedTitle.c_str());
    NRF_LOG_INFO("Decoded SubTitle: %s", decodedSubTitle.c_str());

    bool incomingCall = ancsNotif.uuid != 0 && ancsNotif.category == static_cast<uint8_t>(Categories::IncomingCall);

    if (!incomingCall) {
      if (titleSize >= maxTitleSize) {
        decodedTitle.resize(maxTitleSize - 3);
        decodedTitle += "...";
        if (!decodedSubTitle.empty()) {
          decodedTitle += " - ";
        } else {
          decodedTitle += "-";
        }
      } else {
        if (!decodedSubTitle.empty()) {
          decodedTitle += " - ";
        } else {
          decodedTitle += "-";
        }
      }

      if (subTitleSize > maxSubtitleSize) {
        decodedSubTitle.resize(maxSubtitleSize - 3);
        decodedSubTitle += "...";
      }
    }

    titleSize = static_cast<uint16_t>(decodedTitle.size());
    subTitleSize = static_cast<uint16_t>(decodedSubTitle.size());
    messageSize = static_cast<uint16_t>(decodedMessage.size());

    NotificationManager::Notification notif;
    notif.ancsUid = notificationUid;
    std::string notifStr;

    if (incomingCall) {
      notifStr += "Incoming Call:";
      notifStr += decodedTitle;
      notifStr += "\n";
      notifStr += decodedSubTitle;
    } else {
      notifStr += decodedTitle;
      if (!decodedSubTitle.empty()) {
        notifStr += decodedSubTitle + ":";
      }
      notifStr += decodedMessage;
    }

    // Adjust notification if too long
    if (notifStr.size() > NotificationManager::MessageSize) {
      notifStr.resize(97);
      notifStr += "...";
    }

    notif.message = std::array<char, NotificationManager::MessageSize + 1> {};
    std::strncpy(notif.message.data(), notifStr.c_str(), notif.message.size() - 1);

    if (incomingCall) {
      notif.message[13] = '\0'; // Separate Title and Message
    } else if (!decodedSubTitle.empty()) {
      notif.message[titleSize + subTitleSize] = '\0'; // Separate Title and Message
    } else {
      notif.message[titleSize - 1] = '\0'; // Separate Title and Message
    }

    notif.message[notif.message.size() - 1] = '\0'; // Ensure null-termination
    notif.size = std::min(std::strlen(notifStr.c_str()), notif.message.size());
    if (incomingCall) {
      notif.category = Pinetime::Controllers::NotificationManager::Categories::IncomingCall;
    } else {
      notif.category = Pinetime::Controllers::NotificationManager::Categories::SimpleAlert;
    }
    notificationManager.Push(std::move(notif));

    // Only ping the system task if the notification was added and ignore pre-existing notifications
    if (ancsNotif.isProcessed == false && (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::Silent)) == 0 &&
        (ancsNotif.eventFlags & static_cast<uint8_t>(EventFlags::PreExisting)) == 0) {
      systemTask.PushMessage(Pinetime::System::Messages::OnNewNotification);
    }

    // Mark the notification as processed in the session
    notifications[notificationUid].isProcessed = true;
  }
}

void AppleNotificationCenterClient::AcceptIncomingCall(uint32_t uuid) {
  if (notifications.contains(uuid)) {
    const AncsNotification ancsNotif = notifications[uuid];
    if (ancsNotif.category != static_cast<uint8_t>(Categories::IncomingCall)) {
      return;
    }
  } else {
    return;
  }

  uint8_t value[6];
  value[0] = 0x02; // Command ID: Perform Notification Action
  value[1] = static_cast<uint8_t>((uuid & 0xFF));
  value[2] = static_cast<uint8_t>((uuid >> 8) & 0xFF);
  value[3] = static_cast<uint8_t>((uuid >> 16) & 0xFF);
  value[4] = static_cast<uint8_t>((uuid >> 24) & 0xFF);
  value[5] = 0x00; // Action ID: Positive Action

  ble_gattc_write_flat(systemTask.nimble().connHandle(), controlPointHandle, value, sizeof(value), OnControlPointWriteCallback, this);
}

void AppleNotificationCenterClient::RejectIncomingCall(uint32_t uuid) {
  AncsNotification ancsNotif;
  if (notifications.contains(uuid)) {
    ancsNotif = notifications[uuid];
    if (ancsNotif.category != static_cast<uint8_t>(Categories::IncomingCall)) {
      return;
    }
  } else {
    return;
  }

  uint8_t value[6];
  value[0] = 0x02; // Command ID: Perform Notification Action
  value[1] = static_cast<uint8_t>(uuid & 0xFF);
  value[2] = static_cast<uint8_t>((uuid >> 8) & 0xFF);
  value[3] = static_cast<uint8_t>((uuid >> 16) & 0xFF);
  value[4] = static_cast<uint8_t>((uuid >> 24) & 0xFF);
  value[5] = 0x01; // Action ID: Negative Action

  ble_gattc_write_flat(systemTask.nimble().connHandle(), controlPointHandle, value, sizeof(value), OnControlPointWriteCallback, this);
}

void AppleNotificationCenterClient::Reset() {
  ancsStartHandle = 0;
  ancsEndHandle = 0;
  gattStartHandle = 0;
  gattEndHandle = 0;
  serviceChangedHandle = 0;
  serviceChangedDescriptorHandle = 0;
  notificationSourceHandle = 0;
  notificationSourceEndHandle = 0;
  notificationSourceDescriptorHandle = 0;
  controlPointHandle = 0;
  controlPointDescriptorHandle = 0;
  dataSourceHandle = 0;
  dataSourceEndHandle = 0;
  dataSourceDescriptorHandle = 0;
  pendingSubscriptionDescriptorHandle = 0;
  lastCharacteristicValueHandle = 0;
  isGattDiscovered = false;
  isGattCharacteristicDiscovered = false;
  isGattDescriptorFound = false;
  isDiscovered = false;
  isCharacteristicDiscovered = false;
  isDescriptorFound = false;
  isControlCharacteristicDiscovered = false;
  isControlDescriptorFound = false;
  isDataCharacteristicDiscovered = false;
  isDataDescriptorFound = false;
  subscriptionsDone = false;
  discoveryCompleteNotified = false;
  isNotificationSourceDescriptorDiscoveryComplete = false;
  isDataSourceDescriptorDiscoveryComplete = false;
  isNotificationSourceSubscribed = false;
  isDataSourceSubscribed = false;

  notifications.clear();
}

void AppleNotificationCenterClient::Discover(uint16_t connectionHandle, std::function<void(uint16_t)> onServiceDiscovered) {
  NRF_LOG_INFO("[ANCS] Starting discovery");
  this->onServiceDiscovered = onServiceDiscovered;
  ble_gattc_disc_svc_by_uuid(connectionHandle, &ancsUuid.u, OnDiscoveryEventCallback, this);
}

// This function is used for debugging purposes to log a message and push a notification
// Used to test BLE debugging on production devices
void AppleNotificationCenterClient::DebugNotification(const char* msg) const {
  NRF_LOG_INFO("[ANCS DEBUG] %s", msg);

  NotificationManager::Notification notif;
  std::strncpy(notif.message.data(), msg, notif.message.size() - 1);
  notif.message[notif.message.size() - 1] = '\0'; // Ensure null-termination
  notif.size = std::min(std::strlen(msg), notif.message.size());
  notif.category = Pinetime::Controllers::NotificationManager::Categories::SimpleAlert;
  notificationManager.Push(std::move(notif));

  systemTask.PushMessage(Pinetime::System::Messages::OnNewNotification);
}

namespace {
  static constexpr uint32_t faHeart = 0xF004;
  static constexpr uint32_t faStar = 0xF005;
  static constexpr uint32_t faFire = 0xF06D;
  static constexpr uint32_t faEyeSlash = 0xF070;
  static constexpr uint32_t faHandPointDown = 0xF0A7;
  static constexpr uint32_t faSmile = 0xF118;
  static constexpr uint32_t faFrown = 0xF119;
  static constexpr uint32_t faMeh = 0xF11A;
  static constexpr uint32_t faQuestion = 0xF128;
  static constexpr uint32_t faThumbsUp = 0xF164;
  static constexpr uint32_t faHandPeace = 0xF25B;
  static constexpr uint32_t faDumbbell = 0xF44B;
  static constexpr uint32_t faHands = 0xF4C2;
  static constexpr uint32_t faSmileWink = 0xF4DA;
  static constexpr uint32_t faGlasses = 0xF530;
  static constexpr uint32_t faGrin = 0xF580;
  static constexpr uint32_t faGrinBeamSweat = 0xF583;
  static constexpr uint32_t faGrinHearts = 0xF584;
  static constexpr uint32_t faGrinSquintTears = 0xF586;
  static constexpr uint32_t faGrinStars = 0xF587;
  static constexpr uint32_t faGrinTears = 0xF588;
  static constexpr uint32_t faGrinTongue = 0xF589;
  static constexpr uint32_t faKissWinkHeart = 0xF598;
  static constexpr uint32_t faLaughBeam = 0xF59A;
  static constexpr uint32_t faLaughSquint = 0xF59B;
  static constexpr uint32_t faMehRollingEyes = 0xF5A5;
  static constexpr uint32_t faSadCry = 0xF5B3;
  static constexpr uint32_t faSadTear = 0xF5B4;
  static constexpr uint32_t faSmileBeam = 0xF5B8;
  static constexpr uint32_t faSpa = 0xF5BB;
  static constexpr uint32_t faSurprise = 0xF5C2;
  static constexpr uint32_t faTired = 0xF5C8;
  static constexpr uint32_t faPrayingHands = 0xF684;
  static constexpr uint32_t faGlassCheers = 0xF79F;
  static constexpr uint32_t faHeartBroken = 0xF7A9;

  bool IsInFontDefinition(uint32_t codepoint) {
    // Check if the codepoint falls into the specified font ranges or is explicitly listed.
    return (codepoint >= 0x20 && codepoint <= 0x7E) ||    // Printable ASCII
           codepoint == 0xC4 ||                           // A with diaeresis
           codepoint == 0xC5 ||                           // A with ring
           codepoint == 0xC9 ||                           // E with acute
           codepoint == 0xD6 ||                           // O with diaeresis
           codepoint == 0xE4 ||                           // a with diaeresis
           codepoint == 0xE5 ||                           // a with ring
           codepoint == 0xE9 ||                           // e with acute
           codepoint == 0xF6 ||                           // o with diaeresis
           (codepoint >= 0x410 && codepoint <= 0x44F) ||  // Cyrillic
           codepoint == 0xB0 ||
           codepoint == 0xFFFD;
  }

  bool IsEmojiVariantCodepoint(uint32_t codepoint) {
    return codepoint == 0x200D ||                         // Zero width joiner
           codepoint == 0x2640 ||                         // Female sign
           codepoint == 0x2642 ||                         // Male sign
           codepoint == 0xFE0E ||                         // Text presentation selector
           codepoint == 0xFE0F ||                         // Emoji presentation selector
           (codepoint >= 0x1F3FB && codepoint <= 0x1F3FF); // Emoji skin tone modifiers
  }

  void AppendUtf8(std::string& output, uint32_t codepoint) {
    if (codepoint <= 0x7F) {
      output.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FF) {
      output.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else if (codepoint <= 0xFFFF) {
      output.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else {
      output.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
      output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
      output.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
  }

  const char* EmojiToText(uint32_t codepoint) {
    switch (codepoint) {
      case 0x203C:  // Double exclamation mark
        return "!!";
      case 0x1F4AF: // Hundred points
        return "100";
      default:
        return nullptr;
    }
  }

  uint32_t EmojiToFontAwesome(uint32_t codepoint) {
    switch (codepoint) {
      case 0x1F602: // Face with tears of joy
        return faGrinTears;
      case 0x1F923: // Rolling on the floor laughing
        return faGrinSquintTears;
      case 0x1F60D: // Smiling face with heart-eyes
        return faGrinHearts;
      case 0x1F60A: // Smiling face with smiling eyes
      case 0x1F60C: // Relieved face
      case 0x1F607: // Smiling face with halo
      case 0x1F642: // Slightly smiling face
      case 0x263A:  // Smiling face
        return faSmileBeam;
      case 0x1F62D: // Loudly crying face
        return faSadCry;
      case 0x1F618: // Face blowing a kiss
        return faKissWinkHeart;
      case 0x1F605: // Grinning face with sweat
        return faGrinBeamSweat;
      case 0x1F601: // Beaming face with smiling eyes
      case 0x1F600: // Grinning face
      case 0x1F603: // Grinning face with big eyes
        return faGrin;
      case 0x1F622: // Crying face
        return faSadTear;
      case 0x1F914: // Thinking face
      case 0x1F937: // Shrug
        return faQuestion;
      case 0x1F606: // Grinning squinting face
        return faLaughSquint;
      case 0x1F644: // Face with rolling eyes
        return faMehRollingEyes;
      case 0x1F609: // Winking face
        return faSmileWink;
      case 0x1F917: // Hugging face
        return faHands;
      case 0x1F614: // Pensive face
        return faFrown;
      case 0x1F60E: // Smiling face with sunglasses
        return faGlasses;
      case 0x1F926: // Face palm
        return faTired;
      case 0x1F631: // Face screaming in fear
        return faSurprise;
      case 0x1F60B: // Face savoring food
        return faGrinTongue;
      case 0x1F60F: // Smirking face
      case 0x1F612: // Unamused face
        return faMeh;
      case 0x1F929: // Star-struck
        return faGrinStars;
      case 0x1F604: // Grinning face with smiling eyes
        return faLaughBeam;
      case 0x1F92D: // Face with hand over mouth
        return faSmile;
      case 0x1F64F: // Folded hands
        return faPrayingHands;
      case 0x1F44D: // Thumbs up
        return faThumbsUp;
      case 0x1F44F: // Clapping hands
      case 0x1F64C: // Raising hands
        return faHands;
      case 0x1F4AA: // Flexed biceps
        return faDumbbell;
      case 0x1F44C: // OK hand
        return 0xF00C; // Check
      case 0x270C: // Victory hand
        return faHandPeace;
      case 0x1F447: // Backhand index pointing down
        return faHandPointDown;
      case 0x2764:  // Heavy black heart
      case 0x2665:  // Black heart suit
      case 0x2763:  // Heavy heart exclamation mark ornament
      case 0x1F493: // Beating heart
      case 0x1F495: // Two hearts
      case 0x1F496: // Sparkling heart
      case 0x1F497: // Growing heart
      case 0x1F499: // Blue heart
      case 0x1F49A: // Green heart
      case 0x1F49B: // Yellow heart
      case 0x1F49C: // Purple heart
      case 0x1F49E: // Revolving hearts
      case 0x1F5A4: // Black heart
        return faHeart;
      case 0x1F494: // Broken heart
        return faHeartBroken;
      case 0x1F525: // Fire
        return faFire;
      case 0x1F339: // Rose
      case 0x1F338: // Cherry blossom
        return faSpa;
      case 0x1F389: // Party popper
        return faGlassCheers;
      case 0x2728: // Sparkles
        return faStar;
      case 0x1F648: // See-no-evil monkey
        return faEyeSlash;
      case 0x1F3B6: // Multiple musical notes
        return 0xF001; // Music
      default:
        return 0;
    }
  }

  bool IsNonBreakingSpace(uint32_t codepoint) {
    return codepoint == 0x00A0 || codepoint == 0x202F || codepoint == 0x2007;
  }

  void AppendDecodedCodepoint(std::string& decoded, const std::string& utf8Char, uint32_t codepoint) {
    if (IsEmojiVariantCodepoint(codepoint)) {
      return;
    }

    if (const char* text = EmojiToText(codepoint); text != nullptr) {
      decoded.append(text);
      return;
    }

    if (IsNonBreakingSpace(codepoint)) {
      // Convert non-breaking space to normal space
      decoded.append(" ");
      return;
    }

    const uint32_t fontAwesomeCodepoint = EmojiToFontAwesome(codepoint);
    if (fontAwesomeCodepoint != 0) {
      AppendUtf8(decoded, fontAwesomeCodepoint);
      return;
    }

    if (IsInFontDefinition(codepoint)) {
      decoded.append(utf8Char);
    } else {
      decoded.append("�"); // Replace unsupported
    }
  }
}

std::string AppleNotificationCenterClient::DecodeUtf8String(os_mbuf* om, uint16_t size, uint16_t offset) {
  std::string decoded;
  decoded.reserve(size);

  for (uint16_t i = 0; i < size;) {
    uint8_t byte = 0;
    if (os_mbuf_copydata(om, offset + i, 1, &byte) != 0) {
      break; // Handle error in copying data (e.g., log or terminate processing)
    }

    if (byte <= 0x7F) { // Single-byte UTF-8 (ASCII)
      std::string utf8Char;
      utf8Char.push_back(static_cast<char>(byte));
      AppendDecodedCodepoint(decoded, utf8Char, byte);
      ++i;
    } else { // Multi-byte UTF-8
      // Determine sequence length based on leading byte
      int sequenceLength = 0;
      if ((byte & 0xE0) == 0xC0) {
        sequenceLength = 2; // 2-byte sequence
      } else if ((byte & 0xF0) == 0xE0) {
        sequenceLength = 3; // 3-byte sequence
      } else if ((byte & 0xF8) == 0xF0) {
        sequenceLength = 4; // 4-byte sequence
      }

      if (sequenceLength == 0) {
        decoded.append("�"); // Invalid leading byte, replace
        ++i;
        continue;
      }

      if (i + sequenceLength > size) {
        decoded.append("�"); // Incomplete sequence, replace
        break;
      }

      // Read the full sequence
      std::string utf8Char;
      bool validSequence = true;
      uint32_t codepoint = 0;

      for (int j = 0; j < sequenceLength; ++j) {
        uint8_t nextByte = 0;
        os_mbuf_copydata(om, offset + i + j, 1, &nextByte);
        utf8Char.push_back(static_cast<char>(nextByte));

        if (j == 0) {
          // Leading byte contributes significant bits
          if (sequenceLength == 2) {
            codepoint = nextByte & 0x1F;
          } else if (sequenceLength == 3) {
            codepoint = nextByte & 0x0F;
          } else if (sequenceLength == 4) {
            codepoint = nextByte & 0x07;
          }
        } else {
          // Continuation bytes contribute lower bits
          if ((nextByte & 0xC0) != 0x80) {
            validSequence = false; // Invalid UTF-8 continuation byte
            break;
          }
          codepoint = (codepoint << 6) | (nextByte & 0x3F);
        }
      }

      if (validSequence) {
        AppendDecodedCodepoint(decoded, utf8Char, codepoint);
      } else {
        decoded.append("�"); // Replace unsupported
      }
      i += sequenceLength;
    }
  }

  return decoded;
}
