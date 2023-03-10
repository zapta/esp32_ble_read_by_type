#include "host.h"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <algorithm>
#include <string.h>

namespace host {

static constexpr auto TAG = "app_host";

// clang-format off
#define ENCODE_UUID_128(w32, w1, w2, w3, w48) \
    (((w48) >>  0) & 0xFF), \
    (((w48) >>  8) & 0xFF), \
    (((w48) >> 16) & 0xFF), \
    (((w48) >> 24) & 0xFF), \
    (((w48) >> 32) & 0xFF), \
    (((w48) >> 40) & 0xFF), \
    (((w3)  >>  0) & 0xFF), \
    (((w3)  >>  8) & 0xFF), \
    (((w2)  >>  0) & 0xFF), \
    (((w2)  >>  8) & 0xFF), \
    (((w1)  >>  0) & 0xFF), \
    (((w1)  >>  8) & 0xFF), \
    (((w32) >>  0) & 0xFF), \
    (((w32) >>  8) & 0xFF), \
    (((w32) >> 16) & 0xFF), \
    (((w32) >> 24) & 0xFF)
// clang-format on

// clang-format off
#define ENCODE_UUID_16(w16)  \
    (((w16) >>  0) & 0xFF), \
    (((w16) >>  8) & 0xFF)
// clang-format on

static uint8_t service_uuid[] = {
    ENCODE_UUID_128(0x6b6a78d7, 0x8ee0, 0x4a26, 0xba7b, 0x62e357dd9720)};
static uint8_t command_uuid[] = {ENCODE_UUID_16(0xff06)};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = nullptr,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = false,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = const_cast<uint8_t*>(service_uuid),
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 150,  // x 0.65ms
    .adv_int_max = 300,  // x 0.65ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#pragma GCC diagnostic pop

static bool adv_data_configured = false;
static bool scan_rsp_configured = false;

static uint8_t kPrimaryServiceDeclUuid[] = {
    ENCODE_UUID_16(ESP_GATT_UUID_PRI_SERVICE)};

static uint8_t kCharDeclUuid[] = {ENCODE_UUID_16(ESP_GATT_UUID_CHAR_DECLARE)};
static uint8_t kChrPropertyWriteNrOnly = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static uint8_t command_val[8] = {};

enum {
  ATTR_IDX_SVC,

  ATTR_IDX_COMMAND,
  ATTR_IDX_COMMAND_VAL,

  ATTR_IDX_COUNT,  // Count.
};

// The main BLE attribute table.
static const esp_gatts_attr_db_t attr_table[ATTR_IDX_COUNT] = {

    [ATTR_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
        {sizeof(kPrimaryServiceDeclUuid), kPrimaryServiceDeclUuid,
            ESP_GATT_PERM_READ, sizeof(service_uuid), sizeof(service_uuid),
            service_uuid}},

    // ----- Command.
    //
    // Characteristic
    [ATTR_IDX_COMMAND] = {{ESP_GATT_AUTO_RSP},
        {sizeof(kCharDeclUuid), kCharDeclUuid, ESP_GATT_PERM_READ,
            sizeof(kChrPropertyWriteNrOnly), sizeof(kChrPropertyWriteNrOnly),
            &kChrPropertyWriteNrOnly}},

    // Value
    [ATTR_IDX_COMMAND_VAL] = {{ESP_GATT_AUTO_RSP},
        {sizeof(command_uuid), command_uuid, ESP_GATT_PERM_WRITE,
            sizeof(command_val), sizeof(command_val), command_val}},

};

// static uint16_t handle_table[ATTR_IDX_COUNT];

static bool connected = false;

static void gap_event_handler(
    esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    // Called once during setup to indicate that adv data is configured.
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      ESP_LOGI(TAG, "Adv data configured");
      adv_data_configured = true;
      if (scan_rsp_configured) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;

    // Called once during setup to indicate that adv scan rsp is configured.
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      ESP_LOGI(TAG, "Scan rsp configured");
      scan_rsp_configured = true;
      if (scan_rsp_configured) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      // advertising start complete event to indicate advertising start
      // successfully or failed
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "advertising start failed");
      } else {
        ESP_LOGI(TAG, "advertising start successfully");
      }
      break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Advertising stop failed");
      } else {
        ESP_LOGI(TAG, "Stop adv successfully");
      }
      break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG,
          "update connection params status = %d, min_int = %d, max_int = "
          "%d,conn_int = %d,latency = %d, timeout = %d",
          param->update_conn_params.status, param->update_conn_params.min_int,
          param->update_conn_params.max_int, param->update_conn_params.conn_int,
          param->update_conn_params.latency, param->update_conn_params.timeout);
      break;

    default:
      ESP_LOGI(TAG,
          "Gap event handler: unknown event esp_gap_ble_cb_event_t = %d",
          event);
      break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
  switch (event) {
    case ESP_GATTS_REG_EVT: {
      ESP_LOGI(TAG, "ESP_GATTS_REG_EVT event");
      assert(param->reg.status == ESP_GATT_OK);
      const char* device_name = "esp32-test";
      ESP_LOGI(TAG, "Device name: %s", device_name);
      esp_err_t err = esp_ble_gap_set_device_name(device_name);
      if (err) {
        ESP_LOGE(TAG, "set device name failed, error code = %x", err);
      }

      adv_data_configured = false;
      scan_rsp_configured = false;
      err = esp_ble_gap_config_adv_data(&adv_data);
      if (err) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", err);
      }
      err = esp_ble_gap_config_adv_data(&scan_rsp_data);
      if (err) {
        ESP_LOGE(TAG, "config scan response data failed, error code = %x", err);
      }

      err = esp_ble_gatts_create_attr_tab(
          attr_table, gatts_if, ATTR_IDX_COUNT, 0);
      if (err) {
        ESP_LOGE(TAG, "create attr table failed, error code = %x", err);
      }
    } break;

    case ESP_GATTS_READ_EVT:
      ESP_LOGD(TAG, "ESP_GATTS_READ_EVT");
      assert(false);
      break;

    case ESP_GATTS_WRITE_EVT: {
      const esp_ble_gatts_cb_param_t::gatts_write_evt_param& write_param =
          param->write;
      ESP_LOGI(TAG,
          "ESP_GATTS_WRITE_EVT event: handle=%d, is_prep=%d, need_rsp=%d, "
          "len=%d, value:",
          write_param.handle, write_param.is_prep, write_param.need_rsp,
          write_param.len);
      ESP_LOG_BUFFER_HEX_LEVEL(
          TAG, write_param.value, write_param.len, ESP_LOG_INFO);
      assert(!write_param.is_prep);

    } break;

    case ESP_GATTS_MTU_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, mtu set to %d", param->mtu.mtu);
      break;

    case ESP_GATTS_START_EVT:
      ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d",
          param->start.status, param->start.service_handle);
      break;

    case ESP_GATTS_CONNECT_EVT: {
      ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d, remove address:",
          param->connect.conn_id);
      esp_log_buffer_hex(
          TAG, param->connect.remote_bda, sizeof(param->connect.remote_bda));

      esp_ble_conn_update_params_t conn_params = {};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      conn_params.latency = 0;
      conn_params.max_int = 0x20;
      conn_params.min_int = 0x10;
      conn_params.timeout = 400;
      esp_ble_gap_update_conn_params(&conn_params);
      connected = true;
    } break;

    case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x",
          param->disconnect.reason);
      esp_ble_gap_start_advertising(&adv_params);
      connected = false;
      break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      if (param->add_attr_tab.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "create attribute table failed, error code=0x%x",
            param->add_attr_tab.status);
      } else if (param->add_attr_tab.num_handle != ATTR_IDX_COUNT) {
        ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
            param->add_attr_tab.num_handle, ATTR_IDX_COUNT);
      } else {
        ESP_LOGI(TAG, "create attribute table successfully, num handles = %d",
            param->add_attr_tab.num_handle);
        assert(param->add_attr_tab.num_handle == ATTR_IDX_COUNT);
        // memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
        esp_ble_gatts_start_service(param->add_attr_tab.handles[ATTR_IDX_SVC]);
      }
      break;
    }

    case ESP_GATTS_RESPONSE_EVT:
      if (param->rsp.status == ESP_GATT_OK)
        ESP_LOGD(TAG, "ESP_GATTS_RESPONSE_EVT rsp OK");
      else {
        ESP_LOGW(
            TAG, "ESP_GATTS_RESPONSE_EVT rsp error: %d", param->rsp.status);
      }
      break;

    default:
      ESP_LOGI(TAG,
          "Gatt event handler: ignored event esp_gatts_cb_event_t = %d", event);
      break;
  }
}

void setup() {
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_err_t ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(
        TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    assert(0);
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(
        TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    assert(0);
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(
        TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    assert(0);
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(
        TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    assert(0);
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
    assert(0);
  }

  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gap register error, error code = %x", ret);
    assert(0);
  }

  ret = esp_ble_gatts_app_register(0x55);
  if (ret) {
    ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
    assert(0);
  }

  ret = esp_ble_gatt_set_local_mtu(247);
  if (ret) {
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
    assert(0);
  }
}

bool is_connected() {
  return connected;
}

}  // namespace host