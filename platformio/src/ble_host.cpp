
#include <algorithm>
#include <string.h>

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

// using gatts_read_evt_param = esp_ble_gatts_cb_param_t::gatts_read_evt_param;
// using gatts_write_evt_param =
// esp_ble_gatts_cb_param_t::gatts_write_evt_param;

namespace ble_host {

static constexpr auto TAG = "ble_host";

// #define ESP_APP_ID 0x55
// #define SVC_INST_ID 0

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

// static constexpr uint16_t kMaxRequestedMtu = 247;
// static constexpr uint16_t kMtuOverhead = 3;

// Invalid connection id (0xffff);
// constexpr uint16_t kInvalidConnId = -1;

static const uint8_t service_uuid[] = {
    ENCODE_UUID_128(0x6b6a78d7, 0x8ee0, 0x4a26, 0xba7b, 0x62e357dd9720)};

static const uint8_t model_uuid[] = {ENCODE_UUID_16(0x2a24)};
static const uint8_t revision_uuid[] = {ENCODE_UUID_16(0x2a26)};
static const uint8_t manufacturer_uuid[] = {ENCODE_UUID_16(0x2a29)};
static const uint8_t probe_info_uuid[] = {ENCODE_UUID_16(0xff01)};
static const uint8_t stepper_state_uuid[] = {ENCODE_UUID_16(0xff02)};
static const uint8_t current_histogram_uuid[] = {ENCODE_UUID_16(0xff03)};
static const uint8_t time_histogram_uuid[] = {ENCODE_UUID_16(0xff04)};
static const uint8_t distance_histogram_uuid[] = {ENCODE_UUID_16(0xff05)};
static const uint8_t command_uuid[] = {ENCODE_UUID_16(0xff06)};
static const uint8_t capture_uuid[] = {ENCODE_UUID_16(0xff07)};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
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
static uint8_t kChrConfigDeclUuid[] = {
    ENCODE_UUID_16(ESP_GATT_UUID_CHAR_CLIENT_CONFIG)};
static uint8_t kChrPropertyReadNotify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t kChrPropertyReadOnly = ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t kChrPropertyWriteNrOnly = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

// TODO: what does it do?
static uint8_t state_ccc_val[2] = {};

// TODO: why do we need this?
static uint8_t command_val[1] = {};

// Model string = "Stepper Probe ESP32"
static const uint8_t model_str_value[] = {'S', 't', 'e', 'p', 'p', 'e', 'r',
    ' ', 'P', 'r', 'o', 'b', 'e', ' ', 'E', 'S', 'P', '3', '2'};

// Revision string = "00.00.01"
static const uint8_t revision_str_value[] = {
    '0', '0', '.', '0', '0', '.', '0', '1'};

// Manufacturer string = "Zapta"
static const uint8_t manufacturer_str_value[] = {'Z', 'a', 'p', 't', 'a'};

enum {
  ATTR_IDX_SVC,

  // ATTR_IDX_MODEL,
  // ATTR_IDX_MODEL_VAL,

  // ATTR_IDX_REVISION,
  // ATTR_IDX_REVISION_VAL,

  // ATTR_IDX_MANUFECTURER,
  // ATTR_IDX_MANUFECTURER_VAL,

  // ATTR_IDX_PROBE_INFO,
  // ATTR_IDX_PROBE_INFO_VAL,

  // ATTR_IDX_STEPPER_STATE,
  // ATTR_IDX_STEPPER_STATE_VAL,
  // ATTR_IDX_STEPPER_STATE_CCC,

  // ATTR_IDX_CURRENT_HISTOGRAM,
  // ATTR_IDX_CURRENT_HISTOGRAM_VAL,

  // ATTR_IDX_TIME_HISTOGRAM,
  // ATTR_IDX_TIME_HISTOGRAM_VAL,

  // ATTR_IDX_DISTANCE_HISTOGRAM,
  // ATTR_IDX_DISTANCE_HISTOGRAM_VAL,

  ATTR_IDX_COMMAND,
  ATTR_IDX_COMMAND_VAL,

  // ATTR_IDX_CAPTURE,
  // ATTR_IDX_CAPTURE_VAL,

  ATTR_IDX_COUNT,  // Count.
};

// NOTE: x can't be const.
#define LEN_BYTES(x) sizeof(x), (uint8_t*)(&(x))
#define LEN_LEN_BYTES(x) sizeof(x), sizeof(x), (uint8_t*)(&(x))

// The main BLE attribute table.
static const esp_gatts_attr_db_t attr_table[ATTR_IDX_COUNT] = {

    [ATTR_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
        {LEN_BYTES(kPrimaryServiceDeclUuid), ESP_GATT_PERM_READ,
            LEN_LEN_BYTES(service_uuid)}},

    // // ----- Device Model.
    // //
    // // Characteristic
    // [ATTR_IDX_MODEL] =

    //     {{ESP_GATT_AUTO_RSP},
    //         {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //             LEN_LEN_BYTES(kChrPropertyReadOnly)}},

    // // Value.
    // [ATTR_IDX_MODEL_VAL] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(model_uuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(model_str_value)}},

    // // ----- Firmware revision
    // //
    // // Characteristic
    // [ATTR_IDX_REVISION] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},
    // // Value.
    // [ATTR_IDX_REVISION_VAL] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(revision_uuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(revision_str_value)}},

    // // ----- Manufacturer name
    // //
    // // Characteristic
    // [ATTR_IDX_MANUFECTURER] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},
    // // Value.
    // [ATTR_IDX_MANUFECTURER_VAL] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(manufacturer_uuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(manufacturer_str_value)}},

    // // ----- Probe info
    // //
    // // Characteristic
    // [ATTR_IDX_PROBE_INFO] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},

    // // Value
    // [ATTR_IDX_PROBE_INFO_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(probe_info_uuid), ESP_GATT_PERM_READ, 0, 0, nullptr}},

    // // ----- Stepper state.
    // //
    // // Characteristic
    // [ATTR_IDX_STEPPER_STATE] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadNotify)}},
    // // Value
    // [ATTR_IDX_STEPPER_STATE_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(stepper_state_uuid), ESP_GATT_PERM_READ, 0, 0, nullptr}},

    // // Client Characteristic Configuration Descriptor
    // [ATTR_IDX_STEPPER_STATE_CCC] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kChrConfigDeclUuid),
    //         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    //         LEN_LEN_BYTES(state_ccc_val)}},

    // // ----- Current histogram.
    // //
    // // Characteristic
    // [ATTR_IDX_CURRENT_HISTOGRAM] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},
    // // Value
    // [ATTR_IDX_CURRENT_HISTOGRAM_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(current_histogram_uuid), ESP_GATT_PERM_READ, 0, 0, nullptr}},

    // // ----- Time histogram.
    // //
    // // Characteristic
    // [ATTR_IDX_TIME_HISTOGRAM] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},
    // // Value
    // [ATTR_IDX_TIME_HISTOGRAM_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(time_histogram_uuid), ESP_GATT_PERM_READ, 0, 0, nullptr}},

    // // ----- Distance histogram.
    // //
    // // Characteristic
    // [ATTR_IDX_DISTANCE_HISTOGRAM] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},
    // // Value
    // [ATTR_IDX_DISTANCE_HISTOGRAM_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(distance_histogram_uuid), ESP_GATT_PERM_READ, 0, 0,
    //         nullptr}},

    // ----- Command.
    //
    // Characteristic
    [ATTR_IDX_COMMAND] = {{ESP_GATT_AUTO_RSP},
        {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
            LEN_LEN_BYTES(kChrPropertyWriteNrOnly)}},

    // Value
    [ATTR_IDX_COMMAND_VAL] = {{ESP_GATT_AUTO_RSP},
        {LEN_BYTES(command_uuid), ESP_GATT_PERM_WRITE,
            LEN_LEN_BYTES(command_val)}},

    // // ----- Capture.
    // //
    // // Characteristic
    // [ATTR_IDX_CAPTURE] = {{ESP_GATT_AUTO_RSP},
    //     {LEN_BYTES(kCharDeclUuid), ESP_GATT_PERM_READ,
    //         LEN_LEN_BYTES(kChrPropertyReadOnly)}},

    // // Value
    // [ATTR_IDX_CAPTURE_VAL] = {{ESP_GATT_RSP_BY_APP},
    //     {LEN_BYTES(capture_uuid), ESP_GATT_PERM_READ, 0, 0, nullptr}},

};

static uint16_t handle_table[ATTR_IDX_COUNT];

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

      // #endif
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
      const char* device_name = "esp32-write-by-type";
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
    } break;

    case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x",
          param->disconnect.reason);
      esp_ble_gap_start_advertising(&adv_params);
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
        memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
        esp_ble_gatts_start_service(handle_table[ATTR_IDX_SVC]);
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

}  // namespace ble_host