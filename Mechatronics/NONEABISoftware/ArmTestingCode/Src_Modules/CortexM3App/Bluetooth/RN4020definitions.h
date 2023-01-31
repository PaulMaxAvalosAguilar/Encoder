#ifndef RN4020_DEFINITIONS
#define RN4020_definitions
#define RN4020_MAX_UUID_LEN_BYTES (128 / 8)

//SUPPORTED FEATURES
#define RN4020_FEATURE_PERIFERAL           0x00000000
#define RN4020_FEATURE_CENTRAL             0x80000000
#define RN4020_FEATURE_REAL_TIME_READ      0x40000000
#define RN4020_FEATURE_AUTO_ADVERTISE      0x20000000
#define RN4020_FEATURE_ENABLE_MLDP         0x10000000
#define RN4020_FEATURE_AUTO_MLDP_DISABLE   0x08000000
#define RN4020_FEATURE_NO_DIRECT_ADVERTISE 0x04000000
#define RN4020_FEATURE_UART_FLOW_CONTROL   0x02000000
#define RN4020_FEATURE_RUN_SCRIPT_PWR_ON   0x01000000
#define RN4020_FEATURE_ENABLE_AUTH         0x00400000
#define RN4020_FEATURE_ENABLE_REMOTE_CMD   0x00200000
#define RN4020_FEATURE_DO_NOT_SAVE_BONDING 0x00100000
#define RN4020_FEATURE_IO_CAP              0x00E00000
#define RN4020_FEATURE_BLOCK_SET_CMD       0x00010000
#define RN4020_FEATURE_ENABLE_OTA          0x00008000
#define RN4020_FEATURE_IOS_MODE            0x00004000
#define RN4020_FEATURE_SERVER_ONLY         0x00002000
#define RN4020_FEATURE_ENABLE_UART_SCRIPT  0x00001000
#define RN4020_FEATURE_AUTO_MLDP           0x00000800
#define RN4020_FEATURE_MLDP_WITHOUT_STATUS 0x00000400

//SUPPORTED SERVICES
#define RN4020_SERVICE_DEVICE_INFORMATION    0x80000000
#define RN4020_SERVICE_BATTERY               0x40000000
#define RN4020_SERVICE_HEART_RATE            0x20000000
#define RN4020_SERVICE_HEALTH_THERMOMETER    0x10000000
#define RN4020_SERVICE_GLUCOSE               0x08000000
#define RN4020_SERVICE_BLOOD_PRESSURE        0x04000000
#define RN4020_SERVICE_RUNNING_SPEED_CADENCE 0x02000000
#define RN4020_SERVICE_CYCLING_SPEED_CADENCE 0x01000000
#define RN4020_SERVICE_CURRENT_TIME          0x00800000
#define RN4020_SERVICE_NEXT_DST_CHANGE       0x00400000
#define RN4020_SERVICE_REFERENCE_TIME_UPDATE 0x00200000
#define RN4020_SERVICE_LINK_LOSS             0x00100000
#define RN4020_SERVICE_IMMEDIATE_ALERT       0x00080000
#define RN4020_SERVICE_TX_POWER              0x00040000
#define RN4020_SERVICE_ALERT_NOTIFICATION    0x00020000
#define RN4020_SERVICE_PHONE_ALERT_STATUS    0x00010000
#define RN4020_SERVICE_SCAN_PARAMETERS       0x00004000
#define RN4020_SERVICE_USER_DEFINED          0x00000001

//CHARACTERISTIC PROPERTIES
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_EXTENDED               0b10000000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_AUTH_WRITE             0b01000000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_INDICATE               0b00100000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_NOTIFY                 0b00010000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_WRITE                  0b00001000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_WRITE_WITHOUT_RESPONSE 0b00000100
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_READ                   0b00000010
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_BROADCAST              0b00000001

//SECURITY FLAGS OF CHARACTERISTIC
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_NONE     0b00000000
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_ENCR_R   0b00000001
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_AUTH_R   0b00000010
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_ENCR_W   0b00010000
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_AUTH_W   0b00100000

#endif
