/******************************/
/* QuadQuad2Drv Library v1.00 */
/* Author : Christie Nel      */
/* Date   : 08/01/2021        */
/******************************/

#ifndef QUAD_QUAD2_DRV_H
#define QUAD_QUAD2_DRV_H

/* Error Codes */
// General
#define QQ_ERR_OK                     0
#define QQ_ERR_DATA_OVERFLOW          -1
// Command Errors
#define QQ_ERR_CMD_INVALID_CMD        -10
#define QQ_ERR_CMD_INVALID_PARAMETER  -11
#define QQ_ERR_CMD_INVALID_DATA       -12
// SPI
#define QQ_ERR_SPI_RX_FAIL            -50
#define QQ_ERR_SPI_TX_FAIL            -51
#define QQ_ERR_SPI_RX_PACKET_TOO_BIG  -52
#define QQ_ERR_SPI_TX_PACKET_TOO_BIG  -53
#define QQ_ERR_SPI_RX_TIMEOUT         -54
#define QQ_ERR_SPI_TX_TIMEOUT         -55
#define QQ_ERR_SPI_RX_BUF_FULL        -56
#define QQ_ERR_SPI_TX_BUF_FULL        -57
#define QQ_ERR_SPI_QUERY_TIMEOUT      -58
#define QQ_ERR_SPI_STREAM_TIMEOUT     -59
// Misc
#define QQ_ERR_UNSPECIFIED_ERROR      -127

// Core QuadQuad2Drv constants
#define QQ_NUM_CHANS  4

/* Channel Data Masks */
/* Use to set data_mask_t.mask directly */
#define QQ_DATA_CFG_POS_MASK    0x03    // 2-bits = 0/8/16/32-bit
#define QQ_DATA_CFG_POS8        0x01    // DATA_MASK_POS_BITS -> 8-bit position
#define QQ_DATA_CFG_POS16       0x02    // DATA_MASK_POS_BITS -> 16-bit position
#define QQ_DATA_CFG_POS32       0x03    // DATA_MASK_POS_BITS -> 32-bit position
#define QQ_DATA_CFG_POS_REL     0x04
#define QQ_DATA_CFG_SPEED       0x08
//#define QQ_DATA_CFG_ACCEL       0x10
#define QQ_DATA_CFG_STATUS      0x20

/* Stream Config Masks */
#define QQ_STREAM_CFG_PERIOD_TIMING    0x01    // Stream period timing
#define QQ_STREAM_CFG_PERIODS_ELAPSED  0x02    // # Periods elapsed since last stream packet

/* Status masks */
#define QQ_STATUS_MASK_INVALID_BIT                7
#define QQ_STATUS_MASK_OVERSPEED_BIT              6
#define QQ_STATUS_MASK_INPUT_ACTIVE_BIT           3
#define QQ_STATUS_MASK_INPUT_ACCUM_BIT            2
#define QQ_STATUS_MASK_INPUT_TRIGGERED_ACCUM_BIT  0   // and 1
#define QQ_STATUS_MASK_INVALID                    ( 1 << QQ_STATUS_MASK_INVALID_BIT )
#define QQ_STATUS_MASK_OVERSPEED                  ( 1 << QQ_STATUS_MASK_OVERSPEED_BIT )
#define QQ_STATUS_MASK_INPUT_ACTIVE               ( 1 << QQ_STATUS_MASK_INPUT_ACTIVE_BIT )
#define QQ_STATUS_MASK_INPUT_ACCUM                ( 1 << QQ_STATUS_MASK_INPUT_ACCUM_BIT )
#define QQ_STATUS_MASK_INPUT_TRIGGERED_ACCUM      ( 3 << QQ_STATUS_MASK_INPUT_TRIGGERED_ACCUM_BIT )

typedef signed char err;

typedef enum
{
  BITSIZE_ZERO  = 0,
  BITSIZE_8     = QQ_DATA_CFG_POS8,
  BITSIZE_16    = QQ_DATA_CFG_POS16,
  BITSIZE_32    = QQ_DATA_CFG_POS32
} E_BITSIZE;

typedef union
{
  struct
  {
    E_BITSIZE position_bitsize : 2;
    bool position_relative : 1;
    bool velocity_enable : 1;
    uint8_t reserved : 1;
    bool status_enable : 1;
    uint8_t unused : 2;
  };
  uint8_t mask;
} data_mask_t;

typedef enum
{
  CHAN_NONE_MASK  = 0x00,
  CHAN1_MASK      = 0x01,
  CHAN2_MASK      = 0x02,
  CHAN3_MASK      = 0x04,
  CHAN4_MASK      = 0x08,
  CHAN_ALL_MASK   = 0x0F
} E_CHAN_MASK;

typedef enum
{
    CTRL_MODE_NONE      = 0,
    CTRL_MODE_POWER     = 1,
    CTRL_MODE_POSITION  = 2,
    CTRL_MODE_SPEED     = 3
} E_CTRL_MODE;

typedef struct
{
  E_CHAN_MASK chan_mask : 8;
  data_mask_t data_mask;
} chan_data_masks_t;

typedef data_mask_t data_masks_t[QQ_NUM_CHANS];

typedef struct
{
  int32_t position;
  int32_t velocity;
  uint8_t status;
  bool overspeed;
  bool glitch;
} chan_motion_data_t;

typedef chan_motion_data_t motion_data_t[QQ_NUM_CHANS];

typedef struct
{
  uint16_t period_timing;
  uint8_t periods_elapsed;
  motion_data_t motion_data;
} stream_data_t;

typedef union
{
  struct
  {
    uint8_t period_timing_enable : 1;
    uint8_t periods_elapsed_enable : 1;
    uint8_t reserved : 6;
  };
  uint8_t mask;
} stream_config_t;

typedef enum
{
  INPUT_MODE_DISABLED = 0x00,
  INPUT_MODE_HOME     = 0x01,
  INPUT_MODE_INDEX    = 0x02,
  INPUT_MODE_EDGE     = 0x03,
} E_INPUT_MODE;

typedef enum
{
  INPUT_ACTIVE_HIGH = 0x00,
  INPUT_ACTIVE_LOW  = 0x01
} E_INPUT_POLARITY;

typedef enum
{
  INPUT_EDGE_REC_POSITIVE = 0b00,
  INPUT_EDGE_REC_NEGATIVE = 0b10,
  INPUT_EDGE_SET_POSITIVE = 0b01,
  INPUT_EDGE_SET_NEGATIVE = 0b11,
} E_INPUT_EDGE;

typedef struct
{
  E_CHAN_MASK chan : 4;
  E_BITSIZE value_bitsize : 4;
  int32_t value;
} position_params_t;

typedef struct
{
  // Byte 1
  E_CHAN_MASK chan : 8;
  // Byte 2
  E_INPUT_MODE input_mode : 3;
  E_INPUT_POLARITY input_polarity : 1;
  E_INPUT_EDGE input_edge : 4;
  // Counters
  uint16_t spacing;
  int32_t value;
} input_params_t;

typedef int32_t positions_t[QQ_NUM_CHANS];

typedef enum
{
  GET_POS_AS_CONFIGURED = 0,
  GET_POS_BY_CHAN_MASK = 1,
  GET_POS_BY_SELECTION = 4
} E_GET_POS_MODE;

typedef struct
{
  E_CHAN_MASK chan : 8;
  E_CTRL_MODE ctrl_mode;
} ctrl_mode_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  int16_t drv_power;
} ctrl_drv_power_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  int32_t pos_target;
} ctrl_pos_target_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  int32_t speed_target;
} ctrl_speed_target_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  uint16_t p;
  uint16_t i;
  uint16_t d;
  uint16_t windup_limit;
} ctrl_pos_pid_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  uint16_t p;
  uint16_t i;
  uint16_t d;
} ctrl_speed_pid_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  uint32_t speed_limit;
} ctrl_speed_limit_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  uint32_t accel_limit;
} ctrl_accel_limit_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  int32_t pos_target;
  uint32_t ramp_rate;
  uint32_t ramp_accel;
} ctrl_pos_ramp_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  int32_t speed_target;
  uint32_t ramp_accel;
} ctrl_speed_ramp_params_t;

typedef struct
{
  E_CHAN_MASK chan : 8;
  uint16_t drv_power_max;
} ctrl_drv_power_max_params_t;

class QuadQuad2Drv
{
  public:
    QuadQuad2Drv( uint8_t pin_chip_select_init, uint8_t pin_ready_init, uint8_t pin_reset_init );
    
    void reset( void );
    err get_version           ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t *fw_ver_major, uint8_t *fw_ver_minor, uint8_t *protocol_ver );
    err get_motion_data       ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, data_masks_t data_masks, motion_data_t motion_data );
    err read_stream           ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, stream_config_t stream_config, data_masks_t data_masks, stream_data_t *stream_data );
    err set_stream_period     ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint16_t stream_period );
    err get_stream_period     ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint16_t *stream_period );
    err set_data_mask         ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, chan_data_masks_t *chan_data_masks, uint8_t chan_data_masks_num );
    err get_data_mask         ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, data_masks_t data_masks );
    err set_stream_config     ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, stream_config_t stream_config );
    err get_stream_config     ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, stream_config_t *stream_config );
    err set_position          ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, position_params_t *position_params, uint8_t position_params_num );
    err get_position          ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, E_GET_POS_MODE mode, E_CHAN_MASK chan_mask, data_masks_t data_masks, positions_t positions );
    err set_history_dims      ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t history_len, uint8_t history_time_bits );
    err get_history_dims      ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, uint8_t *history_len, uint8_t *history_time_bits );
    err set_input_mode        ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, input_params_t *input_params, uint8_t input_params_num );
    err set_ctrl_mode         ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_mode_params_t *ctrl_mode_params, uint8_t ctrl_mode_params_num );
    err set_ctrl_drv_power    ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_drv_power_params_t *ctrl_drv_power_params, uint8_t ctrl_drv_power_params_num );
    err set_ctrl_pos_target   ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_pos_target_params_t *ctrl_pos_target_params, uint8_t ctrl_pos_target_params_num );
    err set_ctrl_speed_target ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_speed_target_params_t *ctrl_speed_target_params, uint8_t ctrl_speed_target_params_num );
    err set_ctrl_drv_power_max( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_drv_power_max_params_t *ctrl_drv_power_max_params, uint8_t ctrl_drv_power_max_params_num );
    err set_ctrl_speed_limit  ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_speed_limit_params_t *ctrl_speed_limit_params, uint8_t ctrl_speed_limit_params_num );
    err set_ctrl_accel_limit  ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_accel_limit_params_t *ctrl_accel_limit_params, uint8_t ctrl_accel_limit_params_num );
    err set_ctrl_pos_pid      ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_pos_pid_params_t *ctrl_pos_pid_params, uint8_t ctrl_pos_pid_num );
    err set_ctrl_speed_pid    ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_speed_pid_params_t *ctrl_speed_pid_params, uint8_t ctrl_speed_pid_num );
    err ctrl_pos_ramp         ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_pos_ramp_params_t *ctrl_pos_ramp_params, uint8_t ctrl_pos_ramp_params_num );
    err ctrl_speed_ramp       ( uint8_t *buf, uint8_t buf_size, int16_t *payload_size, err *packet_rc, ctrl_speed_ramp_params_t *ctrl_speed_ramp_params, uint8_t ctrl_speed_ramp_params_num );
    
    uint16_t period_to_ms( uint16_t period );
    uint16_t ms_to_period( uint16_t ms );
    uint16_t history_time_bits_to_ms( uint8_t history_time_bits );
    
    err spi_queryPacket( uint8_t packet_type, uint8_t *payload_tx, uint8_t payload_size_tx, uint8_t *payload_rx, int16_t *payload_size_rx, uint8_t payload_buf_size );
  private:
    uint8_t pin_chip_select;
    uint8_t pin_ready;
    uint8_t pin_reset;
    
    uint16_t stream_timeout_ms;
    
    uint8_t spi_read_byte( byte *byte_read );
    err spi_write( uint8_t *data, uint8_t len );
    void spi_sendBytes( uint8_t *data, uint8_t data_len, uint8_t *checksum, err *error );
    err spi_sendPacket( uint8_t packet_type, uint8_t *payload, uint8_t payload_size, err return_code );
    err spi_receivePacket( uint8_t *packet_type, uint8_t *payload, uint8_t payload_buf_size, int16_t *payload_size );
    err spi_readStreamPacket( uint8_t *payload_rx, int16_t *payload_size_rx, uint8_t payload_buf_size );
    void parse_motion_data( uint8_t **buf, data_masks_t data_masks, motion_data_t motion_data );
};

#endif