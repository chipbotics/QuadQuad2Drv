/**************************/
/* QuadQuad2Drv Demo      */
/* Author : Christie Nel  */
/* Date   : 11/01/2021    */
/**************************/

/* How to get started:
 *
 * Note: This demo was tested on an Arduino Nano v3.
 *
 * (1) Connect the Arduino to the QuadQuad2Drv PCB:
 *     - GND to GND
 *     - 5V to V+
 *     - MISO to SDO
 *     - MOSI to SDI
 *     - SCK to SCK
 *     - qq_pin_chip_select to SS
 *     - qq_pin_backoff to SRDY
 *     - qq_pin_reset to RESET (optional)
 * (2) Connect a motor with quadrature feedback to the board on connectors M1 and DRV1.
 *     If the polarity of either the A/B inputs or the M1 output is wrong, the
 *     motor will spin very fast when the program runs.  If this is the case, reverse
 *     either A/B or M1.  This demo is tuned for a Pololu microgear motor at 12V.
 * (3) Open the Arduino Serial Monitor and set it to 115200 baud.
 * (4) Download this program to your Arduino.  If all is well, you should see
 *     serial output and the motor will ramp to position 5000 at a speed of
 *     2000 transitions/s and stop at position 5000, before ramping back slowly to
 *     position 0.
 * (5) You may touch the DRV1 IN pin to GND to test the HOME input.  This will force
 *     the position counter to 0.
 * 
 * Please read the QuadQuad2Drv Datasheet to better understand the interface and
 * read through the QuadQuad2Drv_Lib.h file for a list of all functions, errors
 * and typedefs.
 */

#include <SPI.h>
#include "QuadQuad2Drv_Lib.h"

/* Local defines */
#define FALSE 0
#define TRUE 1

/* Test constants */
#define POSITION_TARGET     5000
#define POSITION_ZERO       0
  
/* ---------------------------------------------- */

/* Arduino pins */
const uint8_t qq_pin_chip_select = 10;
const uint8_t qq_pin_backoff = 2;
const uint8_t qq_pin_reset = 3;

/* Instantiate QuadQuad2Drv class with configured pins */
QuadQuad2Drv qq2drv( qq_pin_chip_select, qq_pin_backoff, qq_pin_reset );

/* Global variables */
err rc;
err packet_rc;
int16_t payload_size;
byte buf[64];
uint8_t chan;
stream_config_t stream_config;
data_masks_t data_masks;

// ----------------------------------------------------------

void setup()
{
  Serial.begin( 115200 );
  Serial.println();
}

void print_result( err rc, err packet_rc )
{
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
    Serial.println( "Success" );
  else
  {
    Serial.print( "Error " );
    Serial.print( rc );
    Serial.print( ", Packet Error " );
    Serial.println( packet_rc );
  }
}

void print_progress( int32_t position_target, bool direction_positive )
{
  uint32_t sample;
  uint32_t time;
  uint32_t overtime_ms;
  bool reached_target;
  stream_data_t stream_data;
  
  sample = 0;
  overtime_ms = 0;
  reached_target = FALSE;
  
  while ( ( rc == QQ_ERR_OK ) && ( overtime_ms < 1000 ) )
  {
    rc = qq2drv.read_stream( buf, sizeof(buf), &payload_size, stream_config, data_masks, &stream_data );
    
    if ( rc == QQ_ERR_OK )
    {
      sample++;
      
      Serial.print( "Sample " );
      Serial.print( sample );
      
      Serial.print( ", Timing=" );
      Serial.print( stream_data.period_timing );
      Serial.print( "=" );
      Serial.print( qq2drv.period_to_ms( stream_data.period_timing ) );
      Serial.print( "ms" );
      
      Serial.print( ", Elapsed=" );
      Serial.print( stream_data.periods_elapsed );
      
      for ( chan=0; chan<QQ_NUM_CHANS; chan++ )
      {
        Serial.println();
        Serial.print( "Channel " );
        Serial.print( chan + 1 );
        Serial.print( " Position=" );
        Serial.print( stream_data.motion_data[chan].position );
        Serial.print( ", Velocity=" );
        Serial.print( stream_data.motion_data[chan].velocity );
      }
      
      Serial.println();
      
      if ( !reached_target )
      {
        if ( ( (  direction_positive ) && ( stream_data.motion_data[0].position >= position_target ) ) ||
             ( ( !direction_positive ) && ( stream_data.motion_data[0].position <= position_target ) ) )
        {
          /* When we reach the target, keep printing for a short time after */
          
          reached_target = TRUE;
          time = millis();
        }
      }
      else
        overtime_ms = millis() - time;
    }
    else
    {
      print_result( rc, QQ_ERR_OK );
      break;
    }
  }
  
}

void loop()
{
  uint8_t fw_ver_major;
  uint8_t fw_ver_minor;
  uint8_t protocol_ver;
  chan_data_masks_t chan_data_masks;
  motion_data_t motion_data;
  input_params_t input_params[2];
  ctrl_mode_params_t ctrl_mode_params;
  ctrl_pos_target_params_t ctrl_pos_target_params;
  ctrl_pos_ramp_params_t ctrl_pos_ramp_params;
  ctrl_drv_power_max_params_t ctrl_drv_power_max_params;
  ctrl_pos_pid_params_t ctrl_pos_pid_params;
  ctrl_speed_pid_params_t ctrl_speed_pid_params;
  ctrl_speed_limit_params_t ctrl_speed_limit_params;
  ctrl_accel_limit_params_t ctrl_accel_limit_params;
  uint8_t history_len;
  uint8_t history_time_bits;
  uint16_t stream_period;
  
  Serial.println( "QuadQuad2Drv Demo" );
  Serial.println( "--------------------------------------------" );
  
  /* --------------------------------------------------------- */
  /* Report configured Arduino pins                            */
  
  Serial.println( "Arduino pin connections..." );
  Serial.print( "QuadQuad2Drv Chip Select Pin : " );
  Serial.println( qq_pin_chip_select );
  Serial.print( "QuadQuad2Drv Ready Pin       : " );
  Serial.println( qq_pin_backoff );
  Serial.print( "QuadQuad2Drv Reset Pin       : " );
  Serial.println( qq_pin_reset );
  Serial.println( "Standard Arduino SPI pins are used." );
  
  /* --------------------------------------------------------- */
  /* Reset QuadQuad2Drv chip by pulling low reset line             */
  
  Serial.println();
  Serial.println( "Resetting QuadQuad2Drv..." );
  qq2drv.reset();
  delay( 10 );
  
  /* --------------------------------------------------------- */
  /* Read QuadQuad2Drv version information                         */
  
  Serial.println();
  Serial.print( "Reading version..." );
  rc = qq2drv.get_version( buf, sizeof(buf), &payload_size, &packet_rc, &fw_ver_major, &fw_ver_minor, &protocol_ver );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "Firmware Version Major : " );
    Serial.println( fw_ver_major );
    Serial.print( "Firmware Version Minor : " );
    Serial.println( fw_ver_minor );
    Serial.print( "Protocol Version       : " );
    Serial.println( protocol_ver );
  }
  
  Serial.println( "--------------------------------------------" );
  
  /* --------------------------------------------------------- */
  /* Configure which motion data is reported and its format    */
  
  Serial.print( "Configure channel data fields..." );
  chan_data_masks.chan_mask = CHAN_ALL_MASK;               // Which channels to set
  chan_data_masks.data_mask.position_bitsize = BITSIZE_32; // How many bits in position
  chan_data_masks.data_mask.position_relative = FALSE;     // Return delta or absolute position
  chan_data_masks.data_mask.velocity_enable = TRUE;        // Return velocity
  chan_data_masks.data_mask.status_enable = TRUE;          // Return status
  rc = qq2drv.set_data_mask( buf, sizeof(buf), &payload_size, &packet_rc, &chan_data_masks, 1 );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Read back channel data fields..." );
  rc = qq2drv.get_data_mask( buf, sizeof(buf), &payload_size, &packet_rc, data_masks );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Read motion data using polling                            */
  
  Serial.println();
  Serial.print( "Read motion data..." );
  rc = qq2drv.get_motion_data( buf, sizeof(buf), &payload_size, &packet_rc, data_masks, motion_data );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    for ( chan=0; chan<QQ_NUM_CHANS; chan++ )
    {
      Serial.print( "Channel " );
      Serial.print( chan + 1 );
      Serial.println( " : " );
      Serial.print( "  Position  : " );
      Serial.println( motion_data[chan].position );
      Serial.print( "  Velocity  : " );
      Serial.println( motion_data[chan].velocity );
      Serial.print( "  Glitch    : " );
      Serial.println( motion_data[chan].glitch ? "TRUE" : "FALSE" );
      Serial.print( "  Overspeed : " );
      Serial.println( motion_data[chan].overspeed ? "TRUE" : "FALSE" );
    }
  }
  
  /* --------------------------------------------------------- */
  /* Read and write history dimensions                         */
  
  Serial.println();
  Serial.print( "Get history dimensions..." );
  rc = qq2drv.get_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, &history_len, &history_time_bits );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "History Length : " );
    Serial.println( history_len );
    Serial.print( "History Time Bits : " );
    Serial.print( history_time_bits );
    Serial.print( " = " );
    Serial.print( qq2drv.history_time_bits_to_ms( history_time_bits ) );
    Serial.println( "ms" );
  }
  
  Serial.println();
  Serial.print( "Set history dimensions..." );
  history_len = 24;
  history_time_bits = 18;
  rc = qq2drv.set_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, history_len, history_time_bits );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Get history dimensions..." );
  rc = qq2drv.get_history_dims( buf, sizeof(buf), &payload_size, &packet_rc, &history_len, &history_time_bits );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "History Length : " );
    Serial.println( history_len );
    Serial.print( "History Time Bits : " );
    Serial.print( history_time_bits );
    Serial.print( " = " );
    Serial.print( qq2drv.history_time_bits_to_ms( history_time_bits ) );
    Serial.println( "ms" );
  }
  
  /* --------------------------------------------------------- */
  /* Enable reporting of stream timing and periods elapsed     */
  
  Serial.println();
  Serial.print( "Set stream config..." );
  stream_config.period_timing_enable = TRUE;
  stream_config.periods_elapsed_enable = TRUE;
  rc = qq2drv.set_stream_config( buf, sizeof(buf), &payload_size, &packet_rc, stream_config );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Enable input on channel 1 to set position when pulled low */
  
  Serial.println();
  Serial.print( "Set input mode..." );
  /* Set channel 1 position when input is low */
  input_params[0].chan = CHAN1_MASK;
  input_params[0].input_mode = INPUT_MODE_HOME;
  input_params[0].input_polarity = INPUT_ACTIVE_LOW;
  input_params[0].input_edge = INPUT_EDGE_SET_POSITIVE;
  input_params[0].spacing = 0;
  input_params[0].value = 0;
  /* Disable inputs on other channels */
  input_params[1].chan = ~CHAN1_MASK;
  input_params[1].input_mode = INPUT_MODE_DISABLED;
  rc = qq2drv.set_input_mode( buf, sizeof(buf), &payload_size, &packet_rc, input_params, 2 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Set position control PID constants                        */
  
  Serial.println();
  Serial.print( "Set position control PID constants..." );
  ctrl_pos_pid_params.chan = CHAN1_MASK;
  ctrl_pos_pid_params.p = 100;
  ctrl_pos_pid_params.i = 500;
  ctrl_pos_pid_params.d = 500;
  ctrl_pos_pid_params.windup_limit = 600;
  rc = qq2drv.set_ctrl_pos_pid( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_pos_pid_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Set control max power                                     */
  
  Serial.println();
  Serial.print( "Set control max power..." );
  ctrl_drv_power_max_params.chan = CHAN1_MASK;
  ctrl_drv_power_max_params.drv_power_max = 2000;
  rc = qq2drv.set_ctrl_drv_power_max( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_drv_power_max_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Set control speed limit                                   */
  
  Serial.println();
  Serial.print( "Set control speed limit..." );
  ctrl_speed_limit_params.chan = CHAN1_MASK;
  ctrl_speed_limit_params.speed_limit = ( (uint32_t)2000 << 12 ) / 1;
  rc = qq2drv.set_ctrl_speed_limit( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_speed_limit_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Set control acceleration limit                            */
  
  Serial.println();
  Serial.print( "Set control acceleration limit..." );
  ctrl_accel_limit_params.chan = CHAN1_MASK;
  ctrl_accel_limit_params.accel_limit = ( (uint32_t)2000 << 12 ) / 1;
  rc = qq2drv.set_ctrl_accel_limit( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_accel_limit_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Set control mode                                          */
  
  Serial.println();
  Serial.print( "Set position control mode..." );
  ctrl_mode_params.chan = CHAN1_MASK;
  ctrl_mode_params.ctrl_mode = CTRL_MODE_POSITION;
  rc = qq2drv.set_ctrl_mode( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_mode_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Read and write stream period                              */
  
  Serial.println();
  Serial.print( "Set stream period..." );
  rc = qq2drv.set_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, qq2drv.ms_to_period( 200 ) );
  print_result( rc, packet_rc );
  
  Serial.println();
  Serial.print( "Get stream period..." );
  rc = qq2drv.get_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, &stream_period );
  print_result( rc, packet_rc );
  
  if ( ( rc == QQ_ERR_OK ) && ( packet_rc == QQ_ERR_OK ) )
  {
    Serial.print( "Stream Period : " );
    Serial.print( stream_period );
    Serial.print( " = " );
    Serial.print( qq2drv.period_to_ms( stream_period ) );
    Serial.println( "ms" );
  }
  
  /* --------------------------------------------------------- */
  /* Set position target                                       */
  
  Serial.println();
  Serial.println( "Set position target..." );
  ctrl_pos_target_params.chan = CHAN1_MASK;
  ctrl_pos_target_params.pos_target = POSITION_TARGET;
  rc = qq2drv.set_ctrl_pos_target( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_pos_target_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Read stream and motion data from stream                   */
  
  Serial.println();
  Serial.println( "Reading stream..." );
  
  /* --------------------------------------------------------- */
  /* Print progress until target is reached                    */
  
  print_progress( POSITION_TARGET, TRUE );
  
  /* --------------------------------------------------------- */
  /* Initiate control ramp                                     */
  
  Serial.println();
  Serial.print( "Initiate slow ramp to position 0..." );
  ctrl_pos_ramp_params.chan = CHAN1_MASK;
  ctrl_pos_ramp_params.pos_target = POSITION_ZERO;
  ctrl_pos_ramp_params.ramp_rate = ( (uint32_t)1000 << 12 ) / 1;
  ctrl_pos_ramp_params.ramp_accel = ( (uint32_t)200 << 12 ) / 1;
  rc = qq2drv.ctrl_pos_ramp( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_pos_ramp_params, 1 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Print progress until target is reached                    */
  
  print_progress( POSITION_ZERO, FALSE );
  
  /* --------------------------------------------------------- */
  /* Turn off stream by setting stream period to 0             */
  
  Serial.println();
  Serial.print( "Turn off stream..." );
  rc = qq2drv.set_stream_period( buf, sizeof(buf), &payload_size, &packet_rc, 0 );
  print_result( rc, packet_rc );
  
  /* --------------------------------------------------------- */
  /* Turn off control mode                                     */
  
  Serial.println();
  Serial.print( "Turn off control mode..." );
  ctrl_mode_params.chan = CHAN1_MASK;
  ctrl_mode_params.ctrl_mode = CTRL_MODE_NONE;
  rc = qq2drv.set_ctrl_mode( buf, sizeof(buf), &payload_size, &packet_rc, &ctrl_mode_params, 1 );
  print_result( rc, packet_rc );
  
  Serial.println( "--------------------------------------------" );
  
  while (1);
}
