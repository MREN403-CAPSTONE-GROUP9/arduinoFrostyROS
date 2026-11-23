#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <NMEAGPS.h>

#include <GPSport.h>

// Check that the config files are set up properly

#if !defined( NMEAGPS_PARSE_RMC )
  #error You must uncomment NMEAGPS_PARSE_RMC in NMEAGPS_cfg.h!
#endif

#if !defined( GPS_FIX_TIME )
  #error You must uncomment GPS_FIX_TIME in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SPEED )
  #error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SATELLITES )
  #error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

static NMEAGPS  gps; // This parses the GPS characters

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    while (gps.available( gpsPort )){
      doSomeWork( gps.read() );
    }
    gpsMsg = gps.read();
    RCSOFTCHECK(rcl_publish(&publisher, gpsMsg, NULL));
    
  }
}


// ______________________________-
//gps code

static void printL( Print & outs, int32_t degE7 );
static void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg*10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)){
    outs.print( '0' );
    factor /= 10L;
  }
  
  // Print fractional degrees
  outs.print( degE7 );
}

static void doSomeWork( const gps_fix & fix );
static void doSomeWork( const gps_fix & fix )
{
  //  This is the best place to do your time-consuming work, right after
  //     the RMC sentence was received.  If you do anything in "loop()",
  //     you could cause GPS characters to be lost, and you will not
  //     get a good lat/lon.
  //  For this example, we just print the lat/lon.  If you print too much,
  //     this routine will not get back to "loop()" in time to process
  //     the next set of GPS data.

  if (fix.valid.location) {

    if ( fix.dateTime.seconds < 10 )
      DEBUG_PORT.print( '0' );
    DEBUG_PORT.print( fix.dateTime.seconds );
    DEBUG_PORT.print( ',' );

    // DEBUG_PORT.print( fix.latitude(), 6 ); // floating-point display
    // DEBUG_PORT.print( fix.latitudeL() ); // integer display
    printL( DEBUG_PORT, fix.latitudeL() ); // prints int like a float
    DEBUG_PORT.print( ',' );
    // DEBUG_PORT.print( fix.longitude(), 6 ); // floating-point display
    // DEBUG_PORT.print( fix.longitudeL() );  // integer display
    printL( DEBUG_PORT, fix.longitudeL() ); // prints int like a float

    DEBUG_PORT.print( ',' );
    if (fix.valid.satellites)
      DEBUG_PORT.print( fix.satellites );

    DEBUG_PORT.print( ',' );
    DEBUG_PORT.print( fix.speed(), 6 );
    DEBUG_PORT.print( F(" kn = ") );
    DEBUG_PORT.print( fix.speed_mph(), 6 );
    DEBUG_PORT.print( F(" mph") );

  } else {
    // No valid location data yet!
    DEBUG_PORT.print( '?' );
  }

  DEBUG_PORT.println();

} // doSomeWork

//------------------------------------

static void GPSloop();
static void GPSloop()
{
  while (gps.available( gpsPort ))
    doSomeWork( gps.read() );

}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "gps_raw", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "gps_raw_Pub"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;


    DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("NMEAloc.INO: started\n") );
  DEBUG_PORT.print( F("fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("NMEAGPS object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifdef NMEAGPS_NO_MERGING
    DEBUG_PORT.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif

  DEBUG_PORT.flush();

  gpsPort.begin(9600);


}

void loop() {
  delay(100);
  GPSloop();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
