#ifndef CAN_H
#define CAN_H

#define CANBUS_ERR 1
#define CANBUS_WARN 2
#define CANBUS_INFO 3

// TODO allow DEBUG_LEVEL to be configured outside of library
#if !defined(DEBUG_LEVEL)
#define DEBUG_LEVEL CANBUS_INFO
#endif

#include <stdint.h>
#include <linux/can.h>
#include <pthread.h>

typedef struct
{
  /**encoders**/
    //double left_encoder;
    //double right_encoder;
    /**转速**/
    int left_speed; // r/min
    int right_speed;
    /**左右电机母线电流**/
    //int left_electric; // ma
    //int right_electric;
    /**电池电压**/
    int power;
} robot_info;

class canbus {
 public:
  robot_info information;
  
  canbus(const char *ifname);
  ~canbus();

  //void* start_thread(void *arg);

  int start();

  // Initialize
  bool begin();

  // Send a CAN frame
  bool send(uint32_t id, uint8_t *arr, uint8_t len, bool id_ext = true);

  // Send a CAN remote request
  bool request(uint32_t id, uint8_t len, bool id_ext = true);

  // Receive a CAN frame
  bool recv(uint32_t *id, uint8_t *data = NULL, uint8_t *len = NULL, bool *remote_req = NULL, bool *id_ext = NULL);

  // Set CAN filter
  bool filter(can_filter *filters, size_t nfilters);
  //Explain the data,get ready to publish
  void data_explain(canbus *can_ptr);
  void start_read_motorinfo();

 private:
  template<int level>
  void canbus_log(const char *msg);

  void canbus_errno(const char *msg);

  bool check_id(uint32_t id, bool id_ext);

  const char *ifname;
  int can_sockfd;
  bool is_open;
  pthread_t pid;
  static void * start_thread(void* arg);
};


#endif