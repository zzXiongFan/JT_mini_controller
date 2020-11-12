#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_bus.h"



canbus::canbus(const char *iface) : ifname(iface), can_sockfd(-1) {}
canbus::~canbus() {
  if (can_sockfd != -1)
    close(can_sockfd);
    is_open = false;
}

void* canbus::start_thread(void *arg)
{
    canbus *ptr = (canbus *)arg;
    //ptr->begin();
    if(!(prt->begin())){
        printf("canbus begin failed");
        //return 1;
    }
    ptr->data_explain();
}

int canbus::start()
{
    if(pthread_create(&pid,NULL,start_thread,(void *)this) != 0) //重点注意下结果，原程序选择将start_thread设置为静态函数，因此可能需要全局声明
    {
        return -1;
    }
    return 0;
}


const char *debug_lvls[] = {"", "ERROR", "WARN", "INFO"};

template<int level>
void canbus::canbus_log(const char *msg) {
  if (level >= 1 && level <= 3 && DEBUG_LEVEL >= level) {
    printf("[canbus] [%s]: %s\n", debug_lvls[level], msg);
  }
}

void canbus::canbus_errno(const char *msg) {
  if (DEBUG_LEVEL >= CANBUS_ERR) {
    printf("[canbus] [ERROR] [syscall] %s: %s\n", msg, strerror(errno));
  }
}

bool canbus::begin() {
  struct ifreq ifr = {};
  struct sockaddr_can addr;

  if ((can_sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    canbus_errno("socket open failed");
    is_open = false;
    return false;
  }else{
    is_open = true;
  }
  

  strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
  if (ioctl(can_sockfd, SIOCGIFINDEX, &ifr) < 0) {
    canbus_errno("ioctl get interface index failed");
    return false;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_sockfd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    canbus_errno("bind to socket failed");
    return false;
  }

  canbus_log<CANBUS_INFO>("socket opened");

  return true;
}

bool canbus::send(uint32_t id, uint8_t *arr, uint8_t len, bool id_ext) {
  if (len > CAN_MAX_DLEN) {
    canbus_log<CANBUS_ERR>("length > CAN MTU");
    return false;
  }
  if (!check_id(id, id_ext)) {
    canbus_log<CANBUS_ERR>("id invalid");
    return false;
  }

  can_frame frame = {};
  frame.can_id = id;
  if (id_ext) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  frame.can_dlc = len;
  memcpy(frame.data, arr, len);

  int ret;
  do {
    ret = write(can_sockfd, &frame, sizeof(can_frame));
  } while (ret < 0 && errno == ENOBUFS);

  if (ret < 0) {
    canbus_errno("write failed");
    return false;
  }
  if (ret != sizeof(can_frame)) {
    canbus_log<CANBUS_ERR>("incomplete write");
    return false;
  }

  return true;
}

bool canbus::request(uint32_t id, uint8_t len, bool id_ext) {
  if (len > CAN_MAX_DLEN) {
    canbus_log<CANBUS_ERR>("length > CAN MTU");
    return false;
  }

  if (!check_id(id, id_ext)) {
    canbus_log<CANBUS_ERR>("id invalid");
    return false;
  }

  can_frame frame = {};
  frame.can_id = id | CAN_RTR_FLAG;
  if (id_ext) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  frame.can_dlc = len;

  int ret;
  do {
    ret = write(can_sockfd, &frame, sizeof(can_frame));
  } while (ret < 0 && errno == ENOBUFS);

  if (ret < 0) {
    canbus_errno("write failed");
    return false;
  }
  if (ret != sizeof(can_frame)) {
    canbus_log<CANBUS_ERR>("incomplete write");
    return false;
  }
  return true;
}

bool canbus::filter(can_filter *filters, size_t nfilters) {
  if (setsockopt(can_sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, nfilters * sizeof(can_filter)) < 0) {
    canbus_errno("setsockopt filter failed");
    return false;
  }

  canbus_log<CANBUS_INFO>("Filter changed");

  return true;
}

bool canbus::recv(uint32_t *id, uint8_t *data, uint8_t *len, bool *remote_req, bool *id_ext) {
  can_frame frame;
  int ret = read(can_sockfd, &frame, sizeof(can_frame));
  if (ret < 0) {
    canbus_errno("read failed");
    return false;
  }
  if (ret != sizeof(can_frame)) {
    canbus_log<CANBUS_ERR>("incomplete read");
    return false;
  }

  bool req = frame.can_id & CAN_RTR_FLAG;
  bool ext = frame.can_id & CAN_EFF_FLAG;

  if (id != NULL)
    *id = frame.can_id & CAN_ERR_MASK;

  if (!req && data != NULL)
    memcpy(data, frame.data, frame.can_dlc);

  if (len != NULL)
    *len = frame.can_dlc;

  if (remote_req != NULL)
    *remote_req = req;

  if (id_ext != NULL)
    *id_ext = ext;

  return true;
}

bool canbus::check_id(uint32_t id, bool id_ext) {
  if (id_ext) {
    return !(id & ~CAN_EFF_MASK);
  } else {
    return !(id & ~CAN_SFF_MASK);
  }
}

void canbus::data_explain()
{

    unsigned char data[2] = {0};
    int ret;
    unsigned char buff[23] = {0};
    unsigned char sum = 0x00;
    int left_encoder = 0;
    int right_encoder = 0;
    int left_current = 0;
    int right_current = 0;
    int left_speed = 0;
    int right_speed = 0;
    int voltage = 0;

    while(is_open)
    {
        sum = 0x00;
        ret = recv(&id,1);
        if(-1 == ret)
        {
            printf("read error \n");
        }else if(data[0] == 0x68)
        {
            ret = uart_recv(data,1);
            if(-1 == ret)
            {
                printf("read error \n");
            }else if (data[0] == 0x24) {
                ret = uart_recv(buff,23);
                if(-1 == ret)
                {
                    printf("read error \n");
                }else{
                    for(int i=0;i<22;i++)
                    {
                        sum = sum + buff[i];
                    }
//                    printf("%x \n",sum);
//                    printSerial(buff,23);
                    if(sum == buff[22])
                    {
//                        printf("right data \n");
                        left_encoder = buff[4] + buff[3] * 256 + buff[2]*256*256 + buff[1]*256*256*256;
                        right_encoder = buff[9] + buff[8] * 256 + buff[7]*256*256 + buff[6]*256*256*256;
                        if(buff[0] == 0x00)
                        {
                            left_encoder = - left_encoder;
                        }
                        if(buff[5] == 0x00)
                        {
                            right_encoder = - right_encoder;
                        }
                        left_speed = buff[12] + buff[11] * 256;
                        right_speed = buff[15] + buff[14] * 256;
                        if(buff[10] == 0x00)
                        {
                            left_speed = - left_speed;
                        }
                        if(buff[13] == 0x00)
                        {
                            right_speed = - right_speed;
                        }
                        left_current = buff[18] + buff[17] * 256;
                        right_current = buff[21] + buff[20] * 256;
                        if(buff[16] == 0x00)
                        {
                            left_current = - left_current;
                        }
                        if(buff[19] == 0x00)
                        {
                            right_current = - right_current;
                        }
                        if(first_left_encoder == 0 && first_right_encoder == 0){
                            first_left_encoder = left_encoder;
                            first_right_encoder = right_encoder;
                        }
                        information.left_encoder = left_encoder - first_left_encoder;
                        information.right_encoder = right_encoder - first_right_encoder;
                        information.left_electric = left_current;
                        information.right_electric = right_current;
                        information.left_speed = left_speed;
                        information.right_speed = right_speed;
//                        cout<<information.left_encoder<<" "<<information.right_encoder<<endl;
//                        cout<<information.left_encoder<<" "<<information.right_encoder<<" "<<information.left_speed<<" "<<information.right_speed
//                           <<" "<<information.left_electric<<" "<<information.right_electric<<endl;
                    }
                }
            }

        }
    }
}