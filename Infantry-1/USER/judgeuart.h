#include "stm32f4xx.h" 

#define judge_bux_rx 200
#define	HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define SOF_FIXED    0xA5    

typedef enum
{
  GAME_INFO_ID       = 0x0001,  //10Hz
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
  REAL_FIELD_DATA_ID = 0x0005,  //10hZ
  GAME_RESULT_ID     = 0x0006,
  GAIN_BUFF_ID       = 0x0007,
  UWB_GAIN_ID        = 0x0008,
  STU_CUSTOM_DATA_ID = 0x0100,
  ROBOT_TO_CLIENT_ID = 0x0101,
  CLIENT_TO_ROBOT_ID = 0x0102,
} judge_data_id_e;


/** 
  * @brief  GPS state structures definition
  */
typedef __packed struct
{
  uint8_t valid_flag;
  float x;
  float y;
  float z;
  float yaw;
} position_t;

/** 
  * @brief  game information structures definition(0x0001)
  *         this package send frequency is 50Hz
  */
typedef __packed struct
{
	uint16_t stageRemianTime;
	uint8_t  gameProgress;
	uint8_t  reserved;
	uint16_t remainHP;
	uint16_t maxHP;
} game_robot_state_t;

/** 
  * @brief  real time blood volume change data(0x0002)
  */
typedef __packed struct
{
  uint8_t armor_type:4;
 /* 0-3bits: the attacked armor id:
    0x00: 0 front
    0x01:1 left
    0x02:2 behind
    0x03:3 right
    others reserved*/
  uint8_t hurt_type:4;
 /* 4-7bits: blood volume change type
    0x00: armor attacked
    0x01:module offline
    0x02: bullet over speed
    0x03: bullet over frequency */
} robot_hurt_data_t;

/** 
  * @brief  real time shooting data(0x0003)
  */
typedef __packed struct
{
	uint8_t reserved;
	uint8_t bulletFreq;
	float bulletSpeed
} real_shoot_t;

/** 
  * @brief  rfid detect data(0x0005)
  */
typedef __packed struct
{
	uint8_t cardType;
	uint8_t cardIdx
} rfid_detect_t;

/** 
  * @brief  game result data(0x0006)
  */
typedef __packed struct
{
  uint8_t winner;
} game_result_t;

/** 
  * @brief  the data of get field buff(0x0007)
  */
typedef __packed struct
{
	uint16_t buffMusk;
} get_buff_t;

/** 
  * @brief  the direction of robot(0x0008)
  */
typedef __packed struct
{
	float x;
	float y;
	float z;
	float reserved;
}extGameRobotPos_t;


/** 
  * @brief  student custom data
  */
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
} client_show_data_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  game_robot_state_t game_information;
  robot_hurt_data_t  blood_changed_data;
  real_shoot_t       real_shoot_data;
  rfid_detect_t      rfid_data;
  game_result_t      game_result_data;
  get_buff_t         get_buff_data;
	extGameRobotPos_t  game_robot_postion_data;
  server_to_user_t   student_download_data;
} receive_judge_t;


/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;


void usart6_config(void);
void judgement_data_handle(void);

