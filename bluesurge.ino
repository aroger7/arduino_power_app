#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <Time.h>

//Incoming packet types
#define ENABLE_RELAY   16
#define DISABLE_RELAY   8
#define UPDATE_TIME     4
#define SET_ALARM       2
#define CONNECT         1

//Outgoing packet types
#define STATS_REPORT    64
#define CONNECT_CONF    32

#define RELAY_DISABLED  0
#define RELAY_ENABLED   1

#define SAMPLE_DISTANCE       850
#define INCOMING_PACKET_SIZE  101
#define OUTGOING_PACKET_SIZE  5

struct packet_in {
  char packet_type;
  char username[25];
  char password[25];
  char user_to_add[25];
  char pass_to_add[25];
  time_t device_time;
  time_t alarm_time;
  byte alarm_enable;
};

char incoming[INCOMING_PACKET_SIZE];
int in_count = 0;
int pin7 = 7;
int pin8 = 8;
int alarm_set = 0;
byte relay_status = 0;
SoftwareSerial BTserial(2,3);

int unpack_buffer(char buffer[], struct packet_in * pkt) {
  memset(pkt, 0, sizeof(packet_in));
  int i;
  pkt->packet_type = buffer[0];
  long alarm_time = 0;
  long device_time = 0;
  for(i = 0; i < 25; i++) {
   if( buffer[i+1] != 0 ) {
     pkt->username[i] = buffer[i+1];
   }
   if( buffer[i+26] != 0 ) {
     pkt->password[i] = buffer[i+26];
   }
   if( pkt->packet_type == UPDATE_TIME ) {
     if( i < 4 ) {
       byte b = buffer[i+51];
       device_time = b;
       pkt->device_time += device_time << (i*8);
     }
   } else if( pkt->packet_type == SET_ALARM ) {
     if( i < 4 ) {
       byte b = buffer[i+51];
       alarm_time = b;
       pkt->alarm_time += alarm_time << (i*8);
     }
     pkt->alarm_enable = buffer[55];
   } else {
       if( buffer[i+51] != 0 ) {
         pkt->user_to_add[i] = buffer[i+51];
       }
       if( buffer[i+76] != 0 ) {
         pkt->pass_to_add[i] = buffer[i+76];
       }
   }
  }
  return 0;
}

int relay_enable(boolean enabled) {
  if(enabled) {
    relay_status = RELAY_ENABLED;
    digitalWrite(pin7, HIGH);
    digitalWrite(pin8, LOW);
  } else {
    relay_status = RELAY_DISABLED;
    digitalWrite(pin7, LOW);
    digitalWrite(pin8, HIGH);
  }
  delay(100);
  digitalWrite(pin7, LOW);
  digitalWrite(pin8, LOW);
}

int check_login(char * username, char* password) {
  int i, match;
  char admin_user[6];
  char admin_pass[6];
  for(i = 0; i < 6; i++) {
    admin_user[i] = EEPROM.read(i);
    admin_pass[i] = EEPROM.read(i+6);
  }
  if(strcmp(admin_user, username) == 0 && 
     strcmp(admin_pass, password) == 0 ) {
       return 1;
  }
  return 0;
}

void check_alarm_time() {
  time_t t = 0;
  int i;
  for(i = 0; i < 4; i++) {
    byte read_byte = EEPROM.read(900+i);
    long read_int = read_byte;
    t += read_int << (i*8);
  }
    
   if(now() >= t) {
     if( EEPROM.read(904) == 1) {
       relay_enable(true);
     } else {
       relay_enable(false);
     }
     alarm_set = 0;
   }
}

float squared_current_sample_sum = 0;
float avg_power_sum = 0;
int num_samples_taken = 0;
int num_cycles = 0;
unsigned long begin_time = 0;
unsigned long end_time = 0;
float acs_ref_volt = 0;

boolean extra_bytes = false;

void setup() {
  Serial.begin(115200);
  BTserial.begin(9600);
  pinMode(pin7, OUTPUT);
  pinMode(pin8, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  relay_enable(false);
  //calibrate ACS712 zero current reference voltage
  float calib_sample_sum = 0;
  int num_calib_samples = 0;
  for(num_samples_taken = 0; num_samples_taken < 100; num_samples_taken++) {
    int sensor = analogRead(A2);
    float calib_sample = sensor * (5.0 / 1023.0);
    calib_sample_sum += calib_sample;
    num_calib_samples++;
    //Serial.println(calib_sample);
    delayMicroseconds(SAMPLE_DISTANCE);
  }
  acs_ref_volt = calib_sample_sum/(float)num_calib_samples;
  Serial.println(acs_ref_volt);
}

void loop() {
  int i = 0;
  
  int sensor = 0;   
  //take 16 current samples spread evenly across approximate 16.5 ms period (60 Hz)
  for(num_samples_taken = 0; num_samples_taken < 16; num_samples_taken++) {
    sensor = analogRead(A2);
    float current_sample = (acs_ref_volt - (sensor * (5.0 / 1023.0))) / 0.100;
    squared_current_sample_sum += current_sample * current_sample;
    delayMicroseconds(SAMPLE_DISTANCE);
  }
  
  double rms_current = sqrt(squared_current_sample_sum / num_samples_taken);
  sensor = analogRead(A1);
  float rms_voltage = ((sensor * (41.1 / 1023.0))) * 0.707;
  /*Serial.print("rms current is ");
  Serial.println(rms_current);
  Serial.print("rms_voltage is ");
  Serial.println(rms_voltage);*/
  num_cycles++;
  avg_power_sum += (rms_current * rms_voltage);
  if( num_cycles == 60 ) {
    float avg_power = avg_power_sum / num_cycles;
    int avg_power_int = (int)avg_power;
    int avg_power_frac = (avg_power - (int)avg_power) * 100;
    byte buf[5];
    buf[0] = STATS_REPORT;
    buf[1] = avg_power_int;
    buf[2] = avg_power_int >> 8;
    buf[3] = avg_power_frac;
    buf[4] = avg_power_frac >> 8;
    int write_bytes = BTserial.write(buf, 5);
    avg_power_sum = 0;
    num_cycles = 0;
  }
  num_samples_taken = 0;
  squared_current_sample_sum = 0;
 
  if(alarm_set == 1) {
    check_alarm_time();
  }
   
  if(extra_bytes) {
    BTserial.read();
    extra_bytes = false;
  }
  
  if(BTserial.available() == INCOMING_PACKET_SIZE) {
    Serial.println(BTserial.available());
    while(in_count < INCOMING_PACKET_SIZE) {
      incoming[in_count] = BTserial.read();
      in_count++;    
    }
  }
  
  if(in_count == INCOMING_PACKET_SIZE) {
    in_count = 0;
    packet_in pkt;
    unpack_buffer(incoming, &pkt);
    /*Serial.print("Username is ");
    for(count = 0; count < 25; count++) {
      Serial.print(pkt.username[count]);
    }
    Serial.println("");
    Serial.print("Password is ");
    for(count = 0; count < 25; count++) {
      Serial.print(pkt.password[count]);
    }
    Serial.println("");*/
    if(check_login(pkt.username, pkt.password) == 1) {
      switch(pkt.packet_type) {
        case CONNECT:
          byte buf[5];
          buf[0] = CONNECT_CONF;
          buf[1] = 1;
          buf[2] = relay_status;
          buf[3] = 0;
          buf[4] = 0;
          BTserial.write(buf, 5);
          break;        
        case ENABLE_RELAY:
          relay_enable(true);
          Serial.println("Device is turned on");
          break;
        case DISABLE_RELAY:
          relay_enable(false);
          Serial.println("Device is turned off");
          break;
        case UPDATE_TIME:
          setTime(pkt.device_time);
          break;      
        case SET_ALARM:
          if(pkt.alarm_time > now()) {
            for(i = 0; i < 4; i++) {
              EEPROM.write(900 + i, pkt.alarm_time >> (i*8));
            }
            EEPROM.write(904, pkt.alarm_enable);
            alarm_set = 1;
          } else {
            Serial.println("specified alarm time has already passed");
          }          
        default:
          digitalWrite(pin7, LOW);
          digitalWrite(pin8, LOW);
     }
    }
  
    if(BTserial.available() > 0) {
      extra_bytes = true;
    }
    
    for(i = 0; i < INCOMING_PACKET_SIZE; i++) {
      incoming[i] = 0; //flush incoming buffer
    }

  }
}
