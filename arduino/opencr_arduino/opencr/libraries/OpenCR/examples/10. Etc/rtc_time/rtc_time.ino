
typedef struct
{
  uint32_t time_year;
  uint32_t time_mon;
  uint32_t time_day;
  uint32_t time_hour;
  uint32_t time_min;
  uint32_t time_sec;
} rtc_time_t;



bool setRtcTime(uint32_t _year, uint32_t _mon, uint32_t _day, uint32_t _hour, uint32_t _min, uint32_t _sec);
rtc_time_t getRtcTime(void);




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  // if you want to change the time, uncommnet line below.
  //setRtcTime(2018, 3, 5, 13, 22, 1);


}

void loop() {  
  rtc_time_t rtc_time;

  rtc_time = getRtcTime();


  Serial.print(String(rtc_time.time_year));
  Serial.print(String("/") + String(rtc_time.time_mon));
  Serial.print(String("/") + String(rtc_time.time_day));
  Serial.print(String(" ") + String(rtc_time.time_hour));
  Serial.print(String(":") + String(rtc_time.time_min));
  Serial.println(String(":") + String(rtc_time.time_sec));

  delay(1000);
}




rtc_time_t getRtcTime(void)
{
  rtc_time_t ret;
  time_t now;
  struct tm *t;

  now = rtcGetTime();    
  t   = localtime(&now); 

  ret.time_year = t->tm_year + 1900;
  ret.time_mon  = t->tm_mon;
  ret.time_day  = t->tm_mday;
  ret.time_hour = t->tm_hour;
  ret.time_min  = t->tm_min;
  ret.time_sec  = t->tm_sec;  

  return ret;
}


bool setRtcTime(uint32_t _year, uint32_t _mon, uint32_t _day, uint32_t _hour, uint32_t _min, uint32_t _sec)
{
  struct tm time_str;
  time_t time_data;
  
  time_str.tm_year = _year - 1900;
  time_str.tm_mon = _mon;
  time_str.tm_mday = _day;
  time_str.tm_hour = _hour;
  time_str.tm_min = _min;
  time_str.tm_sec = _sec;
  time_str.tm_isdst = -1;

  time_data = mktime(&time_str);
  
  if ( time_data == -1)
  {
    return false;
  }

  rtcSetTime(time_data);
  
  return true;
}

