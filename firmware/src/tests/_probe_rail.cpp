// Throwaway: the ESP measures the board's VCC_3V3 net via GPIO4 (Sense, 1k R14 to VCC_3V3),
// then writes a text report into the spiffs partition so it can be read back with esptool
// (serial on this host resets the chip). Also logs the reset reason and SDA/SCL/INT levels.
#include <Arduino.h>
#include "esp_system.h"
#include "esp_partition.h"
static const char* rr(int r){ switch(r){case 1:return "POWERON";case 3:return "SW";case 4:return "PANIC";case 5:return "INT_WDT";case 6:return "TASK_WDT";case 7:return "WDT";case 8:return "DEEPSLEEP";case 9:return "BROWNOUT";case 10:return "SDIO";case 11:return "USB";case 12:return "JTAG";default:return "?";}}
static char rep[1024]; static int n=0;
static void P(const char* f, ...){ va_list a; va_start(a,f); n+=vsnprintf(rep+n,sizeof(rep)-n,f,a); va_end(a); }
static const esp_partition_t* part(){ const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "spiffs"); return p; }
static void stamp(const char* txt){ const esp_partition_t* p=part(); if(!p) return; char b[256]; int l=snprintf(b,sizeof b,"%s",txt); esp_partition_erase_range(p,0,4096); esp_partition_write(p,0,b,(l+4)&~3); }
void setup(){
  { char b[128]; snprintf(b,sizeof b,"STAGE1 boot reached, reset=%d(%s)\n",(int)esp_reset_reason(),rr(esp_reset_reason())); stamp(b); }
  Serial.begin(115200);
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  pinMode(1,INPUT); pinMode(2,INPUT); pinMode(3,INPUT); pinMode(4,INPUT);
  delay(300);
  P("RAILPROBE reset=%d(%s) uptime_ms=%lu\n", (int)esp_reset_reason(), rr(esp_reset_reason()), millis());
  for (int rep_i=0; rep_i<3; rep_i++){
    const int N=3000; uint32_t mn=9999,mx=0,sum=0,hi=0; char tr[41]; int per=N/40;
    uint32_t t0=micros();
    for(int i=0;i<N;i++){ uint32_t v=analogReadMilliVolts(4); sum+=v; if(v<mn)mn=v; if(v>mx)mx=v; if(v>2500)hi++;
      if(i%per==0 && i/per<40) tr[i/per]= v>2500?'H':(v<800?'_':'m'); }
    tr[40]=0; uint32_t dt=micros()-t0;
    uint32_t sda=0,scl=0,it=0; for(int i=0;i<1000;i++){ sda+=digitalRead(1); scl+=digitalRead(2); it+=digitalRead(3); }
    P("t=%lums VCC_3V3@GPIO4 min=%lu max=%lu mean=%lu mV above2.5V=%lu%% win=%lums %s SDAhi=%lu%% SCLhi=%lu%% INThi=%lu%%\n",
      millis(), mn, mx, sum/N, hi*100/N, dt/1000, tr, sda/10, scl/10, it/10);
    delay(700);
  }
  P("END\n");
  const esp_partition_t* p = part();
  if (p) { esp_partition_erase_range(p, 0, 4096); esp_partition_write(p, 0, rep, (n+3)&~3); P("(written to %s @0x%lx)\n", p->label, (unsigned long)p->address); }
  else P("(no spiffs partition)\n");
  Serial.print(rep);
}
void loop(){ Serial.print(rep); delay(2000); }
