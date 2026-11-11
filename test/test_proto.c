#include "mcu_protocol.h"
#include <stdio.h>
#include <string.h>
static int fails=0;
#define CHK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg);fails++;}else printf("ok: %s\n",msg);}while(0)

int main(void){
  // 1) encode a set_temp(22) frame and verify framing + checksum
  uint8_t out[64]; size_t n = fc_set_temp(out, 22);
  CHK(n==18, "set_temp frame length = 18 (10 payload + 8)");
  CHK(out[0]==0x5C && out[1]==0xC5 && out[2]==0x12, "magic + type 0x12");
  CHK(out[3]==10 && out[4]==0, "len=10 LE");
  CHK(out[6+8]==22, "payload[8]=temp_set=22");
  uint16_t ck=0; for(int i=0;i<16;i++) ck+=out[i];
  CHK((out[16]|(out[17]<<8))==ck, "checksum LE correct");

  // 2) build a synthetic MCU status frame (type 0x12, payload len 0x1c) and parse it
  uint8_t f[64]; memset(f,0,sizeof f);
  uint16_t len=0x1c;
  f[0]=0x5B; f[1]=0xC5; f[2]=0x12; f[3]=len&0xff; f[4]=len>>8; f[5]=0;
  f[0x08]=25;   // temp int
  f[0x0A]=5;    // temp frac -> 25.5
  f[0x0B]=(2)|(0<<5); // mode: (2)&7=2 -> mode=1 ; sys_lock=0
  f[0x0C]=(1<<7)|(1<<6); // flags: power on + heating
  f[0x0D]=22;   // temp_set
  f[0x1D]=3;    // fan raw -> fan_speed=2
  f[0x1F]=30; f[0x20]=16; f[0x21]=4;
  uint16_t s=0; for(int i=0;i<6+len;i++) s+=f[i];
  f[6+len]=s&0xff; f[7+len]=s>>8;
  uint16_t total=len+8;

  fc_parser_t p; fc_parser_reset(&p); fc_state_t st; int got=0;
  for(uint16_t i=0;i<total;i++){ if(fc_parse_byte(&p,f[i],&st)) got=1; }
  CHK(got==1, "status frame parsed");
  CHK(st.power==true, "power on");
  CHK(st.is_heating==true, "is_heating");
  CHK((int)(st.current_temp*10+0.5f)==255, "current_temp=25.5");
  CHK(st.temp_set==22, "temp_set=22");
  CHK(st.fan_speed==2, "fan_speed=2");
  CHK(st.heat_top==30 && st.cool_bot==16 && st.fan_delay==4, "limits decoded");
  CHK(st.reset_requested==false, "no reset flag");

  // 3) reset-bit detection (bit0)
  f[0x0C] |= 1; s=0; for(int i=0;i<6+len;i++) s+=f[i]; f[6+len]=s&0xff; f[7+len]=s>>8;
  fc_parser_reset(&p); got=0;
  for(uint16_t i=0;i<total;i++){ if(fc_parse_byte(&p,f[i],&st)) got=1; }
  CHK(got&&st.reset_requested==true, "reset_requested detected (bit0)");

  printf(fails? "\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
