#include "mcu_protocol.h"
#include <stdio.h>
#include <string.h>
static int fails=0;
#define CHK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg);fails++;}else printf("ok: %s\n",msg);}while(0)

static size_t unhex(const char *h, uint8_t *out, size_t max){
  size_t n=0; unsigned v;
  while(n<max && sscanf(h,"%2x",&v)==1){ out[n++]=(uint8_t)v; h+=2; }
  return n;
}
// feed a buffer, return how many status frames decoded (last one left in *st)
static int feed(fc_parser_t *p, const uint8_t *b, size_t n, fc_state_t *st){
  int got=0; for(size_t i=0;i<n;i++) got+=fc_parse_byte(p,b[i],st); return got;
}

// Healthy MCU status reply (type 0x01), from a 2026-11-10 /log capture.
static const char *HEALTHY =
  "5bb5012100000000120009028c0d00000000140601050000"
  "8d190000000100321001080000003" "52f03";
// Broken-link capture from 2026-11-17 /log: every MCU reply is 41 bytes but its first 10
// bytes (5B B5 header, len, temp) are replaced by a copy of the frame tail, so nothing
// decodes — except one reply that went out back-to-back with the previous one and arrived
// intact (the 5bb50f... frame). The parser must find exactly that one and nothing else.
static const char *SHIFTED =
  "10010000000035f600010188100000000014060105000050300000000100321001000000003504033210"
  "010000000035f6000101801000000000140601050000503000000001003210010000000035ff02321001"
  "0000000035f6000101801000000000140601050000503100000001003210010000000035ef0232100100"
  "0000003504000101801000000000140601050000503100000001003210010000000035ef023210010000"
  "000035f6000201801000000000140601050000503100000001003210010000000035f0025bb50f210000"
  "00001100020188100000000014060105000050310000000100321001000000003506033210010000000035f6";

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
  f[0]=0x5B; f[1]=0xB5; f[2]=0x12; f[3]=len&0xff; f[4]=len>>8; f[5]=0;
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

  fc_parser_t p; fc_parser_reset(&p); fc_state_t st;
  int got = feed(&p,f,total,&st)>0;
  CHK(got==1, "status frame parsed");
  CHK(st.power==true, "power on");
  CHK(st.is_heating==true, "is_heating");
  CHK((int)(st.current_temp*10+0.5f)==255, "current_temp=25.5");
  CHK(st.temp_set==22, "temp_set=22");
  CHK(st.fan_speed==2, "fan_speed=2");
  CHK(st.heat_top==30 && st.cool_bot==16 && st.fan_delay==4, "limits decoded");
  CHK(st.reset_provision==false, "no reset flag");

  // 3) reset-bit detection (bit0)
  f[0x0C] |= 1; s=0; for(int i=0;i<6+len;i++) s+=f[i]; f[6+len]=s&0xff; f[7+len]=s>>8;
  fc_parser_reset(&p);
  got = feed(&p,f,total,&st)>0;
  CHK(got&&st.reset_provision==true, "reset_provision detected (bit0)");

  // 4) false start: a stray 5B B5 with a plausible length directly before a real frame.
  //    The old parser swallowed the real frame into the bogus one; this one must not.
  uint8_t g[128]; size_t gn=0;
  const uint8_t junk[] = {0x00,0x5B,0xB5,0x01,0x05,0x00,0x00};   // claims len 5, no cksum
  memcpy(g+gn,junk,sizeof junk); gn+=sizeof junk;
  memcpy(g+gn,f,total); gn+=total;
  fc_parser_reset(&p);
  CHK(feed(&p,g,gn,&st)==1, "real frame recovered after a false-start header");
  CHK(p.bad_ck>=1, "false start counted as bad_ck");

  // 5) real healthy capture decodes (room 18.9, setpoint 13, power on, heating)
  uint8_t h[64]; size_t hn = unhex(HEALTHY,h,sizeof h);
  fc_parser_reset(&p);
  CHK(hn==41 && feed(&p,h,hn,&st)==1, "healthy 41-byte capture decodes");
  CHK((int)(st.current_temp*10+0.5f)==189 && st.temp_set==13 && st.power, "healthy capture fields");

  // 6) the same body with type 0x0F (MCU's reply to a panel-light frame) also decodes
  h[2]=0x0F; s=0; for(int i=0;i<39;i++) s+=h[i]; h[39]=s&0xff; h[40]=s>>8;
  fc_parser_reset(&p);
  CHK(feed(&p,h,hn,&st)==1, "type 0x0F status reply decodes");

  // 7) shifted-reply capture: exactly one (intact) frame, and the rest is counted as skipped
  uint8_t sh[300]; size_t shn = unhex(SHIFTED,sh,sizeof sh);
  fc_parser_reset(&p);
  CHK(feed(&p,sh,shn,&st)==1 && p.frames==1, "shifted capture: only the intact 0x0F reply decodes");
  CHK(p.skipped > 150, "shifted capture: header-less bytes counted as skipped");

  printf(fails? "\n%d FAIL\n":"\nALL PASS\n", fails);
  return fails?1:0;
}
