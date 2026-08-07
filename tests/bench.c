#include "dz.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// deterministic action stream, independent of env->rng
static unsigned int lcg = 12345;
static float ra(){ lcg = lcg*1103515245u + 12345u; return ((lcg>>9)&0x7fffff)/8388607.0f*2.0f-1.0f; }

static double checksum(SoilEnv* e){
  double s=0; Dozer*d=&e->dozer;
  for(int i=0;i<GRID_SIZE;i++)for(int j=0;j<GRID_SIZE;j++){ s+=e->grid_H[i][j]*1.0+e->grid_L[i][j]*3.0+e->grid_G[i][j]*0.5; }
  s+=d->position_x+d->position_y+d->position_z+d->blade_x+d->blade_y+d->blade_z;
  s+=d->twist_linear_x+d->twist_angular_z+d->last_force+d->vel_tracks_linear+d->vel_tracks_rotational;
  s+=d->pos_virtual_lift_arm+d->pos_blade_pitch+d->pos_blade_roll;
  return s;
}

int main(int argc,char**argv){
  int N = argc>1?atoi(argv[1]):3000000;
  SoilEnv* e=calloc(1,sizeof(SoilEnv));
  e->observations=calloc(5011,sizeof(float));
  e->actions=calloc(6,sizeof(float));
  e->rewards=calloc(1,sizeof(float));
  e->terminals=calloc(1,sizeof(float));
  e->rng=42; c_reset(e);
  double cs=0;
  struct timespec t0,t1; clock_gettime(CLOCK_MONOTONIC,&t0);
  for(int k=0;k<N;k++){
    e->actions[0]=0.8f;               // drive forward
    e->actions[1]=ra()*0.3f;          // gentle turns
    e->actions[2]=-0.5f+ra()*0.2f;    // lift down (dig)
    e->actions[3]=ra()*0.4f;          // pitch
    e->actions[4]=ra()*0.2f;          // roll
    c_step(e);
    if((k+1)%256==0){ cs+=checksum(e); e->rng=42+k; c_reset(e); }
  }
  clock_gettime(CLOCK_MONOTONIC,&t1);
  double sec=(t1.tv_sec-t0.tv_sec)+(t1.tv_nsec-t0.tv_nsec)/1e9;
  cs+=checksum(e);
  printf("steps=%d  time=%.3fs  SPS=%.0f  checksum=%.6f\n", N, sec, N/sec, cs);
  return 0;
}
