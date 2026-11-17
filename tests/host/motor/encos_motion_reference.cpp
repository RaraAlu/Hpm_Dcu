// Frozen motion codec from 680d62e. Keep independent of the production codec
// and its conversion helpers; this is an oracle, not a second implementation to maintain.
#include "encos_motion_reference.hpp"
#include <limits.h>
#include <math.h>
#include <string.h>

namespace legacy_reference {
static constexpr double pi = 3.14159265358979323846;
static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]<<8 | p[1]); }
static void put16(uint8_t *p, uint16_t v) { p[0]=v>>8; p[1]=v; }
static void put32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
static uint32_t float_bits(float f) { uint32_t u; memcpy(&u,&f,4); return u; }
static bool quantize(double x, const double r[2], unsigned bits, uint32_t *v)
{
    if(!isfinite(x) || r[0]>=r[1] || x<r[0] || x>r[1]) return false;
    *v=(uint32_t)((x-r[0])*((1u<<bits)-1)/(r[1]-r[0])); return true;
}
static double decode(uint32_t v,const double r[2],unsigned bits) { return r[0]+v*(r[1]-r[0])/((1u<<bits)-1); }
bool encode(const mc_axis_rx_t *c, const mc_routing_t *r, const encos_profile_t *p, encos_frame_t *f)
{
    double pos=c->TargetPosition/1e6, vel=c->TargetVelocity/1e6;
    if(c->Mode<1 || c->Mode>5 || c->Flags || (c->ControlWord&~15u) || r->CanId<1 || r->CanId>8) return false;
    if((c->Mode==1||c->Mode==4) && (c->TargetPosition<r->PositionLimitMin || c->TargetPosition>r->PositionLimitMax)) return false;
    if(fabs((double)c->TargetVelocity)>r->VelocityLimit || fabs((double)c->TargetCurrent)>r->CurrentLimit*10.0 || fabs((double)c->TargetTorque)>r->TorqueLimit) return false;
    if(c->Mode!=1 && c->Mode!=4 && c->TargetPosition) return false;
    if(c->Mode!=1 && c->Mode!=2 && c->Mode!=4 && c->TargetVelocity) return false;
    if(c->Mode!=3 && c->TargetCurrent) return false;
    if(c->Mode!=4 && c->Mode!=5 && c->TargetTorque) return false;
    if(c->Mode!=4 && (c->Kp||c->Kd)) return false;
    if(c->Mode==1||c->Mode==2) { if(!c->CurrentLimit || c->CurrentLimit>r->CurrentLimit) return false; }
    else if(c->CurrentLimit) return false;
    memset(f,0,sizeof(*f)); f->id=r->CanId;
    switch(c->Mode) {
    case 1: {
        double rpm10=vel*60/(2*pi)*10;
        if(rpm10<0 || rpm10>32767 || c->CurrentLimit>4095) return false;
        uint64_t bits=(UINT64_C(1)<<61) | ((uint64_t)float_bits((float)(pos*180/pi))<<29)
            | ((uint64_t)(uint32_t)rpm10<<14) | ((uint64_t)c->CurrentLimit<<2) | 1;
        f->dlc=8; for(unsigned i=0;i<8;i++) f->data[i]=(uint8_t)(bits>>(56-8*i)); break;
    }
    case 2:
        f->dlc=7; f->data[0]=0x41; put32(f->data+1,float_bits((float)(vel*60/(2*pi)))); put16(f->data+5,c->CurrentLimit); break;
    case 3: case 5: {
        int32_t value=c->Mode==3?c->TargetCurrent:c->TargetTorque/10;
        if(value<INT16_MIN || value>INT16_MAX) return false;
        f->dlc=3; f->data[0]=c->Mode==3?0x61:0x65;put16(f->data+1,(uint16_t)value);break;
    }
    case 4: {
        uint32_t kp,kd,x,v,t;
        if(!quantize(c->Kp/100.,p->kp,12,&kp)||!quantize(c->Kd/1000.,p->kd,9,&kd)
            ||!quantize(pos,p->pos,16,&x)||!quantize(vel,p->vel,12,&v)||!quantize(c->TargetTorque/1000.,p->torque,12,&t)) return false;
        uint64_t bits=(uint64_t)kp<<49 | (uint64_t)kd<<40 | (uint64_t)x<<24 | (uint64_t)v<<12 | t;
        f->dlc=8;for(unsigned i=0;i<8;i++)f->data[i]=(uint8_t)(bits>>(56-8*i));break;
    }
    default:return false;
    }
    return true;
}
bool feedback(const encos_frame_t *f,const encos_profile_t *p,mc_axis_tx_t *s)
{
    if(f->dlc!=8 || (f->data[0]>>5)!=1 || (f->data[0]&31)>7) return false;
    const uint8_t *d=f->data;
    s->ActualPosition=(int32_t)(decode(be16(d+1),p->pos,16)*1e6);
    s->ActualVelocity=(int32_t)(decode((uint32_t)d[3]<<4 | d[4]>>4,p->vel,12)*1e6);
    double current=decode((uint32_t)(d[4]&15)<<8 | d[5],p->current,12)*100;
    if(current<INT16_MIN || current>INT16_MAX) return false;
    s->ActualCurrent=(int16_t)current;
    double torque=s->ActualCurrent*p->kt*10;
    s->EstimatedTorque=(torque>=INT32_MIN && torque<=INT32_MAX)?(int32_t)torque:0;
    s->MotorTemperature=((int)d[6]-50)*5; s->MosTemperature=((int)d[7]-50)*5;
    s->MotorFaultRaw=d[0]&31; s->ValidFlags=(s->ValidFlags&64)|0x37;
    if(p->kt>0 && torque>=INT32_MIN && torque<=INT32_MAX)s->ValidFlags|=8;
    return true;
}
}
