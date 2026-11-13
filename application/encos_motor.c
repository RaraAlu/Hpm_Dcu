#include "encos_motor.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
uint16_t encos_be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]<<8 | p[1]); }
uint32_t encos_be32(const uint8_t *p) { return (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3]; }
static void put16(uint8_t *p, uint16_t v) { p[0]=v>>8; p[1]=v; }
static void put32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
static uint32_t float_bits(float f) { uint32_t u; memcpy(&u,&f,4); return u; }
bool encos_float(const uint8_t *p, double scale, int32_t *v)
{
    uint32_t u=encos_be32(p); float f; memcpy(&f,&u,4); double x=f*scale;
    if(!isfinite(x) || x<INT32_MIN || x>INT32_MAX) return false;
    *v=(int32_t)x; return true;
}
bool encos_profile(const mc_range_t *r, const mc_basic_t *b, encos_profile_t *p)
{
    *p=(encos_profile_t){{r->KpMin,r->KpMax},{r->KdMin,r->KdMax},
        {r->PosMinRaw/100.,r->PosMaxRaw/100.},{r->SpdMinRaw/100.,r->SpdMaxRaw/100.},
        {r->TorMinRaw/10.,r->TorMaxRaw/10.},{r->CurMinRaw/10.,r->CurMaxRaw/10.},b->TorqueConstantRaw/100.};
    return p->kp[0]<p->kp[1] && p->kd[0]<p->kd[1] && p->pos[0]<0 && p->pos[1]>0
        && p->vel[0]<0 && p->vel[1]>0 && p->torque[0]<0 && p->torque[1]>0
        && p->current[0]<0 && p->current[1]>0 && b->CanTimeoutMs>0;
}
static bool quantize(double x, const double r[2], unsigned bits, uint32_t *v)
{
    if(!isfinite(x) || r[0]>=r[1] || x<r[0] || x>r[1]) return false;
    *v=(uint32_t)((x-r[0])*((1u<<bits)-1)/(r[1]-r[0])); return true;
}
static double decode(uint32_t v,const double r[2],unsigned bits) { return r[0]+v*(r[1]-r[0])/((1u<<bits)-1); }
bool encos_encode(const mc_axis_rx_t *c, const mc_routing_t *r, const encos_profile_t *p, encos_frame_t *f)
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
        double rpm10=vel*60/(2*PI)*10;
        if(rpm10<0 || rpm10>32767 || c->CurrentLimit>4095) return false;
        uint64_t bits=(UINT64_C(1)<<61) | ((uint64_t)float_bits((float)(pos*180/PI))<<29)
            | ((uint64_t)(uint32_t)rpm10<<14) | ((uint64_t)c->CurrentLimit<<2) | 1;
        f->dlc=8; for(unsigned i=0;i<8;i++) f->data[i]=(uint8_t)(bits>>(56-8*i)); break;
    }
    case 2:
        f->dlc=7; f->data[0]=0x41; put32(f->data+1,float_bits((float)(vel*60/(2*PI)))); put16(f->data+5,c->CurrentLimit); break;
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
bool encos_feedback(const encos_frame_t *f,const encos_profile_t *p,mc_axis_tx_t *s)
{
    if(f->dlc!=8 || (f->data[0]>>5)!=1 || (f->data[0]&31)>7) return false;
    const uint8_t *d=f->data;
    s->ActualPosition=(int32_t)(decode(encos_be16(d+1),p->pos,16)*1e6);
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
uint8_t encos_parameter_query(uint16_t p)
{
    static const uint8_t q[]={0,5,0,22,23,24,25,26,27,28,31,32,33,34,35,36,38};
    return p<sizeof(q)?q[p]:(p>=0x101 && p<=0x127?(uint8_t)(p-0x100):0);
}
bool encos_query(uint8_t code,uint8_t group,encos_frame_t *f)
{
    if(!(code>=1 && code<=5) && !(code>=22 && code<=39))return false;
    if(code==36 && (group<1||group>10))return false;
    f->dlc=2; memset(f->data,0,8); f->data[0]=0xE0|(code==36?group:0); f->data[1]=code;return true;
}
bool encos_parameter(uint16_t id,const mc_basic_t *b,const mc_range_t *r,const mc_gain_t *g,const mc_kt_t *k,encos_frame_t *f)
{
    static const uint8_t code[]={0,1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    if(id<1||id>15 || id==1) return false; /* Acceleration ACK table conflicts with its example; disabled pending confirmation. */
    memset(f->data,0,8);f->data[0]=0xC0;f->data[1]=code[id];f->dlc=4;
    if(id==2){if(b->CommunicationMode>1)return false;f->data[0]=0xC1;f->data[2]=b->CommunicationMode;f->dlc=3;}
    else if(id==3){if(!b->TorqueConstantRaw)return false;put16(f->data+2,b->TorqueConstantRaw);}
    else if(id>=4 && id<=9){uint16_t v[2];memcpy(v,(const uint8_t *)r+(id-4)*4,4);put16(f->data+2,v[0]);put16(f->data+4,v[1]);f->dlc=6;}
    else if(id==10){if(!b->CanTimeoutMs)return false;put16(f->data+2,b->CanTimeoutMs);}
    else if(id>=11 && id<=13){uint16_t v[2];memcpy(v,(const uint8_t *)g+(id-11)*4,4);put16(f->data+2,v[0]);put16(f->data+4,v[1]);f->dlc=6;}
    else if(id==14){if(k->CalibrationMode>2)return false;f->data[2]=k->CalibrationMode;f->dlc=3;}
    else if(id==15){if(k->Group<1||k->Group>10||k->Kt1Raw>4095||k->Kt2Raw>4095||k->Kt3Raw>4095||k->Kt4Raw>4095)return false;
        f->data[0]|=k->Group;uint64_t bits=(uint64_t)k->Kt1Raw<<36|(uint64_t)k->Kt2Raw<<24|(uint64_t)k->Kt3Raw<<12|k->Kt4Raw;
        for(unsigned i=0;i<6;i++) {
            f->data[i+2]=(uint8_t)(bits>>(40-8*i));
        }
        f->dlc=8;}
    return true;
}
