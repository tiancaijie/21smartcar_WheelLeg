#include "pid.h"
float  err;
float  pid_out = 0;

/*********************
 *
 *  */
void PID_init(PID_HandleTypeDef *pid, PID_InitTypeDef *PID)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->mode                   = PID->mode;//增量式还是位置式啥的
    pid->kp                     = PID->kp;// * PID->factor_kp;
    pid->ki                     = PID->ki;// * PID->factor_ki;
    pid->kd                     = PID->kd;// * PID->factor_kd;
    pid->iOutMax                = PID->iOutMax;//积分限幅
    pid->outMax                 = PID->outMax;//输出限幅
    pid->ISeparation_error_max  = PID->ISeparation_error_max;//积分分离值
}

void PID_loadparam(PID_HandleTypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_loadtarget(PID_HandleTypeDef *pid, float target)
{
    pid->set = target;
}
int pid_err[2] = {0};

float PID_calc(PID_HandleTypeDef *pid, float exp_data, float real_data)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    if(pid->mode == PID_MODE_POSITION)  /* 位置式 */
    {
        pid->set = exp_data;

        //误差迭代
        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->err3[0] = exp_data - real_data;//误差计算
        err = pid->err3[0];
        pid->pOut = pid->kp * pid->err3[0];
        pid->iOut+= (pid->ki * pid->err3[0]);
        pid->dOut = pid->kd * (pid->err3[0] - pid->err3[1]);

        if(pid->iOutMax != 0)//使用积分限幅
        {
            pid->iOut = Amplitude_Limit(pid->iOut, -(pid->iOutMax), pid->iOutMax);
        }

        pid->out = pid->pOut + pid->iOut + pid->dOut;
        pid_out =  pid->out;
        if(pid->outMax != 0)//使用输出限幅
        {
            pid->out = Amplitude_Limit(pid->out, -(pid->outMax), pid->outMax);
        }
    }
    else if(pid->mode == PID_MODE_POSITION_ISeparation)
    {
        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->err3[0] = exp_data - real_data;//误差计算

        pid->pOut  =  pid->kp * pid->err3[0];
        pid->iOut += (pid->ki * pid->err3[0]);
        pid->dOut  =  pid->kd * (pid->err3[0] - pid->err3[1]);


        if(ABS(pid->err3[0]) > pid->ISeparation_error_max )
        {
            pid->out = pid->pOut + pid->dOut;
            pid->iOut = 0;
        }
        else
        {
            pid->out = pid->pOut + pid->iOut + pid->dOut;
        }

        if(pid->outMax != 0)//使用输出限幅
        {
            pid->out=(pid->out > pid->outMax)?pid->outMax:pid->out;
            pid->out=(pid->out < (-(pid->outMax)))?(-pid->outMax):pid->out;
        }
    }
    else if(pid->mode == PID_MODE_ADD )
    {
        pid->err3[2] = pid->err3[1];
        pid->err3[1] = pid->err3[0];
        pid->err3[0] = exp_data - real_data;//误差计算

        pid->pOut  =  pid->kp * (pid->err3[0] -  pid->err3[1]);
        pid->iOut  = (pid->ki * pid->err3[0]);
        pid->dOut  =  pid->kd * ((pid->err3[0]-pid->err3[1])-(pid->err3[1]-pid->err3[2]));

        pid->out += pid->pOut + pid->iOut + pid->dOut ;
        pid_out =  pid->out;
        if(pid->outMax != 0)//使用输出限幅
        {
            pid->out=(pid->out > pid->outMax)?pid->outMax:pid->out;
            pid->out=(pid->out < (-(pid->outMax)))?(-pid->outMax):pid->out;
        }
    }

    return pid->out;
}

void PID_cleardata(PID_HandleTypeDef *pid)
{
    pid->err3[0] = 0;
    pid->err3[1] = 0;
    pid->err3[2] = 0;
    pid->pOut = 0;
    pid->iOut = 0;
    pid->dOut = 0;
    pid->out = 0;
}