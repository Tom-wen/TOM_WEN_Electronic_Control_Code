#include "PID.h"

/**************************几个PID除了过零保护外没有任何区别**********************************/

int Limit_Min_Max(int value,int min,int max);

/**
 * @brief PID数组初始化
 * @param PID PID数组
 * @param kp 
 * @param ki 
 * @param kd 
 * @param i_max 
 * @param out_max 
 */
void PID_init(PID_struct_t *PID,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
  PID->kp      = kp;
  PID->ki      = ki;
  PID->kd      = kd;
  PID->i_max   = i_max;//积分限幅
  PID->out_max = out_max;//输出限幅
}

 void PID_Protect_Angle(PID_struct_t *pid)
{
	if(pid->ref - pid->fdb > 4096)
	{
		pid->fdb+=8190;
	}
	else if(pid->ref - pid->fdb < -4096)
	{
		pid->fdb-=8190;
	}
}

 void PID_Protect_Ink(PID_struct_t *pid)
{
	if(pid->ref - pid->fdb > 180)
	{
		pid->fdb+=360;
	}
	else if(pid->ref - pid->fdb < -180)
	{
		pid->fdb-=360;
	}
}

float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb)//PID运算函数（目标，实际）
{
  PID->ref = ref;
  PID->fdb = fdb;

	PID_Protect_Angle(PID);//过零保护

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

float PID_Calc_Ink(PID_struct_t *PID, float ref, float fdb)//PID运算函数（目标，实际）
{
  PID->ref = ref;
  PID->fdb = fdb;

	PID_Protect_Ink(PID);//过零保护

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb)//PID运算函数（目标，实际）
{
  PID->ref = ref;
  PID->fdb = fdb;

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;

  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

/**
 * @brief 限制一个整数变量 value 在指定的最小值 min 和最大值 max 之间
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 
 */
int Limit_Min_Max(int value,int min,int max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}

/**************************模糊PID控制器实现**********************************/

// 模糊语言变量定义
enum fuzzy_variable { NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3 };

// 论域数量
int num_area = 7;

// 全局变量
float qerror = 0;
float qerror_c = 0;
float qdetail_kp = 0;
float qdetail_ki = 0;
float qdetail_kd = 0;
float detail_kp = 0;
float detail_ki = 0;
float detail_kd = 0;
float kp = 0;
float ki = 0;
float kd = 0;

int e_grad_index[2] = {0, 0};
int ec_grad_index[2] = {0, 0};
float e_gradmembership[2] = {0, 0};
float ec_gradmembership[2] = {0, 0};
float KpgradSums[7] = {0};
float KigradSums[7] = {0};
float KdgradSums[7] = {0};

// 误差论域值
float e_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
// 误差变化论域值
float ec_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
// kp增量论域值
float kp_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
// ki增量论域值
float ki_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
// kd增量论域值
float kd_menbership_values[7] = {-3, -2, -1, 0, 1, 2, 3};

// 量化函数
float Quantization(float maximum, float minimum, float x)
{
	float qvalues = 6.0 * (x - minimum) / (maximum - minimum) - 3;
	return qvalues;
}

// 反量化函数（去模糊化）
float Inverse_quantization(float maximum, float minimum, float qvalues)
{
	float x = (maximum - minimum) * (qvalues + 3) / 6 + minimum;
	return x;
}

// 输入e与de/dt隶属度计算函数
void Get_grad_membership(float error, float error_c)
{
	if (error > e_membership_values[0] && error < e_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (error >= e_membership_values[i] && error <= e_membership_values[i + 1])
			{
				e_gradmembership[0] = -(error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_gradmembership[1] = 1 + (error - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
				e_grad_index[0] = i;
				e_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (error <= e_membership_values[0])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 0;
			e_grad_index[1] = -1;
		}
		else if (error >= e_membership_values[6])
		{
			e_gradmembership[0] = 1;
			e_gradmembership[1] = 0;
			e_grad_index[0] = 6;
			e_grad_index[1] = -1;
		}
	}

	if (error_c > ec_membership_values[0] && error_c < ec_membership_values[6])
	{
		for (int i = 0; i < num_area - 2; i++)
		{
			if (error_c >= ec_membership_values[i] && error_c <= ec_membership_values[i + 1])
			{
				ec_gradmembership[0] = -(error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_gradmembership[1] = 1 + (error_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
				ec_grad_index[0] = i;
				ec_grad_index[1] = i + 1;
				break;
			}
		}
	}
	else
	{
		if (error_c <= ec_membership_values[0])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 0;
			ec_grad_index[1] = -1;
		}
		else if (error_c >= ec_membership_values[6])
		{
			ec_gradmembership[0] = 1;
			ec_gradmembership[1] = 0;
			ec_grad_index[0] = 6;
			ec_grad_index[1] = -1;
		}
	}
}

// 计算输出增量kp,ki,kd的总隶属度
void GetSumGrad()
{
	int Kp_rule_list[7][7] = { {PB, PB, PM, PM, PS, ZO, ZO},      // kp规则表
				{PB, PB, PM, PS, PS, ZO, NS},
				{PM, PM, PM, PS, ZO, NS, NS},
				{PM, PM, PS, ZO, NS, NM, NM},
				{PS, PS, ZO, NS, NS, NM, NM},
				{PS, ZO, NS, NM, NM, NM, NB},
				{ZO, ZO, NM, NM, NM, NB, NB} };

	int Ki_rule_list[7][7] = { {NB, NB, NM, NM, NS, ZO, ZO},     // ki规则表
				{NB, NB, NM, NS, NS, ZO, ZO},
				{NB, NM, NS, NS, ZO, PS, PS},
				{NM, NM, NS, ZO, PS, PM, PM},
				{NM, NS, ZO, PS, PS, PM, PB},
				{ZO, ZO, PS, PS, PM, PB, PB},
				{ZO, ZO, PS, PM, PM, PB, PB} };

	int Kd_rule_list[7][7] = { {PS, NS, NB, NB, NB, NM, PS},     // kd规则表
				{PS, NS, NB, NM, NM, NS, ZO},
				{ZO, NS, NM, NM, NS, NS, ZO},
				{ZO, ZO, NS, NS, NS, ZO, ZO},
				{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
				{PB, PS, PS, PS, PS, PS, PB},
				{PB, PM, PM, PM, PS, PS, PB} };

	// 初始化 Kp、Ki、Kd 总的隶属度值为 0
	for (int i = 0; i <= num_area - 1; i++)
	{
		KpgradSums[i] = 0;
		KigradSums[i] = 0;
		KdgradSums[i] = 0;
	}

	for (int i = 0; i < 2; i++)
	{
		if (e_grad_index[i] == -1)
		{
			continue;
		}
		for (int j = 0; j < 2; j++)
		{
			if (ec_grad_index[j] != -1)
			{
				int indexKp = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
				int indexKi = Ki_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
				int indexKd = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;

				KpgradSums[indexKp] = KpgradSums[indexKp] + (e_gradmembership[i] * ec_gradmembership[j]);
				KigradSums[indexKi] = KigradSums[indexKi] + (e_gradmembership[i] * ec_gradmembership[j]);
				KdgradSums[indexKd] = KdgradSums[indexKd] + (e_gradmembership[i] * ec_gradmembership[j]);

			}
			else
			{
				continue;
			}
		}
	}
}

// 计算输出增量kp,ki,kd对应论域值
void GetOUT()
{
	for (int i = 0; i < num_area - 1; i++)
	{
		qdetail_kp += kp_menbership_values[i] * KpgradSums[i];
		qdetail_ki += ki_menbership_values[i] * KigradSums[i];
		qdetail_kd += kd_menbership_values[i] * KdgradSums[i];
	}
}

// 模糊PID控制实现函数
float FuzzyPIDcontroller(float e_max, float e_min, float ec_max, float ec_min, float kp_max, float kp_min, float error, float error_c, float ki_max, float ki_min, float kd_max, float kd_min, float error_pre, float error_ppre)
{
	qerror = Quantization(e_max, e_min, error);        // 将误差 error 映射到论域中
	qerror_c = Quantization(ec_max, ec_min, error_c); // 将误差变化 error_c 映射到论域中

	Get_grad_membership(qerror, qerror_c); // 计算误差 error 和误差变化 error_c 的隶属度
	GetSumGrad();                         // 计算输出增量 △kp、△ki、△kd 的总隶属度
	GetOUT();                             // 计算输出增量 △kp、△ki、△kd 对应论域值

	detail_kp = Inverse_quantization(kp_max, kp_min, qdetail_kp); // 去模糊化得到增量 △kp
	detail_ki = Inverse_quantization(ki_max, ki_min, qdetail_ki); // 去模糊化得到增量 △ki
	detail_kd = Inverse_quantization(kd_max, kd_min, qdetail_kd); // 去模糊化得到增量 △kd

	// 清零量化值
	qdetail_kd = 0;
	qdetail_ki = 0;
	qdetail_kp = 0;

	kp = kp + detail_kp; // 得到最终的 kp 值
	ki = ki + detail_ki; // 得到最终的 ki 值
	kd = kd + detail_kd; // 得到最终的 kd 值

	// 参数限幅
	if (kp < 0)
		kp = 0;
	if (ki < 0)
		ki = 0;
	if (kd < 0)
		kd = 0;

	// 清零增量
	detail_kp = 0;
	detail_ki = 0;
	detail_kd = 0;

	// 计算最终的输出
	float output = kp * (error - error_pre) + ki * error + kd * (error - 2 * error_pre + error_ppre);

	return output;
}

/**************************简化版模糊PID控制器实现**********************************/

/**
 * @brief 模糊PID控制器初始化
 * @param fuzzy_pid 模糊PID结构体指针
 * @param kp 初始Kp值
 * @param ki 初始Ki值
 * @param kd 初始Kd值
 * @param out_max 输出限幅
 * @param e_max 误差量化上限
 * @param e_min 误差量化下限
 * @param ec_max 误差变化率量化上限
 * @param ec_min 误差变化率量化下限
 * @param kp_max Kp增量上限
 * @param kp_min Kp增量下限
 * @param ki_max Ki增量上限
 * @param ki_min Ki增量下限
 * @param kd_max Kd增量上限
 * @param kd_min Kd增量下限
 */
void FuzzyPID_init(FuzzyPID_struct_t *fuzzy_pid,
                   float kp, float ki, float kd,
                   float out_max,
                   float e_max, float e_min,
                   float ec_max, float ec_min,
                   float kp_max, float kp_min,
                   float ki_max, float ki_min,
                   float kd_max, float kd_min)
{
	// 初始化PID参数
	fuzzy_pid->kp = kp;
	fuzzy_pid->ki = ki;
	fuzzy_pid->kd = kd;
	fuzzy_pid->out_max = out_max;
	
	// 初始化量化范围参数
	fuzzy_pid->e_max = e_max;
	fuzzy_pid->e_min = e_min;
	fuzzy_pid->ec_max = ec_max;
	fuzzy_pid->ec_min = ec_min;
	fuzzy_pid->kp_max = kp_max;
	fuzzy_pid->kp_min = kp_min;
	fuzzy_pid->ki_max = ki_max;
	fuzzy_pid->ki_min = ki_min;
	fuzzy_pid->kd_max = kd_max;
	fuzzy_pid->kd_min = kd_min;
	
	// 初始化历史误差
	fuzzy_pid->err[0] = 0;
	fuzzy_pid->err[1] = 0;
	fuzzy_pid->err[2] = 0;
	
	// 初始化输出
	fuzzy_pid->output = 0;
	fuzzy_pid->ref = 0;
	fuzzy_pid->fdb = 0;
}

/**
 * @brief 模糊PID控制器计算函数（简化版）
 * @param fuzzy_pid 模糊PID结构体指针
 * @param ref 目标值
 * @param fdb 实际值（反馈值）
 * @return 控制输出
 */
float FuzzyPID_Calc(FuzzyPID_struct_t *fuzzy_pid, float ref, float fdb)
{
	// 保存当前的目标值和实际值
	fuzzy_pid->ref = ref;
	fuzzy_pid->fdb = fdb;
	
	// 更新误差历史
	fuzzy_pid->err[2] = fuzzy_pid->err[1];
	fuzzy_pid->err[1] = fuzzy_pid->err[0];
	fuzzy_pid->err[0] = ref - fdb;
	
	// 计算误差变化率
	float error_c = fuzzy_pid->err[0] - fuzzy_pid->err[1];
	
	// 调用原始模糊PID控制器
	float output = FuzzyPIDcontroller(
		fuzzy_pid->e_max, fuzzy_pid->e_min,
		fuzzy_pid->ec_max, fuzzy_pid->ec_min,
		fuzzy_pid->kp_max, fuzzy_pid->kp_min,
		fuzzy_pid->err[0], error_c,
		fuzzy_pid->ki_max, fuzzy_pid->ki_min,
		fuzzy_pid->kd_max, fuzzy_pid->kd_min,
		fuzzy_pid->err[1], fuzzy_pid->err[2]
	);
	
	// 输出限幅
	if (output > fuzzy_pid->out_max)
		output = fuzzy_pid->out_max;
	else if (output < -fuzzy_pid->out_max)
		output = -fuzzy_pid->out_max;
	
	fuzzy_pid->output = output;
	
	return output;
}
