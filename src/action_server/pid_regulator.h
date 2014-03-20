#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H


class PID
{
	double P, I, D;
	double prev_err;
	bool not_started;
	double t_diff;
	double integral;

public:
	PID(double _P = 0.0, double _I = 0.0, double _D = 0.0)
	{
		P = _P; I = _I; D = _D;
		not_started = true;
		prev_err = 0;
		integral = 0;
		t_diff = 0.0333333; // 1/30 sec
	}

	void set_PID(double _P, double _I, double _D)
	{
		P = _P; I = _I; D = _D;
	}

	double get_output(double target_val, double current_val)
	{
		double err = target_val - current_val;

		if(not_started)
		{
			not_started = false;
			prev_err = err;
			return P * err;
		}
		integral += err;

		double P_val = P * err;
		double I_val = I * integral * t_diff;
		double D_val = D * (err - prev_err) / t_diff;

		prev_err = err;
		return P_val + I_val + D_val;
	}
};

#endif // PID_REGULATOR_H
