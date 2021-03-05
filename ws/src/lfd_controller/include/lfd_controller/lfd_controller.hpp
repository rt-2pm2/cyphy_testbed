/*  Copyright (C) 2020, Luigi Pannocchi.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.  */


#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility> 

const int outputSize = 3;
const int stateSize1D = 3;
const int stateSize = (stateSize1D * 3);

typedef Eigen::Matrix<double, outputSize, 1> UType;
typedef Eigen::Matrix<double, stateSize, 1> XType;
typedef Eigen::Matrix<double, outputSize, stateSize> KType;

class LFDController {
	public:
		LFDController();
		~LFDController(); 

		void SetActive(bool active);
		bool isControlActive();

		void SetInitialState(const XType& x0);
		void SetSetpoint(const XType& xref);
		void SetState(const XType& x);
		void SetQuat(const Eigen::Quaterniond& q);

		void SetKyaw(double d);

		void LoadK();

		bool Step(double T); 
		void getControls(UType& ctrls) const;
		const UType getControls() const;
		double getYawCtrl();

		void SetFeedForward(const UType& u_ff);

	private:
		bool active_;

		double t0_;
		unsigned int N_period_;

		XType x0_;

		XType state_ref_;
		XType state_;

		UType ctrl_ff_;

		Eigen::MatrixXd K_;
		Eigen::Quaterniond quat_;

		double kr_rates_;

		UType ctrl_outputs_;
		double ctrl_yaw_;

		UType ComputeU(const XType& err, double T);
		

		double ComputeYawCtrl(const Eigen::Vector3d& acc_d,
				const Eigen::Vector3d& acc);

		void initTime(double t);

};
