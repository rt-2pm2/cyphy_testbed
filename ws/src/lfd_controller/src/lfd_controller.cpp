/**
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>

#include <ros/package.h>

#include "lfd_controller/lfd_controller.hpp"

using namespace std;
using namespace Eigen;

// Read matrix from CSV to Eigen:
template<typename M>
M load_csv (const string& path) {
	ifstream indata;
	indata.open(path);
	string line;
	vector<double> values;
	uint rows = 0;
	while (getline(indata, line)) {
		stringstream lineStream(line);
		string cell;
		while (getline(lineStream, cell, ',')) {
			values.push_back(stod(cell));
		}
		++rows;
	}
	return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}


LFDController::LFDController() :
	active_{false},
	ctrl_outputs_(UType::Zero()),
	x0_{XType::Zero()},
	state_{XType::Zero()},
	state_ref_{XType::Zero()},
	ctrl_ff_{UType::Zero()},
	quat_{Quaterniond::Identity()} {
		ctrl_yaw_ = 0;
		kr_rates_ = 0.8;
		N_period_ = 1;
	}

LFDController::~LFDController() {}

void LFDController::SetInitialState(const XType& x0) {
	x0_ = x0;	
}


void LFDController::SetKyaw(double kyaw) {
	cout << "CIS Supervisor - setting Kyaw: ";

	kr_rates_ = kyaw;
	cout << kr_rates_ << endl;;
}


void LFDController::SetQuat(const Quaterniond& q) {
	quat_ = q;
}


void LFDController::LoadK() {
	string path = ros::package::getPath("lfd_controller");
	string filename = path + "/config/data/";
	//cout << "[LFDController] Loading " << filename << " data." << endl;
	K_ = load_csv<Eigen::MatrixXd>(filename + "K.csv");

	N_period_ = K_.rows() / 3;

	std::cout << "LFDController: Loaded K [" << K_.rows() << " x " << K_.cols() <<"]! " << std::endl;
}

UType LFDController::ComputeU(const XType& err, double t) {
	UType U(UType::Zero()); 

	unsigned int index = (unsigned int)((t - t0_) * 1000) % N_period_;

	KType K = K_.block<outputSize, stateSize>(index, 0);

	for (int i = 0; i < outputSize; i++) {
		for (int j = 0; j < stateSize; j++) {
			U(i) += K(j) * err(j);
		}
	}

	return U;
}




double LFDController::ComputeYawCtrl(
		const Vector3d& acc_d,
		const Vector3d& acc) {

	// Frame with the desired Zb and X align with the the X inertial frame 
	Vector3d z_d = acc_d.normalized();
	Vector3d y_d = Vector3d::UnitY();
	Vector3d x_d = (y_d.cross(z_d)).normalized();
	Matrix3d Rdes;
	Rdes.col(0) = x_d;
	Rdes.col(1) = y_d;
	Rdes.col(2) = z_d;

	// Current Zb
	Vector3d z = acc.normalized();	
	Vector3d ni = z.cross(z_d);
	double alpha = std::asin(ni.norm());
	ni.normalize();

	// Express the rotation in body frame
	Quaterniond q_pq(Quaterniond::Identity());

	if (abs(alpha) > 0.001) {
		Vector3d nb = quat_.inverse() * ni;
		q_pq = AngleAxisd(alpha, nb);
	}

	Quaterniond q_r =
		q_pq.inverse() * quat_.inverse() * Quaterniond(Rdes);

	double ctrl_yaw = (q_r.w() > 0) ? 
		(2.0 * kr_rates_ * q_r.z()) : (-2.0 * kr_rates_ * q_r.z());

	return ctrl_yaw;
}

bool LFDController::Step(double T) {
	XType state_curr_ = state_;

	// Compute the error
	XType err = state_ref_ - state_curr_;
	UType u_des = ComputeU(err, T) + ctrl_ff_;
	Vector3d acc_ref = state_ref_.block<3, 1>(6, 0);
	Vector3d acc = state_.block<3, 1>(6, 0);
	ctrl_yaw_ = ComputeYawCtrl(acc_ref, acc);
	ctrl_outputs_ = u_des;

	return true;
}

void LFDController::getControls(UType& ctrls) const {
	ctrls = ctrl_outputs_;
}

const UType LFDController::getControls() const {
	return ctrl_outputs_;
}

double LFDController::getYawCtrl() {
	return ctrl_yaw_;
}

void LFDController::SetSetpoint(const XType& xref) {
	state_ref_ = xref;
}


void LFDController::SetFeedForward(const UType& u_ff) {
	ctrl_ff_ = u_ff;
}

void LFDController::SetState(const XType& x) {
	state_ = x;
}

void LFDController::SetActive(bool active) {
	active_ = active;
}

bool LFDController::isControlActive() {
	return active_;
}

void LFDController::initTime(double t_) {
	t0_ = t_;
	return;
}
