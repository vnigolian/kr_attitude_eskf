/*
 * kr_attitude_eskf.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_attitude_eskf.
 *
 *  Created on: 17/6/2014
 *		  Author: gareth
 */

#include "AttitudeESKF.hpp"
#include <iostream>

#define ACC_COV 0.4
#define GYR_COV 0.1


using namespace std;
using namespace kr;

  typedef kr::AttitudeESKF::scalar_t scalar_t;
  typedef kr::AttitudeESKF::vec3 Vec3;
  typedef kr::AttitudeESKF::mat3 Mat3;
  typedef kr::AttitudeESKF::quat Quat;
  
int main(int argc, char **argv) {
	AttitudeESKF filter;
	filter.setUsesMagnetometer(false);

	Vec3 acc_init;
	acc_init<<2.9908, -0.41632, -9.1926;
	
	Vec3 acc_init_cov;
	acc_init_cov<<ACC_COV, ACC_COV, ACC_COV;
	


	if( filter.initialize(acc_init, acc_init_cov)){
		std::cout<<"init success"<<std::endl;
	}else{
			
		std::cout<<"init FAILED"<<std::endl;
		return -1;
	}
	
	FILE* imu_input = fopen("/home/val/Desktop/CSVdata/CL_40_1_imu.csv", "r");
	FILE* gareth_output = fopen("/home/val/Desktop/gareth_output.csv", "w+");
	
	if(!imu_input || !gareth_output){
		cout<<"COULD NOT OPEN INPUT AND/OR OUTPUT FILES"<<endl;
		return -1;
	}
	Mat3 acc_cov = Mat3::Identity()*ACC_COV;
	Mat3 gyr_cov = Mat3::Identity()*GYR_COV;
	
	const double dt(0.005);
	
	
	char rest_of_the_line[128];
	//read the header
	fscanf(imu_input, "%s", rest_of_the_line);
	
	Vec3 acc;
	Vec3 gyr;
	
	double acc_norm;
	
	
	int i(0);
	int iteration_count(90000000);
	
	cout<<"running filter on data"<<endl;
	
	while(i < iteration_count && 
		fscanf(imu_input, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %s", 
		&(acc.x()), &(acc.y()), &(acc.z()), &acc_norm, &(gyr.x()), &(gyr.y()), &(gyr.z()), rest_of_the_line) != EOF){
			
		filter.predict(gyr, dt, gyr_cov);
		
		filter.update(acc, acc_cov);
		
		Quat result = filter.getQuat();
		Vec3 bias = filter.getGyroBias();
		
		fprintf(gareth_output, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
				result.w(), result.x(), result.y(), result.z(),
				bias.x(), bias.y(), bias.z());
		
		i++;
	}
	
	cout<<"DONE !"<<endl;

	return 0;
}

