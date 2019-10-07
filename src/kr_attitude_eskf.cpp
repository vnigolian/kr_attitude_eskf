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

#define ACC_COV 0.4//0.4
#define GYR_COV 5 //0.1


using namespace std;
using namespace kr;

  typedef kr::AttitudeESKF::scalar_t scalar_t;
  typedef kr::AttitudeESKF::vec3 Vec3;
  typedef kr::AttitudeESKF::mat3 Mat3;
  typedef kr::AttitudeESKF::quat Quat;
  
  
  Vec3 transform_axes(Vec3 vec){
	  Vec3 transformed;
	  transformed<< vec.x(), vec.y(), vec.z();
	  return transformed;
  } 
  
  
int main(int argc, char **argv) {
	cout<<"------------------------------------------------------------------------"<<endl;
	
	double acc_cov_factor(0);
	double gyr_cov_factor(0);
	
	if(argc == 3){
		acc_cov_factor = stof(argv[1]);
		gyr_cov_factor = stof(argv[2]);
	}else{
		acc_cov_factor = ACC_COV;
		gyr_cov_factor = GYR_COV;
	}
	
	cout<<"acc cov factor : "<<acc_cov_factor<<endl;
	cout<<"gyr cov factor : "<<gyr_cov_factor<<endl;
	
	const double dt(0.005);
	
	double acc_noise = acc_cov_factor * acc_cov_factor;
	double gyr_noise = gyr_cov_factor * gyr_cov_factor * dt * dt;
	
	cout<<"acc noise : "<<acc_noise<<endl;
	cout<<"gyr noise : "<<gyr_noise<<endl;
	
	
	AttitudeESKF filter;
	filter.setUsesMagnetometer(false);

	Vec3 acc_init;
	acc_init<<2.9382, -0.399575, -9.217725;
	
	Vec3 acc_init_cov;
	acc_init_cov<<acc_noise, acc_noise, acc_noise;
	


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
	
	
	Mat3 acc_cov = Mat3::Identity() * acc_noise;
	Mat3 gyr_cov = Mat3::Identity() * gyr_noise;
	
	
	
	char rest_of_the_line[128];
	//read the header
	fscanf(imu_input, "%s", rest_of_the_line);
	
	Vec3 acc;
	Vec3 gyr;
	
	double acc_norm;
	
	
	int i(0);
	int iteration_count(90000000);
	
	cout<<"running filter on data"<<endl;
	
	Quat result;
	
	while(i < iteration_count && 
		fscanf(imu_input, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %s", 
		&(acc.x()), &(acc.y()), &(acc.z()), &acc_norm, &(gyr.x()), &(gyr.y()), &(gyr.z()), rest_of_the_line) != EOF){
			
		acc = transform_axes(acc);
		gyr = transform_axes(gyr);
		
		//invert the axis
		//acc.z() *= -1;
		
		filter.predict(gyr, dt, gyr_cov);
		
		filter.update(acc, acc_cov);
		
		result = filter.getQuat();
		Vec3 bias = filter.getGyroBias();
		
		fprintf(gareth_output, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
				result.w(), result.x(), result.y(), result.z(),
				bias.x(), bias.y(), bias.z());
		/*printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
				result.w(), result.x(), result.y(), result.z(),
				bias.x(), bias.y(), bias.z());*/
		
		i++;
	}
	
	
	cout<<"DONE !"<<endl;
	
	cout<<"final quat : ("<<result.w()<<", "<<result.x()<<", "<<result.y()<<", "<<result.z()<<")"<<endl;
	cout<<"------------------------------------------------------------------------"<<endl;

	return 0;
}

