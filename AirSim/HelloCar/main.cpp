// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>
#include <string>   //ADICIONEI A BIBLIOTECA

using namespace msr::airlib;

bool ChegouNoFinal(const msr::airlib::Pose &pose)
{
	if ((pose.position[0] > -5) && (pose.position[0] < 5)){ 
		if ((pose.position[1] > 0.2) && (pose.position[1] < 1.2)) //y
		return true;
	}
	return false;

}
bool deveSalvarPonto(const msr::airlib::Pose &poseInitial, const msr::airlib::Pose &poseFinal,  float intervalo)
{

	float finalXposition = poseFinal.position[0];
	float finalYposition = poseFinal.position[1];
	float initialXposition = poseInitial.position[0];
	float initialYposition = poseInitial.position[1];
	float dist = sqrt((pow((finalXposition - initialXposition), 2) + (pow((finalYposition - initialYposition), 2))));
	if (dist >= intervalo) 
		return true;
	return false;
}

void printCarPose(const msr::airlib::Pose &pose, float speed)
{
	std::cout << "x=" << pose.position[0] << " y=" << pose.position[1] << " v=" << speed << std::endl;

}

void saveCarPose(std::ofstream &Valores, msr::airlib::Pose &pose, float speed)
{
	
	Valores << pose.position[0] << ";" << pose.position[1] << ";" << speed << std::endl;
	
}

void moveForwardAndBackward(msr::airlib::CarRpcLibClient &client)
{
	client.enableApiControl(true);
	CarApiBase::CarControls controls;

	std::cout << "Pressione Enter andar pra frente." << std::endl; std::cin.get();
	controls.throttle = 0.5f;
	controls.steering = 0.0f;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para puxar o freio de mão." << std::endl; std::cin.get();
	controls.handbrake = true;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para fazer uma manobra." << std::endl; std::cin.get();
	controls.handbrake = false;
	controls.throttle = -0.5;
	controls.steering = 1;
	controls.is_manual_gear = true;
	controls.manual_gear = -1;
	client.setCarControls(controls);

	std::cout << "Pressione Enter para parar." << std::endl; std::cin.get();
	client.setCarControls(CarApiBase::CarControls());
}

void ModoManual(msr::airlib::CarRpcLibClient &simulador)
{
	std::ofstream Valores("Valores.csv");

	msr::airlib::Pose car_poseInitial;
	car_poseInitial.position[0] = 0;
	car_poseInitial.position[1] = 0;
	msr::airlib::Pose car_poseFinal;
	do {
		auto car_state = simulador.getCarState();
		car_poseFinal = car_state.kinematics_estimated.pose;
		auto car_speed = car_state.speed;
		if (deveSalvarPonto(car_poseInitial, car_poseFinal, 1)) {
			saveCarPose(Valores, car_poseInitial, car_speed);
			car_poseInitial = car_poseFinal;
		}
	} while (!ChegouNoFinal(car_poseInitial));

	Valores.close();
}

void ModoAuto(msr::airlib::CarRpcLibClient &simulador) 
{
	std::ifstream Valores("Valores.csv");

	//string getline()               // IMPLEMENTAR A FUNÇAO GETLINE P LER O ARQUIVO
	
	//getline(arq, linha);

	Valores.close();

}
/*
x = [0 0 0 0];
dt = 1 / 20;
baseline = 4.5; % Wheels baseline(default 4.5m)
accelmax = 9.5; % Tesla Model S(96Km / h em 2.8s)
steermax = 1.22; % Maximum steering(default 0.5 radians);
*/

/*void CarroAnda(const vector<float> &x,const float dt,const float baseline,const float accelmax,const float steermax, const  vector<float> &viapoints){

	auto lateral_control = purepursuit(baseline, steermax, 1.0);

	auto velocity_control = pidcontrol(1, 1, 0.01, dt);

	last_state = x;
	last_state = 0;

	while (true) {
		current_state = last_state;
		current_time = last_time + dt;

		if (finished(checkpoints)) {
			break;
		}

		current_velocity = current_state(4);
		desired_velocity = get(checkpoints, current_state);

		lateral_control = update(lateral_control, current_state);
		steering = get(lateral_control, 'steering');

		velocity_control = update(velocity_control, current_time, current_velocity, desired_velocity);
		acceleration = get(velocity_control, 'acceleration');

		control_command = [acceleration, steering];

		//[T, Y] = ode45(@(t, x) kinematics(car, t, x, control_command), [last_time current_time], current_state);
		//passar os comandos para o Airsim
		last_state = current_time;
	}


}*/





int main()
{
    std::cout << "Verifique se o arquivo Documentos\\AirSim\\settings.json " <<
				 "está configurado para simulador de carros \"SimMode\"=\"Car\". " <<
				 "Pressione Enter para continuar." << std::endl; 
	std::cin.get();

    msr::airlib::CarRpcLibClient simulador;
    try {        
		simulador.confirmConnection();
		simulador.reset();

		ModoManual(simulador);

		//ModoAuto(simulador);

	}
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
    }

    return 0;
}
