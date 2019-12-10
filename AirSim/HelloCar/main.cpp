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
#include <sstream>
#include "LongitudinalControl.h"
#include "LateralControl.h"
#include "Waypoints.h"

using namespace std;
using namespace msr::airlib;
using msr::airlib::Pose;


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


bool ChegouNoFinal(const msr::airlib::Pose &pose)
{
	if ((pose.position[0] > -3) && (pose.position[0] < -1)) {
		if ((pose.position[1] > -5) && (pose.position[1] < 5)) //y
			return true;
	}
	return false;

}


int main()
{

	Waypoints checkpoints, trajectory;
	LateralControl Lateral_control(2.3, 1, 1);
	LongitudinalControl velocity_control(1.0, 1.0, 0.01);
	msr::airlib::CarRpcLibClient simulador;
	int escolhafeita;
	std::cout << "Digite a opcao desejada:\n";
	std::cout << "[1] Modo Manual.\n";
	std::cout << "[2] Modo Automatico.\n";
	std::cin >> escolhafeita;

	try {
		simulador.confirmConnection();
		simulador.reset();

		if (escolhafeita == 2) {
			checkpoints.LoadWaypoints("CaminhoAserSeguido.txt");
			simulador.enableApiControl(true);
		}

		msr::airlib::Pose poseAnterior;
		poseAnterior.position[0] = 0;
		poseAnterior.position[1] = 0;
		msr::airlib::Pose poseAtual;
		do {
			auto car_state = simulador.getCarState();

			auto poseAtual = car_state.kinematics_estimated.pose;
			Vector3r pose(poseAtual.position[0], poseAtual.position[1], VectorMath::yawFromQuaternion(poseAtual.orientation));
			double desired_velocity = checkpoints.GetWaypointVelocity(pose);
			auto velocidade = car_state.speed;
			
			float steering = Lateral_control.Update(trajectory, pose, velocidade);
			
			float accelaration = velocity_control.Update(velocidade, desired_velocity);

			CarApiBase::CarControls controls;
			controls.steering = steering;
			if (accelaration > 0)
				controls.throttle = accelaration;
			else
				controls.brake = -accelaration;

			simulador.setCarControls(controls);

			if (deveSalvarPonto(poseAnterior, poseAtual, 1.0)) {
				trajectory.AddWaypoints(poseAtual.position[0], poseAtual.position[1], velocidade);
				poseAnterior = poseAtual;
			}
		} while (!ChegouNoFinal(poseAnterior));
		trajectory.SaveWaypoints("CaminhoPorOndePassou.txt");
	}
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}

	return 0;
}
