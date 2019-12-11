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
#include"LateralControl.h"
#include"LongitudinalControl.h"
#include "Waypoints.h"



using namespace std;
using namespace msr::airlib;
using msr::airlib::Pose;


float deveSalvar(const Pose &a, const Pose &b, float intervalo) {

	float dist = sqrt((pow((b.position[0] - a.position[0]), 2) + pow((b.position[1] - a.position[1]), 2)));
	if (dist >= intervalo)
		return true;

	return false;
}


void printCarPose(const msr::airlib::Pose &pose, float speed)
{
	std::cout << "x=" << pose.position[0] << " y=" << pose.position[1] << " v=" << speed << std::endl;

}

void saveCarPose(ofstream &Valores, const msr::airlib::Pose &pose, float speed)
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

void moveInTheTrajectory(msr::airlib::CarRpcLibClient &client, float &aceleration, float &steering) {
	CarApiBase::CarControls controls;
	if (aceleration >= 0)
		controls.throttle = aceleration;
	else
		controls.brake = -aceleration;
	controls.steering = steering;
	client.setCarControls(controls);
}


bool chegada(const msr::airlib::Pose &pose) {
	if (pose.position[0] > -3 && pose.position[0] <-1) {
		if (pose.position[1] > -5 && pose.position[1] <5) {
			return true;
		}
	}
	return false;
}


int main()
{
	msr::airlib::CarRpcLibClient simulador;
	Waypoints checkpoints, trajectory;
	LateralControl lateral_control(20, 6, 10);
	LongitudinalControl velocity_control(2.0, 0.0, 0.05);

	int opcao;
	std::cout << "Favor digite a opcao de controle:\n";
	std::cout << "Opcao 1-Manual\nOpcao 2-Automatico\n";
	std::cin >> opcao;

	try {

		simulador.confirmConnection();
		simulador.reset();


		if (opcao == 2) {
			checkpoints.LoadWaypoints("Pontos.txt");
			simulador.enableApiControl(true);
		}

		msr::airlib::Pose poseAnterior;
		msr::airlib::Pose poseAtual;
		poseAnterior.position[0] = 0;
		poseAnterior.position[1] = 0;

		do {
			auto car_state = simulador.getCarState();
			poseAtual = car_state.kinematics_estimated.pose;
			auto velocidade = car_state.speed;

			if (opcao == 2) {

				Vector3r pose(poseAtual.position[0], poseAnterior.position[1], VectorMath::yawFromQuaternion(poseAtual.orientation));
				double desired_velocity = checkpoints.GetWaypointVelocity(pose);
				float steering = lateral_control.Update(checkpoints, pose, velocidade);
				float aceleration = velocity_control.Update(velocidade, desired_velocity);
				moveInTheTrajectory(simulador, aceleration, steering);
			}


			if (deveSalvar(poseAnterior, poseAtual, 1)) {
				trajectory.AddWaypoints(poseAtual.position[0], poseAtual.position[1], velocidade);

				poseAnterior = poseAtual;
			}
		} while (!chegada(poseAnterior));


		trajectory.SaveWaypoints("caminho.txt");
	}


	catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique a exceção lançada pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}

	return 0;
}