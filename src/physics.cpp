#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include "GL_framework.h"
#include <stdlib.h>
#include <time.h>
#include <iostream>

bool show_test_window = false;

namespace Cube {
	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(glm::mat4* transform);
	extern void drawCube();
}

float timePerFrame = 0.03;
glm::vec3 fuerzaInicial = glm::vec3 (3.0f, 3.0f, 3.0f);
glm::vec3 gravedad = glm::vec3 (0.0f,-9.8f,0.0f);
glm::vec3 fuerzaTotal = fuerzaInicial+gravedad;
glm::vec3 torque = glm::vec3 (0.0f,0.0f,0.0f);


struct theCube {
	glm::vec3 posCentroMasa;
	glm::vec3 lastPosCentroMasa;
	glm::vec3 vel;
	glm::mat3 rotacion;
	glm::mat3 lastRotacion;
	glm::vec3 velAngular;
	glm::vec3 linearMomentum;
	glm::vec3 angularMomentum;
	glm::vec3 lastLinearMomentum;
	glm::vec3 lastAngularMomentum;
	//glm::mat3 inertiaTensor;
	glm::mat3 INVinertiaTensor;
	bool first;
	glm::mat3 inerciaBody;
	GLfloat masa;
};

theCube *TheCube = new theCube();

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void UpdatePosition() {

}



void PhysicsInit() {
	TheCube->first = true;
	TheCube->rotacion = glm::mat3(0.0f, 0.0f, 0.0f,
								  0.0f, 0.0f, 0.0f,
								  0.0f, 0.0f, 0.0f);
	TheCube->velAngular = glm::vec3(0.0f,0.0f,0.0f);
	TheCube->masa = 3.0f;
	TheCube->vel = glm::vec3(0.0f, 0.0f, 0.0f);

	TheCube->linearMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->angularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->posCentroMasa = glm::vec3(0.0f, 0.5f, 0.0f);
	TheCube->inerciaBody = glm::mat3(((1/12)*TheCube->masa)*((glm::pow(0.5f,2) + glm::pow(0.5,2))),0,0,
									 0,((1 / 12)*TheCube->masa)*((glm::pow(0.5f, 2) + glm::pow(0.5, 2))),0,
									 0,0,((1 / 12)*TheCube->masa)*((glm::pow(0.5f, 2) + glm::pow(0.5, 2))));
	torque = glm::cross(fuerzaTotal, (TheCube->posCentroMasa-glm::vec3(0.0f,0.5f,0.0f),-TheCube->posCentroMasa));
	//TheCube->inertiaTensor = glm::mat3(TheCube->rotacion*TheCube->inerciaBody*glm::transpose(TheCube->rotacion));
}
void PhysicsUpdate(float dt) {
	TheCube->lastLinearMomentum = TheCube->linearMomentum;
	TheCube->linearMomentum = TheCube->lastLinearMomentum + (dt * fuerzaTotal);
	TheCube->lastAngularMomentum = TheCube->angularMomentum;
	if (TheCube->first) {
		TheCube->angularMomentum = TheCube->lastAngularMomentum + (dt * torque);
		TheCube->first = false;
	}
	TheCube->vel = TheCube->linearMomentum / TheCube->masa;
	TheCube->lastPosCentroMasa = TheCube->posCentroMasa;
	TheCube->posCentroMasa = TheCube->lastPosCentroMasa + (dt * TheCube->vel);
	TheCube->lastRotacion = TheCube->rotacion;
	TheCube->INVinertiaTensor = TheCube->lastRotacion * glm::inverse(TheCube->inerciaBody) * glm::transpose(TheCube->lastRotacion);
	TheCube->velAngular = TheCube->INVinertiaTensor * TheCube->angularMomentum;
	glm::mat3 matrizAngular = glm::mat3(0, -TheCube->velAngular.z, TheCube->velAngular.y,
		TheCube->velAngular.z, 0, -TheCube->velAngular.x,
		-TheCube->velAngular.y, TheCube->velAngular.x, 0);
	TheCube->rotacion = TheCube->lastRotacion + dt * (matrizAngular*TheCube->lastRotacion);


	//TODO objMat crear matriz FOTO!!!!!!!!!!

}
void PhysicsCleanup() {
	//TODO
}