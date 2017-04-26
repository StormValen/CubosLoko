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
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}

float timePerFrame = 0.03;
glm::vec3 fuerzaInicial = glm::vec3 (50.f, 200.f, 0.f);
glm::vec3 gravedad = glm::vec3 (0.0f,-9.8f,0.0f);
glm::vec3 fuerzaTotal = fuerzaInicial+gravedad;
glm::vec3 torque = glm::vec3 (0.0f,0.0f,0.0f);
glm::vec3 PuntoDeFuerza = glm::vec3(0.1f,0.5f,0.1f);


struct theCube {
	glm::vec3 posCentroMasa;
	glm::vec3 lastPosCentroMasa;
	glm::vec3 vel;
	glm::quat qRotacion;
	glm::quat qLastRotacion;
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

void PhysicsInit() {
	TheCube->first = true;
	TheCube->rotacion = glm::mat3(1.0f);
	TheCube->velAngular = glm::vec3(0.0f,0.0f,0.0f);
	TheCube->masa = 0.3f;
	TheCube->vel = glm::vec3(0.0f, 0.0f, 0.0f);

	TheCube->linearMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->angularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->posCentroMasa = glm::vec3(0.0f, 0.0f, 0.0f);

	TheCube->inerciaBody = glm::mat3(TheCube->masa*2/12, 0, 0,
		0, TheCube->masa * 2 / 12, 0,
		0, 0, TheCube->masa * 2 / 12);

	TheCube->qRotacion = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
	

	torque = glm::cross((TheCube->posCentroMasa-PuntoDeFuerza-TheCube->posCentroMasa),fuerzaTotal);
	//TheCube->inertiaTensor = glm::mat3(TheCube->rotacion*TheCube->inerciaBody*glm::transpose(TheCube->rotacion));


}
void PhysicsUpdate(float dt) {
	TheCube->qLastRotacion = TheCube->qRotacion;
	
	TheCube->lastLinearMomentum = TheCube->linearMomentum;
	TheCube->linearMomentum = TheCube->lastLinearMomentum + (dt * fuerzaTotal);
	
	TheCube->lastAngularMomentum = TheCube->angularMomentum;
	
	if (TheCube->first) {
		TheCube->angularMomentum = TheCube->lastAngularMomentum + (dt * torque);
		fuerzaTotal -= fuerzaInicial;
		TheCube->first = false;
	}
	
	TheCube->vel = TheCube->linearMomentum / TheCube->masa;
	
	TheCube->lastPosCentroMasa = TheCube->posCentroMasa;
	TheCube->posCentroMasa = TheCube->lastPosCentroMasa + (dt * TheCube->vel);
	
	TheCube->lastRotacion = TheCube->rotacion;

	glm::mat3 TempRot = glm::mat3_cast(TheCube->qRotacion);

	TheCube->INVinertiaTensor = TempRot * glm::inverse(TheCube->inerciaBody) * glm::transpose(TempRot);
	TheCube->velAngular = TheCube->INVinertiaTensor * TheCube->angularMomentum;

	TheCube->qRotacion = TheCube->qLastRotacion + dt * 1/2*(glm::quat(0.0f,TheCube->velAngular.x, TheCube->velAngular.y, TheCube->velAngular.z)*TheCube->qLastRotacion);
	TheCube->qRotacion = glm::normalize(TheCube->qRotacion);

	glm::mat4 FinalRot = glm::mat4_cast(TheCube->qRotacion);

	glm::mat4 matIdentidad = glm::mat4(1.0f);
	matIdentidad = glm::translate(matIdentidad, glm::vec3(TheCube->posCentroMasa.x, TheCube->posCentroMasa.y, TheCube->posCentroMasa.z));
	matIdentidad = matIdentidad*FinalRot;

	Cube::updateCube(matIdentidad);
}
void PhysicsCleanup() {
	//TODO
}