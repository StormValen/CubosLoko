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
glm::vec3 fuerzaInicial;
glm::vec3 gravedad = glm::vec3 (0.0f,-9.8f,0.0f);
glm::vec3 fuerzaTotal;
glm::vec3 torque = glm::vec3 (0.0f,0.0f,0.0f);
glm::vec3 PuntoDeFuerza;
float halfW = 0.5;
int t =0;

glm::vec3 verts[] = {
	glm::vec3(-halfW, -halfW, -halfW),
	glm::vec3(-halfW, -halfW,  halfW),
	glm::vec3(halfW, -halfW,  halfW),
	glm::vec3(halfW, -halfW, -halfW),
	glm::vec3(-halfW,  halfW, -halfW),
	glm::vec3(-halfW,  halfW,  halfW),
	glm::vec3(halfW,  halfW,  halfW),
	glm::vec3(halfW,  halfW, -halfW)
};

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

theCube *TheCube;

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

glm::vec3 RandPosCube() {
	srand(time(NULL));
	glm::vec3 newCubePos = glm::vec3(rand() % 8 - 4, 2, rand() % 8 - 4);
	
	//la fuerza se aplica en la cara inferior del cubo en posicion random
	PuntoDeFuerza =glm::vec3(newCubePos.x + ((float)rand()/RAND_MAX -0.5f), newCubePos.y - 0.5, newCubePos.z + ((float)rand() / RAND_MAX - 0.5f));
	//cantidad de fuerza
	fuerzaInicial = glm::vec3(-50 + rand()%100, rand() % 50 + 150, -50 + rand() % 100);
	fuerzaTotal = fuerzaInicial+gravedad;
	return newCubePos;
}


void PhysicsInit() {
	TheCube = new theCube();

	TheCube->first = true;
	TheCube->rotacion = glm::mat3(1.0f);
	TheCube->velAngular = glm::vec3(0.0f,0.0f,0.0f);
	TheCube->masa = 0.5f;
	TheCube->vel = glm::vec3(0.0f, 0.0f, 0.0f);

	TheCube->linearMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->angularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
	TheCube->posCentroMasa = RandPosCube();

	TheCube->inerciaBody = glm::mat3(TheCube->masa*2/12, 0, 0,
		0, TheCube->masa * 2 / 12, 0,
		0, 0, TheCube->masa * 2 / 12);

	TheCube->qRotacion = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
	

	torque = glm::cross(TheCube->posCentroMasa-PuntoDeFuerza,fuerzaTotal);
	//TheCube->inertiaTensor = glm::mat3(TheCube->rotacion*TheCube->inerciaBody*glm::transpose(TheCube->rotacion));
}

void UpdateColision(glm::vec4 verticeTrans, int d, glm::vec3 normal) {

	float impulse;
	float e = 1;
	glm::vec3 pa = TheCube->vel + glm::cross(TheCube->velAngular, (glm::vec3(verticeTrans) - TheCube->posCentroMasa));
	float vRel = glm::dot(normal, pa);
	
	glm::vec3 segundoCarro = TheCube->INVinertiaTensor*(glm::cross(glm::vec3(verticeTrans), normal));

	impulse = (-(1 + e)*(vRel) / ((1 / TheCube->masa) + glm::dot(normal,glm::cross(segundoCarro, glm::vec3(verticeTrans)))+0));

	glm::vec3 J = impulse*normal;
	glm::vec3 tImpulse = glm::cross(glm::vec3(verticeTrans), J);

	TheCube->linearMomentum = TheCube->linearMomentum + J;
	TheCube->angularMomentum = TheCube->angularMomentum + tImpulse;
}

void CheckColision(glm::vec4 verticeTrans) {
	glm::vec4 hola = verticeTrans;
	
	//FLOOR
	if (verticeTrans.y <= 0) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { 0,1,0 };
		d = 0;
		UpdateColision(verticeTrans, d, normal);
	}
	//LEFT WALL
	if (verticeTrans.x <= -5) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { 1,0,0 };
		d = 5;
		UpdateColision(verticeTrans, d, normal);
	}
	//RIGHT WALL
	else if (verticeTrans.x >= 5) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { -1,0,0 };
		d = 5;
		UpdateColision(verticeTrans, d, normal);
	}
	//FRONT WALL
	else if (verticeTrans.z <= -5) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { 0,0,1 };
		d = 5;
		UpdateColision(verticeTrans, d, normal);
	}
	//BACK WALL
	else if (verticeTrans.z >= 5) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { 0,0,-1 };
		d = 5;
		UpdateColision(verticeTrans, d, normal);
	}
	//TOP WALL
	else if (verticeTrans.y >= 10) {
		glm::vec3 normal = { 0,0,0 };
		float d;
		normal = { 0,-1,0 };
		d = 10;
		UpdateColision(verticeTrans, d, normal);
	}
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

	for (int i = 0; i < 8; i++) {
		glm::vec4 verticeTrans = matIdentidad * glm::vec4(verts[i],1.0f);
		CheckColision(verticeTrans);
	}

	if (t % 150 == 0){
		PhysicsInit();
	}
	t++;
	Cube::updateCube(matIdentidad);
}
void PhysicsCleanup() {

}