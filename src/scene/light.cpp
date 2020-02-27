#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>


using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3& P) const
{
	// distance to light is infinite, so f(di) goes to 0.  Return 1.
	return 1.0;
}


glm::dvec3 DirectionalLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.

	ray lightray = ray(p, -orientation, glm::dvec3(1,1,1), ray::SHADOW);
	isect i;
	if(scene->intersect(lightray, i)){
		glm::dvec3 intersectionpoint = lightray.at(i.getT());
		glm::dvec3 kt = i.getMaterial().kt(i);
		glm::dvec3 I = shadowAttenuation(r, intersectionpoint);
		return I * kt;
	}
	else{
		return color;
	}

	//create a ray from point p that goes in direction of light
	//find intersection points where the ray hits things
	//if no intersection return color of light
	//if intersection see how transmissive the material is 
	//recur to find more info
	//return color, intensity of light x material trasmissivness
	//return glm::dvec3(1.0, 1.0, 1.0);
}

glm::dvec3 DirectionalLight::getColor() const
{
	return color;
}

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3& P) const
{
	return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3& P) const
{

	// YOUR CODE HERE

	// You'll need to modify this method to attenuate the intensity 
	// of the light based on the distance between the source and the 
	// point P.  For now, we assume no attenuation and just return 1.0
	double dist = glm::distance(position, P);
	double fd = 1.0 / (constantTerm + (linearTerm * dist) + (quadraticTerm * (dist * dist)));
	return min(1.0, fd);
}

glm::dvec3 PointLight::getColor() const
{
	return color;
}

glm::dvec3 PointLight::getDirection(const glm::dvec3& P) const
{
	return glm::normalize(position - P);
}


glm::dvec3 PointLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.
	ray lightray(p, getDirection(p), glm::dvec3(1.0,1.0,1.0), ray::SHADOW);
	isect i;
	glm::dvec3 intersectionpoint = lightray.at(i);
	double dist = (p - intersectionpoint).length();
	double lightdist = (position - p).length();
	if(scene->intersect(lightray, i) && dist < lightdist){
		glm::dvec3 kt = i.getMaterial().kt(i);
		glm::dvec3 I = shadowAttenuation(r, intersectionpoint);
		return I * kt;
	}
	else{
		return color;
	}
}

#define VERBOSE 0

