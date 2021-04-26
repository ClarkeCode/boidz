#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include <vector>
#include "lib/ecs.hpp"
#include "lib/spatial_sets.hpp"
#include "lib/linalg.h"

//TODO: Following includes should be moved into cpp implementation file(s)
#include "lib/random_manager.hpp"   //RandomNumberFactory
#include "raylib.h"                 //renderingsystem
#include <utility>                  //move
#include <algorithm>                //for_each
#include <cmath>                    //sinf cosf etc
#include <iostream>                 //Console logging for debug

namespace boid {
	using namespace linalg::aliases;

	namespace detail {
		using linalgvec2 = linalg::aliases::float2;
		const float _pi = PI;
		//Conversion function to turn linalg vector into a raylib vector
		Vector2 compat(linalgvec2 const& vect) { return Vector2{vect.x, vect.y}; }
		//Conversion function to turn 4-length linalg floatvector into a raylib Rectangle
		Rectangle compatRect(float4 const& vect4) { return {vect4.x, vect4.y, vect4.z, vect4.w}; }
		//Conversion function to turn 4-length linalg bytevector into a raylib Color
		Color compatColour(byte4 const& vect4) { return (Color){vect4.x, vect4.y, vect4.z, vect4.w}; }
		//Produces linalg vector from a radian angle
		float2 produceUnitVector(float radianAngle) { return float2(cosf(radianAngle), sinf(radianAngle)); }
		//Produce a linalg position vector where vector is {0-maxX, 0-maxY}
		float2 produceRandomPos(randutil::RandomNumberFactory<>& randomer, float maxX, float maxY) {
			return float2(randomer.produceRandom<float>(0.0f, maxX), randomer.produceRandom<float>(0.0f, maxY));
		}
		//Normalize function safe for zero-length vectors
		float2 snormalize(float2 vect) {
			if (linalg::length(vect) != 0.0f)
				return linalg::normalize(vect);
			return vect;
		}
		//Ensures a given vector is not longer than the provided maximum length
		float2 vclamp(float2 vect, float maxVectLength) {
			if (linalg::length(vect) > maxVectLength)
				return maxVectLength * snormalize(vect);
			return vect;
		}
		//Normalizes, then scales the provided vector to the desired length
		float2 vscaleToLength(float2 vect, float desiredLength) {
			return snormalize(vect) * desiredLength;
		}

		//Conversion function to convert a degree angle to a radian scalar
		template<typename T = float> T degreeToRadian(T value) { return value * _pi / ((T) 180.0f); }
		//Conversion function to convert a radian scalar to a degree angle
		template<typename T = float> T radianToDegree(T value) { return value * ((T) 180.0f) / _pi; }

		//Return value is guaranteed to be greater or equal to the limit
		template<typename T = float>
		T constrainAbove(T value, T lowerLimit) { return (value < lowerLimit) ? lowerLimit : value; }

		//Return value is guaranteed to be less than or equal to the limit
		template<typename T = float>
		T constrainBelow(T value, T upperLimit) { return (value > upperLimit) ? upperLimit : value; }

		//Return value is guaranteed to be within the inclusive range [lowerLimit, upperLimit]
		template<typename T = float>
		T constrainWithin(T value, T lowerLimit, T upperLimit) { return constrainAbove<T>(constrainBelow<T>(value, upperLimit), lowerLimit); }
	}
	
	class SpacialComponent : public ecs::Component {
		public:
		float2 pos, vel, acc;
		float mass, maxSpeed, maxForce;
		float visionConeDegrees;
		
		SpacialComponent(float2 position, float2 velocity, float2 acceleration, float componentMass, float max_speed, float max_force, float vision_cone_degree) : 
			pos(position), vel(velocity), acc(acceleration), mass(componentMass), maxSpeed(max_speed), maxForce(max_force), visionConeDegrees(vision_cone_degree) {}
		SpacialComponent() : SpacialComponent(float2(0,0), float2(0,0), float2(0,0), 1.5f, 100.0f, 50.0f, 270.0f) {}
	};

	class VisualComponent : public ecs::Component {
		public:
		int debugLevel;
		bool isDisplayable, showDebugInfo;
		byte4 borderColour, fillColour;

		VisualComponent() : debugLevel(0), isDisplayable(true), showDebugInfo(false), borderColour(0,0,0,255), fillColour(128,128,128,255) {}
		inline void setBorderColour(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) { borderColour = byte4(r, g, b, a); }
		inline void setFillColour(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) { fillColour = byte4(r, g, b, a); }
	};

	class Boid { // : public ecs::Entity {
		public:
		using ptr_t = std::shared_ptr<Boid>;
		using Flock = std::vector<Boid::ptr_t>;

		SpacialComponent spacialInfo;
		VisualComponent visualInfo;

		Boid() = default;
		Boid(float2 startPosition, float2 startVelocity) { setPosition(startPosition); spacialInfo.vel = startVelocity; }
		inline void setPosition(float2 xy) { spacialInfo.pos = xy; }
		inline float getPosX() const { return spacialInfo.pos.x; }
		inline float getPosY() const { return spacialInfo.pos.y; }
		inline float2 getPos() const { return spacialInfo.pos; }
		inline float2 getVel() const { return spacialInfo.vel; }
		inline float2 getAcc() const { return spacialInfo.acc; }
		inline bool isWithinVision(float2 const& queriedPosition) const {
			float2 betweenVec = queriedPosition - this->spacialInfo.pos;
			float ang = angle(this->getVel(), betweenVec);
			return !(   ang > (detail::degreeToRadian(this->spacialInfo.visionConeDegrees/2)) && 
						ang < (2*3.1415f - detail::degreeToRadian(this->spacialInfo.visionConeDegrees/2)));
		}
		inline bool isWithinVision(Boid::ptr_t otherBoid) const {
			return isWithinVision(otherBoid->getPos());
		}
	};

	//Forward Declaration
	class BoidModel;

	float2 produceSteeringVector(Boid::ptr_t boidp, float2 desiredVelocity) {
		float2 steeringVector = desiredVelocity - boidp->getVel();
		if (linalg::length(steeringVector) > boidp->spacialInfo.maxForce) {
			steeringVector = detail::vscaleToLength(steeringVector, boidp->spacialInfo.maxForce);
		}
		return steeringVector;
	}

	float2 doSeek(Boid::ptr_t boidp, float2 targetPos, bool respectVision = false) {
		if (respectVision && !boidp->isWithinVision(targetPos)) return float2(0.0f);

		float2 desiredVel = detail::vscaleToLength(targetPos - boidp->getPos(), boidp->spacialInfo.maxSpeed);
		return produceSteeringVector(boidp, desiredVel);
	}
	float2 doFlee(Boid::ptr_t boidp, float2 targetPos, bool respectVision = false) {
		if (respectVision && !boidp->isWithinVision(targetPos)) return float2(0.0f);

		float2 desiredVel = detail::vscaleToLength(boidp->getPos() - targetPos, boidp->spacialInfo.maxSpeed);
		return produceSteeringVector(boidp, desiredVel);
	}

	float2 doPursue(Boid::ptr_t boidp, float2 targetPos, float2 targetVel, bool respectVision = false) {
		return doSeek(boidp, targetPos + targetVel, respectVision);
	}
	float2 doEvasion(Boid::ptr_t boidp, float2 targetPos, float2 targetVel, bool respectVision = false) {
		return doFlee(boidp, targetPos + targetVel, respectVision);
	}

	float2 doArrival(Boid::ptr_t boidp, float2 targetPos, float stoppingRadius, bool respectVision = false) {
		if (respectVision && !boidp->isWithinVision(targetPos)) return float2(0.0f);

		float2 targetOffset = targetPos - boidp->getPos();
		float distance = linalg::length(targetOffset);
		float rampedSpeed = boidp->spacialInfo.maxForce * (distance / stoppingRadius);
		float clippedSpeed = linalg::min(rampedSpeed, boidp->spacialInfo.maxSpeed);

		float2 desiredVel = (clippedSpeed / distance) * targetOffset;
		return produceSteeringVector(boidp, desiredVel);
	}

	//TODO: change the static radian angle so that it is not shared across all instances, add new component for holding local variables via maps?
	float2 doWander(Boid::ptr_t boidp, float wanderRadius, randutil::RandomNumberFactory<>& randomFactory, bool allowRendering = false) {
		static float radianAngle = 0.0f;

		radianAngle += randomFactory.produceRandom<float>(-0.3f, 0.3f);
		float2 ray = detail::produceUnitVector(radianAngle) * wanderRadius;

		if (allowRendering) {
			DrawCircleLines( (boidp->getPos()+boidp->getVel()).x, (boidp->getPos()+boidp->getVel()).y, wanderRadius, {128,128,0,255});
			DrawLineEx(detail::compat(boidp->getPos()+boidp->getVel()), detail::compat(boidp->getPos()+boidp->getVel()+ray), 1, RED);
			//DrawText(TextFormat("%f", radianAngle), 300, 10, 12, BLACK);
		}

		float2 targetPos = boidp->getPos() + boidp->getVel() + ray;
		float2 desiredVel = detail::vscaleToLength(targetPos - boidp->getPos(), boidp->spacialInfo.maxSpeed);
		return produceSteeringVector(boidp, desiredVel);
	}

	//wallCoordinates is a tuple of (x,y,width,height)
	//TODO: alter the containment behaviour to reflect steering across the radius border instead of just seeking the centre
	float2 doSquareContainment(Boid::ptr_t boidp, float4 wallCoordinates, float evasionRadius, bool allowRendering = false) {
		//float2 currentVelocity = boidp->getVel();
		//float2 futurePosition = boidp->getPos() + boidp->getVel();
		float4 evasionRectangle = {wallCoordinates.x+evasionRadius, wallCoordinates.y+evasionRadius, wallCoordinates.z-2*evasionRadius, wallCoordinates.w-2*evasionRadius};

		if (allowRendering) {
			DrawRectangleLinesEx(detail::compatRect(wallCoordinates), 3, BLACK);
			DrawRectangleLinesEx(detail::compatRect(evasionRectangle), 1, BLUE);
		}

		//If the boid is not within the area which requires steering, no steering vector is required, so exit early
		if (!spatialsets::isPointWithinDifference(boidp->getPos(), wallCoordinates, evasionRectangle)) {
			return float2(0.0f);
		}
		return doSeek(boidp, spatialsets::getCentreOfArea(wallCoordinates));
	}

	class MovementSystem : public ecs::System {
		public:
		bool positionLimitWrapping;
		float2 limitsXY;
		MovementSystem() : positionLimitWrapping(true) {}
		//TODO: May need to keep screensize information as member attributes that are initialized at construction
		virtual void process(ecs::Entity::EntityContainer const& entities) override {
			//TODO: Implement movement mehaviour of boids with SpacialComponents, rules for alignment, separation, cohesion may need to be a separate system
		}
		randutil::RandomNumberFactory<> randomFactory;
		inline void process(float frametime, float2 const& mousePos, Boid::Flock & flock) {
			//Apply forces to each member of the flock
			//forceManager.applyForces(flock);

			//flock[0]->spacialInfo.acc = float2(0.0f);
			flock[0]->spacialInfo.acc = doArrival(flock[0], mousePos, 200);
			flock[1]->visualInfo.setFillColour(0, 128, 0); flock[1]->spacialInfo.acc = doPursue(flock[1], flock[0]->getPos(), flock[0]->getVel());
			flock[2]->visualInfo.setFillColour(0, 0, 128); flock[2]->spacialInfo.acc = doFlee(flock[2], mousePos);
			flock[3]->visualInfo.setFillColour(128, 128, 0); flock[3]->spacialInfo.acc = doEvasion(flock[3], flock[0]->getPos(), flock[0]->getVel());
			flock[4]->visualInfo.setFillColour(0, 128, 128); flock[4]->spacialInfo.acc = doWander(flock[4], 100.0f, randomFactory, true);
			flock[5]->visualInfo.setFillColour(128, 0, 128); flock[5]->spacialInfo.acc = doSquareContainment(flock[5], float4(30,30,700,400), flock[5]->spacialInfo.maxSpeed, true);

			//Apply Euler-integration style movement
			std::for_each(flock.begin(), flock.end(), [this, frametime](Boid::ptr_t const& boidp)->void{
				boidp->spacialInfo.acc = detail::vclamp(boidp->getAcc() / boidp->spacialInfo.mass, boidp->spacialInfo.maxForce);
				boidp->spacialInfo.vel += frametime * boidp->spacialInfo.acc;
				boidp->spacialInfo.vel = detail::vclamp(boidp->getVel(), boidp->spacialInfo.maxSpeed);
				boidp->spacialInfo.pos += frametime * boidp->spacialInfo.vel;
				if (positionLimitWrapping) {
					if (boidp->getPosX() < 0) boidp->spacialInfo.pos.x = limitsXY.x;
					if (boidp->getPosY() < 0) boidp->spacialInfo.pos.y = limitsXY.y;
					if (boidp->getPosX() > limitsXY.x) boidp->spacialInfo.pos.x = 0.0f;
					if (boidp->getPosY() > limitsXY.y) boidp->spacialInfo.pos.y = 0.0f;
				}
			});
		}
	};

	
	class RenderingSystem : public ecs::System {
		public:
		virtual void process(ecs::Entity::EntityContainer const& entities) override {
			//TODO: Implement drawing behaviours of boids with SpacialComponents and VisualComponents
		}

		inline void render(Boid::Flock const& flock) const {
			using namespace detail;
			for (Boid::ptr_t const& boidp : flock) {
				DrawCircleV(compat(boidp->spacialInfo.pos), 5, compatColour(boidp->visualInfo.fillColour));
				DrawLineEx(compat(boidp->spacialInfo.pos), compat(boidp->spacialInfo.pos + (boidp->spacialInfo.vel)), 1, BLACK);

				if (boidp->visualInfo.showDebugInfo) {
					DrawLineEx(compat(boidp->getPos()+boidp->getVel()), compat(boidp->getPos()+boidp->getVel() + (boidp->spacialInfo.acc)), 1, GREEN);

					DrawLineEx(compat(boidp->spacialInfo.pos), compat(
						boidp->spacialInfo.pos + linalg::rot(detail::degreeToRadian(boidp->spacialInfo.visionConeDegrees/2), boidp->spacialInfo.vel)), 1, ORANGE);
					DrawLineEx(compat(boidp->spacialInfo.pos), compat(
						boidp->spacialInfo.pos + linalg::rot(2*detail::_pi - detail::degreeToRadian(boidp->spacialInfo.visionConeDegrees/2), boidp->spacialInfo.vel)), 1, PURPLE);

					for (Boid::ptr_t const& other : flock) {
						if (boidp->isWithinVision(other) && distance(boidp->getPos(), other->getPos()) < 100 ) {
							DrawLineEx(compat(boidp->getPos()), compat(other->getPos()), 1, GRAY);
						}
					}
					// DrawCircleSectorLines(compat(boidp->spacialInfo.pos), 100.0f, 0.0f, boidp->spacialInfo.visionConeDegrees, 8, BLACK);
				}
			}
		}
	};

	class BoidModel {
		public:
		float2 modelDimensions;

		Boid::Flock flock;

		MovementSystem movementSystem;
		RenderingSystem renderingSystem;

		//Amount of time in seconds before the model will allow another command to be executed
		float modelCommandCooldown;

		float2 mousePosition;

		randutil::RandomNumberFactory<float> randomManager;

		BoidModel() = delete;
		BoidModel(float2 worldDimensions) : modelDimensions(worldDimensions) {
			movementSystem.limitsXY = modelDimensions;
			modelCommandCooldown = 0.0f;
			
			for (int x = 0; x < 6; x++) { flock.push_back(std::move(std::make_shared<Boid>())); }

			this->resetPositions();

			flock[0]->spacialInfo.pos = float2(modelDimensions.x/2.0f, modelDimensions.y/2.0f);
			flock[0]->visualInfo.showDebugInfo = true;
			flock[0]->visualInfo.fillColour = {255, 0, 0, 255};
		};

		inline void resetPositions() {
			if (modelCommandCooldown == 0)  {
				modelCommandCooldown = .66f;
				randutil::RandomNumberFactory<> randomFactory;
				std::for_each(flock.begin(), flock.end(), [this, randomFactory](Boid::ptr_t& boidp)mutable->void{
					boidp->setPosition(detail::produceRandomPos(randomFactory, modelDimensions.x, modelDimensions.y));
					boidp->spacialInfo.vel = boidp->spacialInfo.maxSpeed * detail::produceUnitVector(randomFactory.produceRandom<float>(0.0f, 2.0f*detail::_pi));
				});
			}
		}

		inline void addBoids(int boids) {
			randutil::RandomNumberFactory<> randomFactory;
			while (boids-- > 0)
				flock.push_back(std::move(std::make_shared<Boid>(modelDimensions/2.0f, detail::produceUnitVector(randomFactory.produceRandom<float>(0.0f, 2.0f*detail::_pi)))));
		}
		inline void removeBoids(int boids) {
			while (boids-- > 0) 
				if (flock.size() > 0) flock.pop_back();
		}

		inline void updateModel(float frametime, float2 mouseLocation) {
			mousePosition = mouseLocation;
			modelCommandCooldown = detail::constrainAbove(modelCommandCooldown - frametime, 0.0f);
			movementSystem.process(frametime, mousePosition, flock);
		}
		inline void renderModel() const { renderingSystem.render(flock); }
	};
}

#endif