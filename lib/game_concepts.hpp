/*
    Game Concepts header
    Version:    2.0
*/

#ifndef GAME_CONCEPTS
#define GAME_CONCEPTS
#include "raylib.h"
#include "raylib_extensions.hpp"

namespace concept {


    //=================================================================================================================
    class GameWorld {
        public:
        float width, height;
        GameWorld(float horizontalWidth, float verticalHeight) : width(horizontalWidth), height(verticalHeight) {}
    };

    class Model {};
    //=================================================================================================================





    //Game Objects
    //=================================================================================================================
    class GameObject {
        public:
        virtual ~GameObject() = default;
    };

    class PhysicsObject : public virtual GameObject {
        public:
        Vector2 pos, vel, acc;
        virtual ~PhysicsObject() = default;
    };

    class DrawableObject : public virtual GameObject {
        public:
        virtual void drawObject() = 0;
        virtual ~DrawableObject() = default;
    };
    //=================================================================================================================

    class CollidableRectangle {
        protected:
        Rectangle rect;

        public:
        CollidableRectangle(Rectangle rect) : rect(rect) {}
        CollidableRectangle() : CollidableRectangle(extensions::RectangleZero()) {}
        
        virtual inline Rectangle getCollisionBox() const { return rect; }
        virtual ~CollidableRectangle() {}
    };

    class GeometryColours {
        public:
        Color lineColour;
        Color fillColour;

        GeometryColours(Color line, Color fill) : lineColour(line), fillColour(fill) {}
    };
}

#endif