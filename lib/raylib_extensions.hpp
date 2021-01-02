/*
    Raylib Extension header
    Version:    2.0
*/
#ifndef RAYLIB_EXTENSIONS
#define RAYLIB_EXTENSIONS
#include "raylib.h"
#include "raymath.h"

namespace extensions {
    //Rectangle related extensions
    enum RectangleSide { TOP, BOTTOM, LEFT, RIGHT };
    enum RectangleVertex { TOPLEFT, TOPRIGHT, BOTTOMLEFT, BOTTOMRIGHT };

    inline Rectangle RectangleZero() { return Rectangle{0}; }
    inline Rectangle RectangleOne() { return Rectangle{1, 1, 1, 1}; }
    inline Rectangle RectangleCentreOnCoord(Vector2 const& coord, float width, float height) {
        return Rectangle{coord.x-width/2.0f, coord.y-height/2.0f, width, height};
    }
    inline Vector2 RectangleGetCentre(Rectangle const& rect) { return Vector2{rect.x+rect.width/2.0f, rect.y+rect.height/2.0f}; }
    inline Rectangle RectangleCentreOnCoord(Vector2 const& coord, Rectangle const& rect) {
        return Rectangle{coord.x-rect.width/2.0f, coord.y-rect.height/2.0f, rect.width, rect.height};
    }
    inline Rectangle RectangleExpandByAmount(Rectangle const& rect, float amount) {
        return Rectangle{rect.x-amount, rect.y-amount, rect.width+2*amount, rect.height+2*amount};
    }
    inline Rectangle RectangleTranslateByDirection(Rectangle const& rect, Vector2 const& direction) {
        return Rectangle{rect.x + direction.x, rect.y + direction.y, rect.width, rect.height};
    }
    inline Vector2 RectangleGetVertexCoord(Rectangle const& rect, RectangleVertex const& desiredVertex) {
        if (desiredVertex == TOPLEFT)
            return Vector2{rect.x, rect.y};
        else if (desiredVertex == TOPRIGHT)
            return Vector2{rect.x+rect.width, rect.y};
        else if (desiredVertex == BOTTOMLEFT)
            return Vector2{rect.x, rect.y+rect.height};
        else
            return Vector2{rect.x+rect.width, rect.y+rect.height};
    }

    //Add the scalar value to both the vector's x and y component
    inline Vector2 Vector2AddScalar(Vector2 v, float scalar) { return Vector2{v.x + scalar, v.y + scalar}; }
    //Returns a perpendicular vector that is rotated clockwise from the argument vector
    inline Vector2 Vector2PerpendicularCW(Vector2 v) { return Vector2{v.y, -v.x}; }
    //Returns a perpendicular vector that is rotated counter-clockwise from the argument vector
    inline Vector2 Vector2PerpendicularCCW(Vector2 v) { return Vector2{-v.y, v.x}; }
    //Guarantees that the resulting vector does not have a length longer than the specified length
    inline Vector2 Vector2Clamp(Vector2 v, float maxLength) {
        if (Vector2Length(v) > maxLength) return Vector2Scale(Vector2Normalize(v), maxLength);
        return v;
    }


    inline bool isValueWithinRange(float value, float min, float max) { return value >= min && value <= max; }
    inline bool isPointWithinYValues(Vector2 const& xyCoord, Rectangle const& rect) {
        return xyCoord.y >= rect.y && xyCoord.y <= rect.y + rect.height;
    }
    inline bool isPointWithinXValues(Vector2 const& xyCoord, Rectangle const& rect) {
        return xyCoord.x >= rect.x && xyCoord.x <= rect.x + rect.width;
    }
}

#endif