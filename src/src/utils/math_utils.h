#pragma once

struct Vector2D {
    float x, y;
    Vector2D(float x_ = 0.0F, float y_ = 0.0F) : x(x_), y(y_) {}

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    // Normalize the vector
    Vector2D normalize() const {
        float length = sqrt(x * x + y * y);
        if (length > 0) {
            return Vector2D(x / length, y / length);
        }
        return *this;
    }
};