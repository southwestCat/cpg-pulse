#pragma once

#include <webots/Display.hpp>
#include "Tools/Math/Eigen.h"

class DebugDraw
{
public:
    DebugDraw(webots::Display *dis);
    void drawAxis();
    int getHeight() { return height; }
    int getWidth() { return width; }
    void setOrigin(int x, int y) { origin.x() = x; origin.y() = y; }
    void setAxis(float xmin, float xmax, float ymin, float ymax);
    void fillRectangle(float x, float y, float w, float h);
    void clear(int color);
    void setColor(int color);
    void fillCircle();
    // void drawOval(int cx, int cy, int a, int b);
    void drawOval(float cx, float cy, float a, float b);

private:
    int height;
    int width;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    Vector2i origin;
    webots::Display* display;
};