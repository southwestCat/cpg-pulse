#include "DebugDraw.h"

#define WHITE 0xFFFFFF
#define BLACK 0x000000
#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF

DebugDraw::DebugDraw(webots::Display *dis) : display(dis)
{
    height = display->getHeight();
    width = display->getWidth();
}

void DebugDraw::drawAxis()
{
    display->setColor(WHITE);
    display->drawLine(0, height / 2, width, height / 2);
    display->drawLine(width / 2, 0, width / 2, height);
}

void DebugDraw::setAxis(float xmin, float xmax, float ymin, float ymax)
{
    this->xmin = xmin;
    this->xmax = xmax;
    this->ymin = ymin;
    this->ymax = ymax;
}

void DebugDraw::fillRectangle(float x, float y, float w, float h)
{
    int xi = x / xmax * (width / 2) + origin.x();
    int yi = origin.y() - (y / ymax * (height / 2));
    int wi = w / xmax * width / 2;
    int hi = h / ymax * height / 2;
    display->fillRectangle(xi, yi, wi, hi);
}

void DebugDraw::clear(int color)
{
    display->setColor(color);
    display->fillRectangle(0, 0, width, height);
}

void DebugDraw::setColor(int color)
{
    display->setColor(color);
}

void DebugDraw::fillCircle()
{
    // display->fillOval();
    // display->drawOval();
}

void DebugDraw::drawOval(float cx, float cy, float a, float b)
{
    int xi = cx / xmax * (width / 2) + origin.x();
    int yi = origin.y() - (cy / ymax * (height / 2));
    int ai = a / xmax * width / 2;
    int bi = b / ymax * height / 2;
    display->drawOval(xi, yi, ai, bi);
}
