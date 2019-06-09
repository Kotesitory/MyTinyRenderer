#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 200;
const int height = 200;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x=x0; x<=x1; x++) {
        float t = (x-x0)/(float)(x1-x0);
        int y = y0*(1.-t) + y1*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void line(Vec2i v0, Vec2i v1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(v0.x-v1.x)<std::abs(v0.y - v1.y)) {
        std::swap(v0.x, v0.y);
        std::swap(v1.x, v1.y);
        steep = true;
    }
    if (v0.x > v1.x) {
        std::swap(v0.x, v1.x);
        std::swap(v0.y, v1.y);
    }

    for (int x=v0.x; x<=v1.x; x++) {
        float t = (x-v0.x)/(float)(v1.x-v0.x);
        int y = v0.y*(1.-t) + v1.y*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    //Checking for outlier triangles
    if(t0.y == t1.y && t0.y == t2.y)
        return;

    //Sorting points by height in order do divide triangle in 2 triangles with one horizontal side
    if (t0.y>t1.y) std::swap(t0, t1);
    if (t0.y>t2.y) std::swap(t0, t2);
    if (t1.y>t2.y) std::swap(t1, t2);

    int partTriangleHeight, wholeTriangleHeight = t2.y - t0.y;
    //Iterating through y values in the triangle like in scan conversion
    for(int y = 0; y < wholeTriangleHeight; y++){
        bool secongTriangle = y > t1.y - t0.y || t1.y ==  t0.y;
        partTriangleHeight = secongTriangle? t2.y - t1.y : t1.y - t0.y; //Checking which part triangle is being drawn ATM
        //Calculating scalars for determining limiting points in the current horizontal segment
        float p1 = (float)y / wholeTriangleHeight;
        float p2 = (float)(y - (secongTriangle? t1.y - t0.y : 0))/ partTriangleHeight;

        //Calculating limiting vectors (vertices) by getting the vector of the 2 barrieres (left and right)
        //multipying it by the scalars and then moving it in reference to t0 instead of the origin point
        Vec2i Left = t0 + (t2 - t0) * p1;
        Vec2i Right = secongTriangle? t1 + (t2 - t1) * p2 : t0 + (t1 - t0) * p2;
        if(Left.x > Right.x)
            std::swap(Left, Right);

        //Drawing one horizontal line segment between the limiting points "Left" and "Right"
        for(int x = Left.x; x <= Right.x; x++){
            image.set(x, y + t0.y, color);
        }
    }
}

void triangle_outline(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
    line(t0, t1, image, color);
    line(t1, t2, image, color);
    line(t2, t0, image, color);
}


int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("object/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);

    triangle(Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80), image, red);

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
