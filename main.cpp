#include <vector>
#include <cmath>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

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

Vec3f cross(Vec3f vect_A, Vec3f vect_B) {

    Vec3f cross_P;
    cross_P.x = vect_A.y * vect_B.z - vect_A.z * vect_B.y;
    cross_P.y = vect_A.x * vect_B.z - vect_A.z * vect_B.x;
    cross_P.z = vect_A.x * vect_B.y - vect_A.y * vect_B.x;
    return cross_P;
}

Vec3f barycentric(Vec2i *pts, Vec2i P) {
    Vec3f u = cross(Vec3f(pts[2].x-pts[0].x, pts[1].x-pts[0].x, pts[0].x-P.x), Vec3f(pts[2].y-pts[0].y, pts[1].y-pts[0].y, pts[0].y-P.y));
    /* `pts` and `P` has integer value as coordinates
       so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
    Vec2i bboxmin(image.get_width()-1,  image.get_height()-1);
    Vec2i bboxmax(0, 0);
    Vec2i clamp(image.get_width()-1, image.get_height()-1);
    for (int i=0; i<3; i++) {
        bboxmin.x = std::max(0,        std::min(bboxmin.x, pts[i].x));
        bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
        bboxmin.y = std::max(0,        std::min(bboxmin.y, pts[i].y));
        bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
    }
    Vec2i P;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f bc_screen  = barycentric(pts, P);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            image.set(P.x, P.y, color);
        }
    }
}


int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("object/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0,0,-1);

    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
            world_coords[j]  = v;
        }
        Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
        n.normalize();
        float intensity = n*light_dir;
        if (intensity>0) {
            triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        }
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
