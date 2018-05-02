//
// Created by matthew on 5/2/18.
//

#include <vector>
#include <rover_navigation/lines_layer.h>

void rover_navigation::LineLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<Point> &cells) {
    // totally not stolen from another package because i couldn't be bothered to write out a line algorithm
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    Point pt = {};
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
}

