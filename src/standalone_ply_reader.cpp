//
// Created by olaghon 9/8/24.
//
#include "happly.h"
#include <iostream>
#include <vector>
#include <string>

std::tuple<double, double, double> calculate_rgb(double f_dc_0, double f_dc_1, double f_dc_2) {
    const double SH_C0 = 0.28209479177387814;
    double r = 0.5 + SH_C0 * f_dc_0;
    double g = 0.5 + SH_C0 * f_dc_1;
    double b = 0.5 + SH_C0 * f_dc_2;
    return std::make_tuple(r, g, b);
}

int main() {
    // Load the PLY file
    happly::PLYData plyData("/home/olagh48652/rss_paper/data/train/point_cloud/iteration_7000/point_cloud.ply");

    // Extract vertex positions
    std::vector<std::array<double, 3>> vertices = plyData.getVertexPositions();
    // new addition to hpp happly
//    std::vector<std::array<double, 3>> color = plyData.getVertexColorsfdc();

    // Extract custom fields (e.g., color components f_dc_0, f_dc_1, f_dc_2)
    std::vector<double> f_dc_0 = plyData.getElement("vertex").getProperty<double>("f_dc_0");
    std::vector<double> f_dc_1 = plyData.getElement("vertex").getProperty<double>("f_dc_1");
    std::vector<double> f_dc_2 = plyData.getElement("vertex").getProperty<double>("f_dc_2");

    double R, G, B;
    // Print the extracted data
    for (size_t i = 0; i < 5 ; ++i) { //vertices.size()
        std::cout << "Vertex " << i << ": Position (" << vertices[i][0] << ", " << vertices[i][1] << ", " << vertices[i][2] << ")";
//        std::cout << ": Color (" << color[i][0] << ", " << color[i][1] << ", " << color[i][2] << ")\n";
//        std::cout << ", Color (f_dc_0: " << f_dc_0[i] << ", f_dc_1: " << f_dc_1[i] << ", f_dc_2: " << f_dc_2[i] << ")\n";
//        std::cout << ", rest (f_rest_0: " << f_rest_0[i] << ")\n";
//
        std::tie(R, G, B) = calculate_rgb(f_dc_0[i], f_dc_1[i], f_dc_2[i]);

        std::cout << "R: " << R << "\n";
        std::cout << "G: " << G << "\n";
        std::cout << "B: " << B << "\n";
    }

    return 0;
}
