#include <iostream>
#include <matplot/matplot.h>
extern "C"
{
#include <xCoords.h>
#include <yCoords.h>
#include <zCoords.h>
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    // Constants for motor positions //
    std::vector<double> pB = {0.08, 0.08, 0.015};   // pB = [lB; wB; hB];
    std::vector<double> pC = {-0.08, 0.08, 0.015};  // pC = [lC; wC; hC];
    std::vector<double> pD = {-0.08, -0.08, 0.015}; // pD = [lD; wD; hD];
    std::vector<double> pE = {0.08, -0.08, 0.015};  // pE = [lE; wE; hE];

    // Define Propeller data //
    double propDia = 0.127; // Propeller Diameter

    // Define generalized coordinates to check //
    std::vector<double> q = {0, 0, 0, 1, 0, 0, 0, M_PI_4, M_PI_2 + M_PI_4, M_PI_4, M_PI_2 + M_PI_4};

    // Get the x, y and z coordinate values //
    matplot::vector_1d x(12 * 2);
    matplot::vector_1d y(12 * 2);
    matplot::vector_1d z(12 * 2);

    std::cout << "Defined 1d vectors" << std::endl;

    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

    std::cout << "Got all coordinates" << std::endl;

    matplot::vector_2d X_base(12, std::vector<double>(2)), Y_base(12, std::vector<double>(2)), Z_base(12, std::vector<double>(2));
    matplot::vector_2d X_body(12, std::vector<double>(2)), Y_body(12, std::vector<double>(2)), Z_body(12, std::vector<double>(2));
    matplot::vector_2d X_prop(12, std::vector<double>(2)), Y_prop(12, std::vector<double>(2)), Z_prop(12, std::vector<double>(2));

    for (int i = 0; i < 4; ++i)
    {
        X_base[i][0] = x[i * 2];
        X_base[i][1] = x[i * 2 + 1];
        Y_base[i][0] = y[i * 2];
        Y_base[i][1] = y[i * 2 + 1];
        Z_base[i][0] = z[i * 2];
        Z_base[i][1] = z[i * 2 + 1];
    }

    for (int i = 4; i < 8; ++i)
    {
        X_body[i][0] = x[i * 2];
        X_body[i][1] = x[i * 2 + 1];
        Y_body[i][0] = y[i * 2];
        Y_body[i][1] = y[i * 2 + 1];
        Z_body[i][0] = z[i * 2];
        Z_body[i][1] = z[i * 2 + 1];
    }

    for (int i = 8; i < 12; ++i)
    {
        X_prop[i][0] = x[i * 2];
        X_prop[i][1] = x[i * 2 + 1];
        Y_prop[i][0] = y[i * 2];
        Y_prop[i][1] = y[i * 2 + 1];
        Z_prop[i][0] = z[i * 2];
        Z_prop[i][1] = z[i * 2 + 1];
    }

    std::cout << "Transferred all coordinate values" << std::endl;

    auto basePlot = matplot::plot3(X_base, Y_base, Z_base);
    for (auto line : basePlot)
        line->line_width(3).color("blue");
    auto bodyPlot = matplot::plot3(X_body, Y_body, Z_body);
    for (auto line : bodyPlot)
        line->line_width(3).color("blue");
    auto propPlot = matplot::plot3(X_prop, Y_prop, Z_prop);
    for (auto line : propPlot)
        line->line_width(3).color("#FFFFFF");
    matplot::axis(matplot::equal);
    matplot::xlim({-0.5, 0.5});
    matplot::ylim({-0.5, 0.5});
    matplot::zlim({-0.1, 0.5});

    std::cout << "Plot done" << std::endl;

    matplot::show();

    return 0;
}
