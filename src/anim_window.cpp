#include <cstdio>
#include <cmath>
#include <matplot/matplot.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world quad_anim package\n");

  std::vector<double> t = matplot::iota(0, matplot::pi / 50, 10 * matplot::pi);
  std::vector<double> st = matplot::transform(t, [](auto x)
                                              { return sin(x); });
  std::vector<double> ct = matplot::transform(t, [](auto x)
                                              { return cos(x); });
  auto l = matplot::plot3(st, ct, t);
  matplot::show();

  return 0;
}
