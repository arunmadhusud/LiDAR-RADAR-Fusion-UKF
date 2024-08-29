#ifndef PLOT_H
#define PLOT_H

#include <vector>
#include <pybind11/embed.h> // everything needed for embedding
namespace py = pybind11;

void plotNIS(const std::vector<double>& nis, double threshold, std::string id);

#endif // PLOT_H
