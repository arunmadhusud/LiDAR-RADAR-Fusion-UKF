#include "plot.h"
#include <iostream>

void plotNIS(const std::vector<double>& nis, double threshold, std::string id) {
    try {
        
        
        py::module sys = py::module::import("sys"); // Import the sys module of Python
        sys.attr("path").attr("append")("/home/arun/SFND_Unscented_Kalman_Filter/src"); // Append the path to the Python sys.path list
   
        py::module plot_module = py::module::import("plot"); // Import the plot module

        py::object plot_nis = plot_module.attr("plot_nis"); // Get the plot_nis function from the plot module

        py::list py_nis; // Convert the C++ vector to a Python list for plotting
        for (const auto& val : nis) {
            py_nis.append(val);
        }

        plot_nis(py_nis, threshold , id); // Call the plot_nis function with the NIS values and threshold
    } 
    catch (const py::error_already_set &e) {
        std::cerr << "Python error: " << e.what() << std::endl;
        return;
    }

    return;
}
