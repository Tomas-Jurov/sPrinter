#ifndef SRC_PRINTER_JOINT_POSITIONS_H
#define SRC_PRINTER_JOINT_POSITIONS_H

#include <vector>

std::vector<double> joint_positions_home_ = { 0.0, 0.0, 0.08, 0.0, 0.0 };
std::vector<double> joint_positions_idle1_ = { 0.0, 0.0, 0.615, 0.0, 0.0 };
std::vector<double> joint_positions_idle2_ = { 0.0, 0.0, 0.615, 0.0, 1.15 };

#endif  // SRC_PRINTER_JOINT_POSITIONS_H
