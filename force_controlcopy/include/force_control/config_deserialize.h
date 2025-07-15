#pragma once
#include <yaml-cpp/yaml.h>

template <typename T>
bool deserialize(const YAML::Node& node, T& config);