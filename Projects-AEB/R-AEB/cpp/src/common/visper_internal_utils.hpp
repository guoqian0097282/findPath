#pragma once

#include <any>
#include <filesystem>
#include <string>
#include <unordered_map>

namespace visper::internal {

std::string NormalizeSaveDirKey(const std::filesystem::path& dir);
void PrepareSaveDirOnce(const std::string& save_dir);
void ResetPreparedSaveDirs();

double AnyToFiniteDouble(const std::any& v, double fallback = 0.0);
double GetEgoYawFromExtra(const std::unordered_map<std::string, std::any>& extra);

} // namespace visper::internal

