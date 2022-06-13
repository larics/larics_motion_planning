#ifndef CONFIG_PATH_UTIL_H
#define CONFIG_PATH_UTIL_H

#include <string>
#include <fstream>

namespace motion_util {

/**
 * @brief Return true if file exists, otherwise false.
 *
 */
inline bool fileExists(const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

/**
 * @brief Get the User Prefix object
 * 
 */
inline std::string getUserPrefix()
{
  if (getenv("ABSOLUTE_CONFIG") != nullptr)
  {
    return {};
  }

  std::string username = "/home/";
  username = username + getenv("USER") + "/";
  return username;
}

/**
 * @brief Load path using get_param functor. If it fails try the
 * backup path using a defined environment variable. If that fails - throw.
 *
 */
template<typename F> std::string loadPathOrThrow(F get_path, const char* env_backup, const std::string& param_name="config")
{
  std::string config_file;
  try {

    // Try loading using the given functor...
    config_file = get_path();

  } catch (const std::exception& e) {

    // If it throws, notify the user...
    std::cerr << "Error while trying to load " << param_name << " path: " << e.what() << '\n';
    std::cout
      << "Try to load through environment variable ${" << env_backup << "}\n";

    // ... and try loading using the env file
    auto* config_ptr = getenv(env_backup);

    if (config_ptr == nullptr) {
      throw std::runtime_error("Did not get " + param_name + " file, nor ${"
                               + std::string(env_backup) + "} env. Throwing...");
    }

    config_file = std::string(config_ptr);
  }

  std::cout << "Loaded path [" << param_name << "]:\n" << config_file << "\n";
  return config_file;
}
}// namespace motion_util

#endif /*CONFIG_PATH_UTIL_H*/