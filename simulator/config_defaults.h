#ifndef ROADNET_CONFIG_DEFAULTS_H
#define ROADNET_CONFIG_DEFAULTS_H

#include <string>

namespace roadnet_defaults {
// -----------------------------------------------------------------------------
// Default path configuration
// -----------------------------------------------------------------------------
// Prefer command-line arguments for reproducible experiments, for example:
//   ./roadnet_sim --base ./data/Manhattan_Data --sumo-net ./test.net.xml
//
// If you do not want to pass command-line arguments, edit DEFAULT_BASE_DIR below.
// Graph::set_base_path() derives all standard BJ/SUMO input paths from this
// directory. Explicit command-line path arguments always override these defaults.
static const std::string DEFAULT_BASE_DIR = "./data/Manhattan_Data";
static const std::string DEFAULT_SUMO_NET_PATH = "./test.net.xml";
}

#endif // ROADNET_CONFIG_DEFAULTS_H
