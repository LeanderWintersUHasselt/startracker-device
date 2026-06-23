#pragma once
#include "common/Config.hpp"
#include <string>

namespace util {

// Laad parameters vanuit een JSON-bestand en overschrijf de defaults in AppConfig.
// Sleutels die ontbreken in het bestand worden overgeslagen → default blijft.
// Geeft true als het bestand geladen werd, false als het niet bestaat (geen fout).
bool loadConfig(const std::string& path, AppConfig& cfg);

// Sla de huidige AppConfig op als JSON (handig om een startbestand te genereren).
void saveConfig(const std::string& path, const AppConfig& cfg);

} // namespace util
