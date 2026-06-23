#pragma once
#include "common/Types.hpp"
#include <string>

namespace util {

// Laad K en dist vanuit een intrinsics-JSON-bestand.
// Formaat (zoals aangemaakt door build_starmap.py):
//   { "fx":..., "fy":..., "cx":..., "cy":..., "dist":[...] }
Intrinsics loadIntrinsics(const std::string& jsonPath);

// Laad sterkaart vanuit CSV (id,x,y,z — z wordt genegeerd).
StarMap loadStarMap(const std::string& csvPath);

// Sla sterkaart op in CSV.
void saveStarMap(const StarMap& map, const std::string& csvPath);

// ── StarMap3D: metrische sterkaart ────────────────────────────────────────────

// Laad StarMap3D vanuit CSV.
// Accepteert oude header (id,x,y,z) → ScaleStatus::Legacy
// en nieuwe header (id,x_m,y_m,z_m) → ScaleStatus::Metric.
StarMap3D loadStarMap3D(const std::string& csvPath,
                         StarMap3DMetadata* meta = nullptr);

// Sla StarMap3D op in CSV met header id,x_m,y_m,z_m.
void saveStarMap3D(const StarMap3D& map, const std::string& csvPath,
                   const StarMap3DMetadata* meta = nullptr);

// Converteer legacy StarMap naar StarMap3D (units onbekend, ScaleStatus::Legacy).
StarMap3D starMap3DFromLegacy(const StarMap& map);

// Converteer StarMap3D naar legacy StarMap (alleen x,y — voor backward compat).
StarMap starMapFromStarMap3D(const StarMap3D& map);

} // namespace util
