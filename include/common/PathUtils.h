#pragma once

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include <cstdlib>

namespace PathUtils {

namespace detail {
inline void addUnique(std::vector<std::filesystem::path>& roots,
                      const std::filesystem::path& p) {
    auto normalized = p.lexically_normal();
    auto it = std::find_if(roots.begin(), roots.end(),
                           [&](const std::filesystem::path& existing) {
                               return existing.lexically_normal() == normalized;
                           });
    if (it == roots.end()) {
        roots.push_back(normalized);
    }
}
}  // namespace detail

// Common roots that cover running from:
// - repository root
// - build/, script/, debug/ (and their subdirectories)
// - one level above either of the above
inline std::vector<std::filesystem::path> commonRoots() {
    namespace fs = std::filesystem;
    std::vector<fs::path> roots{
        fs::path("."),
        fs::path(".."),
        fs::path("../.."),
        fs::path("../../.."),
    };

    fs::path cwd = fs::current_path();
    detail::addUnique(roots, cwd);
    if (cwd.has_parent_path()) {
        detail::addUnique(roots, cwd.parent_path());
        if (cwd.parent_path().has_parent_path()) {
            detail::addUnique(roots, cwd.parent_path().parent_path());
        }
    }
    return roots;
}

inline void addRootIfSet(std::vector<std::filesystem::path>& roots,
                         const char* envVar,
                         const std::filesystem::path& subDir = {}) {
    if (!envVar) return;
    const char* raw = std::getenv(envVar);
    if (!raw) return;
    std::filesystem::path base(raw);
    detail::addUnique(roots, subDir.empty() ? base : base / subDir);
}

inline std::optional<std::filesystem::path> resolveExistingPath(
    const std::filesystem::path& requested,
    const std::vector<std::filesystem::path>& extraRoots = {},
    const std::vector<std::filesystem::path>& extraSearchDirs = {}) {
    namespace fs = std::filesystem;

    auto tryCandidate = [](const fs::path& candidate)
            -> std::optional<fs::path> {
        std::error_code ec;
        if (fs::exists(candidate, ec)) {
            auto normalized = fs::canonical(candidate, ec);
            return ec ? candidate.lexically_normal() : normalized;
        }
        return std::nullopt;
    };

    if (requested.is_absolute()) {
        if (auto hit = tryCandidate(requested)) return hit;
    }

    std::vector<fs::path> roots = commonRoots();
    for (const auto& r : extraRoots) {
        detail::addUnique(roots, r);
    }

    for (const auto& root : roots) {
        fs::path candidate = root / requested;
        if (auto hit = tryCandidate(candidate)) return hit;
    }

    // Fallback: search by filename under known roots and additional dirs.
    std::string baseName = requested.filename().string();
    if (baseName.empty()) return std::nullopt;

    std::vector<fs::path> searchDirs = extraSearchDirs;
    for (const auto& root : roots) {
        detail::addUnique(searchDirs, root);
    }

    for (const auto& dir : searchDirs) {
        std::error_code ec;
        if (!fs::exists(dir, ec) || !fs::is_directory(dir, ec)) continue;
        fs::recursive_directory_iterator it(dir, ec), end;
        for (; !ec && it != end; ++it) {
            if (!it->is_regular_file(ec)) continue;
            if (it->path().filename() == baseName) {
                return it->path();
            }
        }
    }

    return std::nullopt;
}

inline std::string resolvePathOrWarn(
    const std::string& requested,
    const std::string& description,
    const std::vector<std::filesystem::path>& extraRoots = {},
    const std::vector<std::filesystem::path>& extraSearchDirs = {}) {
    auto hit = resolveExistingPath(requested, extraRoots, extraSearchDirs);
    if (hit) {
        return hit->string();
    }
    std::cerr << "[PathUtils] warning: cannot locate " << description
              << " via \"" << requested << "\"; using the provided value as-is."
              << std::endl;
    return requested;
}

}  // namespace PathUtils
