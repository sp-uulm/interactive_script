#ifndef LOAD_SAVE_UTIL_H
#define LOAD_SAVE_UTIL_H

#include <optional>
#include <string>

namespace load_save {

struct Filename {
    std::string name;
    enum {LUA, XML} format;
};

std::optional<Filename> save_dialog(bool blockly_perspective);

std::optional<Filename> load_dialog();

void save_file(const Filename& filename, const std::string& content);

std::string load_file(const Filename& filename);

}

#endif // LOAD_SAVE_UTIL_H
