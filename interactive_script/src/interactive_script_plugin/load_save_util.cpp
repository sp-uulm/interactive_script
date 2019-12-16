#include <interactive_script/load_save_util.h>
#include <QFileDialog>
#include <iostream>
#include <fstream>

namespace load_save {

std::optional<Filename> save_dialog(bool blockly_perspective) {
    QString selected_filter = "Blockly Xml (*.xml)";
    if (!blockly_perspective) {
        selected_filter = "Lua Script (*.lua)";
    }

    Filename result;

    result.name = QFileDialog::getSaveFileName(nullptr, "Save script", "", "Lua Script (*.lua);;Blockly Xml (*.xml)", &selected_filter).toStdString();
    result.format = (selected_filter == "Blockly Xml (*.xml)" ? Filename::XML : Filename::LUA);

    if (result.name.empty())
        return std::nullopt;

    return std::move(result);
}

std::optional<Filename> load_dialog() {

    QString name = QFileDialog::getOpenFileName(nullptr, "Load script", "", "Supported Scripts (*.lua *.xml)");

    Filename result;
    result.name = name.toStdString();
    result.format = (name.endsWith(".lua") ? Filename::LUA : Filename::XML);

    if (result.name.empty())
        return std::nullopt;

    return std::move(result);
}

void save_file(const Filename& filename, const std::string& content) {
    std::cout << "Saving file " << filename.name << (filename.format == Filename::LUA ? " (lua) " : " (xml) ") << std::endl;

    std::ofstream out(filename.name, std::ios_base::out);
    out << content;
    out.close();
}

std::string load_file(const Filename& filename) {
    std::cout << "Loading file " << filename.name << (filename.format == Filename::LUA ? " (lua) " : " (xml) ") << std::endl;

    std::string content, buffer;
    std::ifstream in(filename.name, std::ios_base::in);
    while (std::getline(in, buffer)) {
        content += buffer;
    }
    in.close();

    return content;
}

}
