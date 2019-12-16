#include "interactive_script/blocklybridge.h"

BlocklyBridge::BlocklyBridge(QObject *parent) : QObject(parent)
{

}

void BlocklyBridge::log(const QString& msg) {
    std::cout << msg.toStdString() << std::endl;
}

void BlocklyBridge::on_event(const QString& type, const QString& workspace_id, const QString& block_id, const QString& group_id) {
    std::cout << type.toStdString() << " " << block_id.toStdString() << std::endl;

//    if (type == "create") {
//        highlight_block(block_id);
//    }

//    if (type == "move") {
//        set_field_value(block_id, "42", "NUM");
//    }
}

void BlocklyBridge::state_changed(const QString& xml, const QString& lua) {
    std::cout << xml.toStdString() << std::endl;
    std::cout << lua.toStdString() << std::endl;
    if (editor) {
        editor->setPlainText(lua);
    }
    current_xml = xml;
}

void BlocklyBridge::setEditor(QPlainTextEdit *editor) {
    this->editor = editor;
}
