#include "interactive_script/blocklywidget.h"
#include <ros/package.h>

BlocklyWidget::BlocklyWidget(QWidget *parent) : QWebEngineView(parent)
{
    QString path = "file://"
         + QString::fromStdString(ros::package::getPath("interactive_script_blockly"))
         + "/../quadcopter/index.html";

    std::cout << "Loading blockly URL: " << path.toStdString() << std::endl;

    load(QUrl(path));

    bridge = new BlocklyBridge();
    channel = new QWebChannel(this);

    channel->registerObject("bridge", bridge);
    page()->setWebChannel(channel);
}

void BlocklyWidget::loadXml(const QString& xml) {
    std::cout << "Blockly.Xml.clearWorkspaceAndLoadFromXml('" + xml.toStdString() + "', workspace);" << std::endl;
    page()->runJavaScript("Blockly.Xml.clearWorkspaceAndLoadFromXml(Blockly.Xml.textToDom('" + xml + "'), workspace);");
}

QString BlocklyWidget::xml() {
    return bridge->current_xml;
}

void BlocklyWidget::setEditor(QPlainTextEdit *editor) {
    bridge->setEditor(editor);
}

void BlocklyWidget::setBlockValue(QString id, QString field, QString value) {
    emit bridge->set_field_value(id, field, value);
}

void BlocklyWidget::highlightBlock(QString id) {
    emit bridge->highlight_block(id);
}

void BlocklyWidget::highlightField(QString id, QString field) {
    emit bridge->highlight_field(id, field);
}

void BlocklyWidget::removeHighlights() {
    emit bridge->remove_highlights();
}
