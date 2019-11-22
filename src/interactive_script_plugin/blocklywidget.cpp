#include "interactive_script/blocklywidget.h"

BlocklyWidget::BlocklyWidget(QWidget *parent) : QWebEngineView(parent)
{
    load(QUrl("file:///home/thomas/catkin_ws/src/blockly/index.html"));

    bridge = new BlocklyBridge();
    channel = new QWebChannel(this);

    channel->registerObject("bridge", bridge);
    page()->setWebChannel(channel);
}

void BlocklyWidget::setEditor(QPlainTextEdit *editor) {
    bridge->setEditor(editor);
}

void BlocklyWidget::setBlockValue(QString id, QString field, QString value) {
    emit bridge->set_field_value(id, field, value);
}
