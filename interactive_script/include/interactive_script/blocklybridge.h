#ifndef BLOCKLYBRIDGE_H
#define BLOCKLYBRIDGE_H

#include <iostream>
#include <QObject>
#include <QPlainTextEdit>

class BlocklyBridge : public QObject
{
    Q_OBJECT
public:
    explicit BlocklyBridge(QObject *parent = nullptr);
    void setEditor(QPlainTextEdit *editor);

    QString current_xml = "";

private:
    QPlainTextEdit *editor = nullptr;

signals:
    void highlight_block(QString id);
    void highlight_field(QString id, QString field);
    void remove_highlights();
    void set_field_value(QString id, QString field, QString value);

public slots:
    void log(const QString& msg);
    void on_event(const QString& type, const QString& workspace_id, const QString& block_id, const QString& group_id);
    void state_changed(const QString& xml, const QString& lua);
};

#endif // BLOCKLYBRIDGE_H
