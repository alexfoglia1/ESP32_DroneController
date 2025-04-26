#ifndef COMMANDLINEEDIT_H
#define COMMANDLINEEDIT_H

#include <QLineEdit>
#include <QStringList>

class CommandLineEdit : public QLineEdit
{
    Q_OBJECT

public:
    CommandLineEdit(QWidget* parent = nullptr);

    void addToHistory(const QString& command);

protected:
    void keyPressEvent(QKeyEvent* event) override;

private:
    QStringList _commandHistory;
    int _historyIndex;
};

#endif // COMMANDLINEEDIT_H
