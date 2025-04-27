#include "CommandLineEdit.h"
#include <QKeyEvent>

CommandLineEdit::CommandLineEdit(QWidget* parent)
    : QLineEdit(parent), _historyIndex(-1), _isCommandFromHistory(false)
{
}

void CommandLineEdit::addToHistory(const QString& command)
{
    if (!command.isEmpty())
    {
        if (!_isCommandFromHistory)
        {
            _commandHistory.append(command);
        }
        _historyIndex = _commandHistory.size(); // Reset index
    }
}

void CommandLineEdit::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Up)
    {
        if (_historyIndex > 0)
        {
            _isCommandFromHistory = true;

            _historyIndex--;
            setText(_commandHistory.at(_historyIndex));
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (_historyIndex < _commandHistory.size() - 1)
        {
            _isCommandFromHistory = true;
            _historyIndex++;
            setText(_commandHistory.at(_historyIndex));
        }
        else
        {
            _isCommandFromHistory = false;
            _historyIndex = _commandHistory.size();
            clear();
        }
    }
    else
    {
        QLineEdit::keyPressEvent(event);
    }
}
