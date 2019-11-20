#ifndef LOGSCROLL_H
#define LOGSCROLL_H

#include <QProcess>
#include <QDebug>
#include <QWidget>
#include <QSettings>

namespace Ui {
class Logscroll;
}

class Logscroll : public QWidget
{
    Q_OBJECT

public:
    explicit Logscroll(QWidget *parent = 0, QString cmd = "", bool save_position = true);
    QString get_latest_str();
    void startProcess();
    void stopProcess();
    bool doneProcess();
    ~Logscroll();

private:
    QString cmdstring;
    Ui::Logscroll *ui;
    QProcess * process;
    QString latest_str;
    bool save_position;
    void closeEvent(QCloseEvent *event);
    void savePosition();

private slots:
    void processStandardOutput();
    void processStandardError();
};

#endif // LOGSCROLL_H
