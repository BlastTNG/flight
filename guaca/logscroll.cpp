#include "logscroll.h"
#include "ui_logscroll.h"

Logscroll::Logscroll(QWidget *parent, QString cmd, bool save_position) :
    QWidget(parent),
    ui(new Ui::Logscroll)
{
    ui->setupUi(this);
    this->save_position = save_position;

    process = NULL;
    cmdstring = cmd;
    startProcess();

    QSettings settings("guacamole", "guaca");
    restoreGeometry(settings.value(cmdstring).toByteArray());
}

Logscroll::~Logscroll()
{
    qDebug() << "Destroying log";
    savePosition();
    stopProcess();
    if (ui) delete ui;
    ui = NULL;
    printf("Destroyed log\n");
}

void Logscroll::stopProcess() {
    if (process) {
        process->kill();
        delete process;
    }
    process = NULL;
}

bool Logscroll::doneProcess() {
    return (process->state() == QProcess::NotOpen);
}

void Logscroll::startProcess() {
    stopProcess();

    process = new QProcess(this);

    connect(process, &QProcess::readyReadStandardOutput, this, &Logscroll::processStandardOutput);
    connect(process, &QProcess::readyReadStandardError, this, &Logscroll::processStandardError);
    connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this, &Logscroll::close);

    process->start(cmdstring);
    qDebug() << "Starting process: " + cmdstring;
}

void Logscroll::savePosition()
{
    if (save_position) {
        QSettings settings("guacamole", "guaca");
        settings.setValue(cmdstring, saveGeometry());
    }
}

void Logscroll::closeEvent(QCloseEvent *event)
{
    qDebug() << cmdstring << " told to close";
    savePosition();
    QWidget::closeEvent(event);
}

void Logscroll::processStandardOutput() {
    ui->logText->moveCursor(QTextCursor::End);
    latest_str = process->readAllStandardOutput();
    ui->logText->insertPlainText(latest_str);
    if (*latest_str.end() == '\r') {
        printf("line!\n");
        ui->logText->moveCursor(QTextCursor::StartOfLine);
    }
    ui->logText->moveCursor (QTextCursor::End);
}


void Logscroll::processStandardError() {
    ui->logText->moveCursor(QTextCursor::End);
    latest_str = process->readAllStandardError();
    ui->logText->insertPlainText(latest_str);
    ui->logText->moveCursor(QTextCursor::End);
}

QString Logscroll::get_latest_str() {
    return latest_str;
}
