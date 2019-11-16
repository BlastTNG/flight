#include "logscroll.h"
#include "ui_logscroll.h"

Logscroll::Logscroll(QWidget *parent, QString cmd) :
    QWidget(parent),
    ui(new Ui::Logscroll)
{
    ui->setupUi(this);

    process = new QProcess(this);
    cmdstring = cmd;

    connect(process, &QProcess::readyReadStandardOutput, this, &Logscroll::processStandardOutput);
    connect(process, &QProcess::readyReadStandardError, this, &Logscroll::processStandardError);
    connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this, &Logscroll::close);

    QSettings settings("SuperBIT", "guaca");
    restoreGeometry(settings.value(cmdstring).toByteArray());

    process->start(cmdstring);
}

Logscroll::~Logscroll()
{
    qDebug() << "Destroying log";
    savePosition();
    process->kill();
    delete ui;
}

void Logscroll::savePosition()
{
    QSettings settings("SuperBIT", "guaca");
    settings.setValue(cmdstring, saveGeometry());
}

void Logscroll::closeEvent(QCloseEvent *event)
{
    savePosition();
    QWidget::closeEvent(event);
}

void Logscroll::processStandardOutput() {
    ui->logText->moveCursor (QTextCursor::End);
    QString text = process->readAllStandardOutput();
    ui->logText->insertPlainText(text);
    ui->logText->moveCursor (QTextCursor::End);
}


void Logscroll::processStandardError() {
    ui->logText->moveCursor (QTextCursor::End);
    QString text = process->readAllStandardError();
    ui->logText->insertPlainText(text);
    ui->logText->moveCursor (QTextCursor::End);
}
