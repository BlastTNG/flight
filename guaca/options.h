#ifndef OPTIONS_H
#define OPTIONS_H

#include <QObject>
#include <QWidget>
#include <QDialog>
#include <QAbstractButton>
#include <QMessageBox>
#include <QDebug>
#include <QCheckBox>
#include <QTableWidget>
#include <QSettings>
#include <QFile>
#include <QCompleter>
#include <QFileSystemModel>

#include "logscroll.h"

namespace Ui {
class Options;
}

class Helper {
    public:
        Helper(QTableWidget *tw, int index);
        Helper(QString cmdname, QString args, bool terminal);
        ~Helper();

        Logscroll * log;

        QString cmdname;
        QString args;
        bool terminal;
};

class Options : public QDialog
{
    Q_OBJECT
public:
    explicit Options(QWidget *parent = nullptr);
    ~Options();

    void start_helpers();
    void start_a_helper(Helper * helper);
    void show_helpers();
    void hide_helpers();
    void load_options();
    void restore_options();
    void default_options();
    void apply_options();
    void save_options();
    void enable_options();
    void disable_options();
    unsigned int add_helper(QString cmdname, QString args, bool terminal, unsigned int row);
    unsigned int remove_helper(unsigned int row);


    bool server;
    bool client;
    bool backup;
    bool no_checksums;
    bool mole_terminal;

    unsigned int server_port;
    unsigned int client_port;

    bool auto_live;
    QString live_name;
    QString mole_dir;
    QString raw_dir;

private:
    QWidget * main;
    Ui::Options * ui;
    std::vector<Helper *> helpers;
    QSettings settings;
    QCompleter * completer;
    QFileSystemModel * fsmodel;

signals:

public slots:

private slots:
    void on_addHelper_clicked();
    void on_deleteHelper_clicked();
    void on_buttonBox_clicked(QAbstractButton *button);
    void on_auto_live_toggled(bool checked);
};

#endif // OPTIONS_H
