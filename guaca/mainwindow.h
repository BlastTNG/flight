#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdint.h>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QInputDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTest>
#include <QCloseEvent>
#include <QListWidgetItem>

#include "options.h"

#define MAX_NUM_LINKFILE 256
//#define GUACAPORT 31413
#define GUACAPORT 40204
#define MAXLINELENGTH 256
#define IMAGE_IND_OFF 0
#define IMAGE_LOOP_LOW 19
#define IMAGE_LOOP_HIGH 23
#define IMAGE_TOTAL 32

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    unsigned int mole_active;

private slots:
    void on_toggleMole_clicked();

    void dancing();

    void make_listfiles();

    void on_linkSelect_currentIndexChanged(const QString &arg1);

    bool change_remote_host(const QString &arg);

    void on_actionClose_triggered();

    void on_actionSlave_to_triggered();

    void on_actionAbout_triggered();

    void on_actionAbout_Qt_triggered();

    void on_actionOptions_triggered();

    void on_actionPurge_old_data_triggered();

    void on_actionClear_remote_hosts_triggered();

    void on_hosts_activated(int index);

    void on_linkSelect_activated(const QString &arg1);

    void on_multiLinkSelect_itemSelectionChanged();

    int add_a_host(const QString &thehost);

private:
    int num_linkfile;
    int syncstate;
    QString linkfile[MAX_NUM_LINKFILE];
    Ui::MainWindow *ui;
    Options *options;
    std::vector<Logscroll *> mole_logs;
    bool still_dancing;
    QSettings settings;
    bool servermode;

    void freeze();
    void unfreeze();
    void start_a_mole(int );
    void stop_all_moles();
    int get_server_data();
    void savePosition();
    void auto_select_link();

    void saveConfig();
    void loadConfig();
    void defaultConfig();

    void closeEvent(QCloseEvent *event);

    QIcon qi[IMAGE_TOTAL];
    QSize qs;

    QTimer * _ut, * _ut_listfiles;
    int image_i, inc;
    FILE * logfile;
    FILE * statfile;
    char buf[MAXLINELENGTH+5];
    uint64_t prev_size;
    int logend;
    int data_incoming;
    char gnd_ip[128];
    QFuture<void> f1;

    int host_index;
    QString linkItem;
    QStringList linkSelect;
    bool has_warned;
};

#endif // MAINWINDOW_H
