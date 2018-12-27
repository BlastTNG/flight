#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdint.h>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QInputDialog>
#include <QtConcurrent/QtConcurrent>
#include <QTest>

#define MAX_NUM_LINKFILE 256
#define GUACAPORT 31413
#define MAXLINELENGTH 256
#define IMAGE_IND_OFF 0
#define IMAGE_LOOP_LOW 19
#define IMAGE_LOOP_HIGH 23
#define IMAGE_TOTAL 32

struct GUACACONFIG
{
    int linkindex;
    int hostindex;
    int checksum;
    int server;
    int backup;
    int rewind;
    int active;
    int multilinknum;
    int multilinkindex[MAX_NUM_LINKFILE];
    char customhost[64];
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_toggleMole_clicked();
    void dancing();

    void on_multiLinkSelect_itemSelectionChanged();

    void on_linkSelect_currentIndexChanged(const QString &arg1);

    void on_remoteHost_activated(const int &arg1);

    void change_remote_host(const QString &arg);

    void on_actionClose_triggered();

    void on_actionSlave_to_triggered();

    void on_actionAbout_triggered();

    void on_actionAbout_Qt_triggered();

private:
    int num_linkfile;
    int linkids[3];
    int syncstate;
    QString linkfile[MAX_NUM_LINKFILE];
    Ui::MainWindow *ui;
    unsigned int mole_active;
    struct GUACACONFIG cfg;
    void freeze();
    void unfreeze();
    void start_a_mole(int );
    int get_server_data();
    void updateSettings();
    void getSettings();
    QIcon qi[IMAGE_TOTAL];
    QSize qs;

    QTimer * _ut;
    int image_i, inc;
    FILE * logfile;
		FILE * statfile;
    char buf[MAXLINELENGTH+5];
    uint64_t prev_size;
		int logend;
    int data_incoming;
		char gnd_ip[128];
		int servermode;
		QFuture<void> f1;
};

#endif // MAINWINDOW_H
