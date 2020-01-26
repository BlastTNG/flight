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
#include <QAction>
#include <QMenu>
#include <QComboBox>

#include "options.h"
#include "server.h"

#define MAX_NUM_LINKFILE 256
#define MAXLINELENGTH 256
#define IMAGE_IND_OFF 0
#define IMAGE_LOOP_LOW 19
#define IMAGE_LOOP_HIGH 23
#define IMAGE_TOTAL 32

class HostMenu : public QMenu
{
    Q_OBJECT

public:
    explicit HostMenu(QComboBox *parent);
    ~HostMenu();

private:
    QAction *new_item;
    QAction *modify_item;
    QAction *delete_item;
    QAbstractItemView* view;
    QComboBox *combo;

    bool eventFilter(QObject *o, QEvent *e);

private slots:
    void list_context_menu(QPoint pos);
    void handle_host_menu(QAction *);
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
    bool mole_active;

private slots:
    void on_toggleMole_clicked();

    void dancing();

    int make_listfiles();

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

    void on_computeFrameMap_clicked();

private:
    int num_linkfile;
    QString linkfile[MAX_NUM_LINKFILE];
    Ui::MainWindow *ui;
    Options *options;
    HostMenu *host_menu;
    std::vector<Logscroll *> mole_logs;

    Server *server;
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
    char buf[MAXLINELENGTH+5];
    uint64_t prev_size;
    uint64_t logend;
    bool data_incoming;

    int host_index;
    QString linkItem;
    QStringList linkSelect;
    bool has_warned;
    QString last_msg;
};

#endif // MAINWINDOW_H
