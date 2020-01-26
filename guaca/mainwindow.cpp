#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

#include "../liblinklist/linklist.h"
#include "../liblinklist/linklist_connect.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#define VERBOSE 0
#define SELECTION_APPEND_TEXT " - Custom selection... -"
#define GUACA_MIN_TIME_CHECK_DATA 2
#define LINKLIST_DATEFORMAT_LENGTH 20

char archivedir[128] = "/data/mole";
char moledir[128] = "/data/mole";
char masterlink[128] = "/data/etc/mole.lnk";
char masterlist[128] = "/data/etc/mole.lst";

void USAGE(void) {

  printf("\n\nGUI front-end for mole to conveniently grab telemetry data from server.\n\n"
      "Usage: guaca [host] [OPTION]...\n\n"
      " -h                 Display this message.\n"
      "\n");

    exit(0);
}

int MainWindow::make_listfiles() {
    QStringList linklist_names;
    QList<QFile*> linklist_files;

    QDir archdir(archivedir);
    archdir.setFilter(QDir::AllDirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    archdir.setSorting(QDir::Name);

    QStringList filelist = archdir.entryList();

    for (int i=0; i<filelist.size(); i++) {
        int len = filelist[i].length();
        // has the correct format (i.e. ****_yyyy-mm-dd-hh-mm-ss)
        if (len > LINKLIST_DATEFORMAT_LENGTH && filelist[i][len-LINKLIST_DATEFORMAT_LENGTH] == '_') {
            // build the linklist name and find it in the list of linklist_names
            filelist[i] = archdir.absolutePath() + "/" + filelist[i];
            QString name = QString(filelist[i]);
            name.chop(LINKLIST_DATEFORMAT_LENGTH);
            name += ".lst";
            int index = linklist_names.indexOf(name);

            // get the file handle for the given linklist name
            QFile * file;
            if (index < 0) {
                // generate a new file and append to the list
                file = new QFile();
                file->setFileName(name);
                if (!file->open(QIODevice::Truncate | QIODevice::WriteOnly)) {
                    qDebug() << "Cannot open file" << name;
                    continue;
                }
                // qDebug() << "Opened file" << name;
                linklist_files.append(file);
                linklist_names.append(name);
            } else {
                // grab the file from the array
                file = linklist_files[index];
            }
            file->write(filelist[i].toUtf8().data());
            file->write("\n");
        }
    }
    // cleanup files
    for (int i=0; i<linklist_files.size(); i++) {
        linklist_files[i]->flush();
        linklist_files[i]->close();
        delete linklist_files[i];
    }
    linklist_files.clear();
    linklist_names.clear();

    return linklist_files.size();
}


/*
 * A very important function that animates guaca when
 * appropriate. Also takes care of displaying logs in the
 * status bar.
 */

void MainWindow::dancing()
{
  int thetime = time(0);
  still_dancing = true;

  // get the latest text from the currently active mole process
  QString msg = "Mole not receiving data.\n";
  unsigned int current_index = ui->linkSelect->currentIndex();
  if (mole_logs.size() > current_index) {
      msg = mole_logs[current_index]->get_latest_str();
  }

  // check the status of data based on mole terminal text
  // this sets the data incoming status
  if ((thetime-logend) > GUACA_MIN_TIME_CHECK_DATA) {
      data_incoming = (msg != last_msg);
      last_msg = msg;
      logend = thetime;
  }
  ui->statusBar->showMessage(msg);

  // dancing logic
  if (mole_active) {
    if (data_incoming) {
      if (image_i==IMAGE_LOOP_HIGH) inc = -1;
      else if (image_i==IMAGE_LOOP_LOW) inc = 1;
    } else {
      inc = 1;
    }
  } else {
    inc = 1;
    if (image_i==(IMAGE_IND_OFF+1)%IMAGE_TOTAL) image_i = IMAGE_IND_OFF;
  }

  int ss = qMin(ui->dance->width(),ui->dance->height())*15/8;
  qs = QSize(ss,ss);
  ui->dance->setPixmap(qi[image_i].pixmap(qs).scaled(ss,ss,Qt::KeepAspectRatio));
  ui->dance->show();

  image_i = (image_i+inc)%IMAGE_TOTAL;

  still_dancing = false;
}

bool HostMenu::eventFilter(QObject *o, QEvent *e) {
  (void) o;
  if (e->type() == QEvent::MouseButtonRelease) {
    if (static_cast<QMouseEvent*>(e)->button() == Qt::RightButton) {
      return true;
    }
  }
  return false;
}

void HostMenu::list_context_menu(QPoint pos) {
  QModelIndex index = view->indexAt(pos);
  int item = index.row();
  new_item->setData(item);
  modify_item->setData(item);
  delete_item->setData(item);
  if (item == 0) {
      delete_item->setDisabled(true);
      modify_item->setDisabled(true);
  } else {
      delete_item->setEnabled(true);
      modify_item->setEnabled(true);
  }
  if (!index.isValid()) { return; }
  this->exec(view->mapToGlobal(pos));
}

HostMenu::HostMenu(QComboBox *parent) {
    combo = parent;
    view = combo->view();
    view->viewport()->installEventFilter(this);
    view->setContextMenuPolicy(Qt::CustomContextMenu);

    // build the menu
    new_item = this->addAction(QString("New..."));
    modify_item = this->addAction(QString("Modify"));
    delete_item = this->addAction(QString("Delete"));

    connect(this, SIGNAL(triggered(QAction*)), this, SLOT(handle_host_menu(QAction*)));
    connect(view, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(list_context_menu(QPoint)));
}

HostMenu::~HostMenu() {

}
void HostMenu::handle_host_menu(QAction *action) {
    int index = action->data().toInt();

    if (action == new_item) {
        bool ok = false;
        QString thehost = QInputDialog::getText(this, "New remote host", "Enter the remote host name/IP:", QLineEdit::Normal, NULL, &ok);

        if (ok) {
            // only add new host if it is not already in the list
            int existing_index = combo->findText(thehost);
            if (existing_index < 0) {
                index = index+1;
                combo->insertItem(index, thehost);
            } else {
                index = existing_index;
            }
            combo->setCurrentIndex(index);
            combo->activated(index);
        }
    } else if (action == modify_item) {
        bool ok = false;
        QString thehost = QInputDialog::getText(this, "Modify remote host", "Enter the remote host name/IP:", QLineEdit::Normal, combo->itemText(index), &ok);

        if (ok) {
            combo->removeItem(index);
            combo->insertItem(index, thehost);
            combo->setCurrentIndex(index);
            combo->activated(index);
        }

    } else if (action == delete_item) {
        if (index > 0) {
            int existing_index = combo->currentIndex();
            combo->removeItem(index);
            // check to see if the current host was removed
            if (existing_index == index) {
                combo->setCurrentIndex(index-1);
                combo->activated(index-1);
            }
        }
    }

}

/*
 * Main constructor: looks in the linklist directory for all the available
 * linklists and makes them selectable in the main link widget. Also starts
 * the server thread if necessary and the animation/log checking thread.
 */

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    settings("guacamole", "guaca")
{   
  ui->setupUi(this);

  ui->numRewind->setValidator(new QIntValidator(0, 1e8, this) );
  ui->startFrame->setValidator(new QIntValidator(0, INT32_MAX, this) );
  ui->endFrame->setValidator(new QIntValidator(0, INT32_MAX, this) );

  // setup hosts combobox contextmenu
  host_menu = new HostMenu(ui->hosts);
  options = new Options(this);

  QSettings oldsettings("SuperBIT", "guaca");
  if (settings.contains("mainwindow/customConfig")) {
      qDebug() << "Restoring saved config";
      loadConfig();
  } else if (oldsettings.contains("mainwindow/customConfig")) {
      qDebug() << "Migrating to new settings";
      QStringList items = oldsettings.allKeys();
      for (int i=0; i<items.size(); i++){
          settings.setValue(items[i], oldsettings.value(items[i]));
          qDebug() << "Transfering " << items[i];
      }
      oldsettings.clear();
      loadConfig();
  } else {
      qDebug() << "Using default config";
      defaultConfig();
      saveConfig();
  }

  // Parse arguments
  QStringList args = QCoreApplication::arguments();
  for (int i = 1; i < args.count(); i++) {
      if (QString(args[i]).startsWith('-')) {
          if (QString(args[i]) == "-h") {
              USAGE();
          } else {
              printf("\n\n");
              qDebug() << "Unrecognized option: " + args[i];
              USAGE();
          }
      } else if (i == 1) { // first argument is host
          host_index = ui->hosts->findText(args.at(1));
          if (host_index < 0) {
              ui->hosts->addItem(args.at(1));
              host_index = ui->hosts->count()-1;
          }
          ui->hosts->setCurrentIndex(host_index);
      } else {
          USAGE();
      }
  }

  // Query remote host
  if (change_remote_host(ui->hosts->currentText())) {
      saveConfig();
  }

  QString base = ":/images/guaca-";
  QString ext = ".svg";

  for (int i=0; i<IMAGE_TOTAL; i++) {
    qi[i] = QIcon(base+QString::number(i)+ext);
  }
  qs = QSize(300,300);

  ui->dance->setAlignment(Qt::AlignCenter);
  ui->dance->setPixmap(qi[IMAGE_IND_OFF].pixmap(qs));
  ui->dance->show();
  image_i = IMAGE_IND_OFF;
  inc = 1;
  srand(time(0));

  mole_active = false;
  servermode = false;
  logend = time(0);

  // Start the config server
  server = new Server(this);

  _ut = new QTimer(this);
  _ut->setInterval(90);
  connect(_ut,SIGNAL(timeout()),this,SLOT(dancing()));
  _ut->start();

  make_listfiles();
  _ut_listfiles = new QTimer(this);
  _ut_listfiles->setInterval(5000);
  connect(_ut_listfiles,SIGNAL(timeout()),this,SLOT(make_listfiles()));
  _ut_listfiles->start();

  restoreGeometry(settings.value("mainwindow").toByteArray());
}

MainWindow::~MainWindow()
{
  printf("Destructing main window\n");
  saveConfig();

  if (options) delete options;
  options = NULL;
  if (server) delete server;
  server = NULL;
  if (ui) delete ui;
  ui = NULL;
  qDebug() << "Closing";
}

void MainWindow::closeEvent(QCloseEvent *event) {
    savePosition();
    event->accept();
    _ut->stop();
    _ut_listfiles->stop();
    while(still_dancing) QTest::qWait(90);
    if (options) delete options;
    options = NULL;
}

void MainWindow::savePosition()
{
    settings.setValue("mainwindow", saveGeometry());
}

void MainWindow::saveConfig() {
    settings.setValue("mainwindow/numRewind", ui->numRewind->text().toInt());
    settings.setValue("mainwindow/smartRewind", ui->smartRewind->isChecked());
    settings.setValue("mainwindow/startFrame", ui->startFrame->text().toInt());
    settings.setValue("mainwindow/endFrame", ui->endFrame->text().toInt());
    settings.setValue("mainwindow/tabSelection", ui->tabWidget->currentIndex());

    QStringList hosts;
    for (int i = 1; i < ui->hosts->count(); i++) {
        hosts << ui->hosts->itemText(i);
    }
    settings.setValue("mainwindow/host_list", hosts);
    settings.setValue("mainwindow/host_index", ui->hosts->currentIndex());
    settings.setValue("mainwindow/linkItem", linkItem);
    settings.setValue("mainwindow/linkSelect", QVariant::fromValue(linkSelect));

    settings.setValue("mainwindow/customConfig", true);
}

void MainWindow::loadConfig() {

    ui->numRewind->setText(QString::number(QVariant(settings.value("mainwindow/numRewind")).toInt()));
    ui->smartRewind->setChecked(QVariant(settings.value("mainwindow/smartRewind")).toBool());
    ui->startFrame->setText(QString::number(QVariant(settings.value("mainwindow/startFrame")).toInt()));
    ui->endFrame->setText(QString::number(QVariant(settings.value("mainwindow/endFrame")).toInt()));
    ui->tabWidget->setCurrentIndex(QVariant(settings.value("mainwindow/tabSelection")).toInt());

    QStringList hosts = QVariant(settings.value("mainwindow/host_list")).toStringList();
    for (int i = 0; i < hosts.count(); i++) {
        ui->hosts->insertItem(i+1, hosts[i]);
    }
    host_index = QVariant(settings.value("mainwindow/host_index")).toInt();
    host_index = (host_index < (ui->hosts->count()-1)) ? host_index : (ui->hosts->count()-1);
    ui->hosts->setCurrentIndex(host_index);
    linkItem = QVariant(settings.value("mainwindow/linkItem")).toString();
    linkSelect = QStringList(QVariant(settings.value("mainwindow/linkSelect")).toStringList());

}

void MainWindow::defaultConfig() {
    qDebug() << "Default config";
    ui->numRewind->setText("100");
    ui->smartRewind->setChecked(true);
    ui->startFrame->setText("");
    ui->endFrame->setText("");
    ui->tabWidget->setCurrentIndex(0);

    ui->hosts->setCurrentIndex(ui->hosts->count()-1);
    host_index = ui->hosts->count()-1;
    linkItem = "default";
}

/*
 * Unfreezes all user selectable widgets in the GUI
 */
void MainWindow::unfreeze()
{
  if (!servermode)
  {
    ui->toggleMole->setText("Start MOLE");
    ui->multiLinkSelect->setEnabled(true);
    ui->numRewind->setEnabled(true);
    ui->hosts->setEnabled(true);
    ui->toggleMole->setEnabled(true);
    ui->linkSelect->setEnabled(true);
    ui->tabWidget->setEnabled(true);

    options->enable_options();
  }
  else
  {

    ui->multiLinkSelect->setDisabled(true);
    ui->numRewind->setDisabled(true);
    ui->hosts->setDisabled(true);
    ui->toggleMole->setDisabled(true);
    ui->linkSelect->setDisabled(true);
    ui->tabWidget->setDisabled(true);

    options->disable_options();
  }
}

/*
 * Freezes all user selectable widgets in the GUI. Leaves the
 * link selector unfrozen to allow for mole.lnk changes while
 * running.
 */
void MainWindow::freeze()
{
  ui->multiLinkSelect->setDisabled(true);
  ui->numRewind->setDisabled(true);
  ui->hosts->setDisabled(true);
  ui->tabWidget->setDisabled(true);
  ui->toggleMole->setText("Stop MOLE");
  options->disable_options();

  if (servermode)
  {
    ui->toggleMole->setDisabled(true);
    ui->linkSelect->setDisabled(true);
  }
}

/*
 * Starts a single mole client based on the selection index from
 * the linklist selector.
 */
void MainWindow::start_a_mole(int index)
{
  QString mole_cmd = "mole --live-name "+ui->linkSelect->itemText(index)+" "+
                          "--filename "+ui->linkSelect->itemText(index)+" "+
                          "@"+ui->hosts->currentText().split(QRegExp("\\s+-"), QString::SkipEmptyParts)[0]+" "+
                          "--client-port "+QString::number(options->client_port)+" --server-port "+QString::number(options->server_port)+" "+
                          "--archive-dir "+options->raw_dir+" --mole-dir "+options->mole_dir;

  QWidget * current_tab = ui->tabWidget->currentWidget();
  if (current_tab == ui->rewindTab) {
      if (ui->smartRewind->isChecked()) {
          mole_cmd += " -W ";
      } else {
          mole_cmd += " -w ";
      }
      if (!ui->numRewind->text().isEmpty()) {
        mole_cmd += ui->numRewind->text();
      } else {
        mole_cmd += "0";
        ui->numRewind->setText(QString::number(0));
      }
  } else if (current_tab == ui->dataSelectionTab) {
      if (!ui->startFrame->text().isEmpty()) {
          mole_cmd += " -S " + ui->startFrame->text();
      } else {
          mole_cmd += "- S 0";
          ui->startFrame->setText(QString::number(0));
      }
      if (!ui->endFrame->text().isEmpty()) {
          mole_cmd += " -E " + ui->endFrame->text();
      } else {
          mole_cmd += "- E " + QString::number(INT32_MAX);
          ui->endFrame->setText(QString::number(INT32_MAX));
      }
  } else if (current_tab == ui->dataFillTab) {
      mole_cmd += "-M ";
  }

  if (options->no_checksums) mole_cmd += " --no-check";
  if (options->backup) mole_cmd += " --backup";
  if (options->server) mole_cmd += " --server";
  if (!options->client) mole_cmd += " --no-client";

  Logscroll * mole_log = new Logscroll(NULL, mole_cmd, false);
  mole_log->setWindowTitle(mole_cmd);
  mole_logs.push_back(mole_log);
  if (options->mole_terminal) {
      mole_log->show();
  }

}

void MainWindow::stop_all_moles() {

    printf("Stopping all moles...\n");
    for (unsigned int i = 0; i < mole_logs.size(); i++) {
        if (mole_logs[i]) delete mole_logs[i];
        mole_logs[i] = NULL;
    }
    mole_logs.clear();
    printf("All moles stopped\n");
}

/*
 * Toggles whether or not mole clients are active
 */
void MainWindow::on_toggleMole_clicked() {
  saveConfig();
  stop_all_moles();

  if (mole_active) {
    // unfreeze the UI
    unfreeze();
  } else {
    // ensure there is a primary link selected before doing anything
    if (ui->linkSelect->count() == 0) {
        QMessageBox::warning(this,"No Link Selected","No primary link selected\nPlease make a linklist(s) selection and then select primary link.");
        return;
    }
    if (ui->tabWidget->currentWidget() == ui->dataFillTab) {
        QMessageBox::StandardButton reply = QMessageBox::question(this,
                                                                  "Fill all data gaps?",
                                                                  "This will fill in all dirfile data gaps for link " + ui->linkSelect->currentText() +
                                                                  ". Confirmed?",
                                                                  QMessageBox::Ok  |QMessageBox::Cancel);
        if (reply == QMessageBox::Cancel) {
            return;
        }
    }

    // freeze the UI
    freeze();

    // start moles
    for (int i=0;i<ui->linkSelect->count();i++) start_a_mole(i);

    // establish symlinks
    on_linkSelect_currentIndexChanged(ui->linkSelect->currentText());

    // show helper functions, if necessary
    options->show_helpers();
  }

  // toggle
  mole_active = !mole_active;
}


/*
 * Forms symlinks for a given linklist when the option is changed
 * in the linklist selector.
 */
void MainWindow::on_linkSelect_currentIndexChanged(const QString &arg1)
{
  if (arg1.length() == 0) return;

  QString aname = arg1;
  QString linkname = QString(moledir)+"/"+aname+".lnk";

  QByteArray ba = linkname.toLatin1();

  printf("%s -> %s\n",masterlink,ba.data());

  unlink(masterlink);
  if (symlink(ba.data(),masterlink) < 0)
  {
    printf("Symlink failed (%s->%s)\n",masterlink,ba.data());
  }
  // force dance check
  logend = 0;
}

bool MainWindow::change_remote_host(const QString &arg)
{
  bool connected = false;

  QString hostname = arg.split(QRegExp("\\s+-"), QString::SkipEmptyParts)[0];
  struct TCPCONN tcpconn;
  strcpy(tcpconn.ip,hostname.toLatin1().data());
  tcpconn.flag |= TCPCONN_NOLOOP;

  char names[LINKLIST_MAX_FILENAME_SIZE][LINKLIST_SHORT_FILENAME_SIZE] = {{0}};
  int numlink = 0;

  num_linkfile = 0;
  QStringList storedLinkSelect = QStringList(linkSelect);

  ui->multiLinkSelect->clear();

  // set the port
  set_linklist_client_port(options->client_port);

  printf("Attempting to connect to %s:%d...\n", tcpconn.ip, options->client_port);
  if ((numlink = request_server_archive_list(&tcpconn,names)) > 0)// made a connection with the server
  {
    printf("Got server list\n");

    for (int i=0; i<numlink; i++)
    {
      QString name = QString(names[i]);
      ui->multiLinkSelect->addItem(name);
      if ((options->auto_live && name.endsWith(options->live_name)) || storedLinkSelect.contains(name)) {
          ui->multiLinkSelect->item(i)->setSelected(true);
      }
      num_linkfile++;
    }
    linkSelect = storedLinkSelect;

    // auto select the active link
    auto_select_link();

    ui->linkDir->setText("Available telemetry streams from "+QString(hostname.data()));
    connected = true;
  }
  else
  {
    QMessageBox::information(
        this,
        tr("Guaca-mole"),
        "No archive found at " + QString(hostname.data()) + ":" + QString::number(options->client_port) + ".");
  }
  close_connection(&tcpconn);

  return connected;
}

void MainWindow::auto_select_link() {
    int linkIndex = ui->linkSelect->findText(linkItem);
    if (linkIndex < 0) linkIndex = 0;
    ui->linkSelect->setCurrentIndex(linkIndex);
}

void MainWindow::on_actionClose_triggered()
{
  close();
}

void MainWindow::on_actionSlave_to_triggered()
{
  return;

  bool ok;
  QString dummy;
  QString host = QInputDialog::getText(this, "Choose Master", "What server should guaca connect to?", QLineEdit::Normal, dummy, &ok);

  if (!ok) {
    return;
  } else {
    servermode = true;
    if (1) {
      QMessageBox::information(this,"Guaca is slaved","Guaca slaved to \""+host+"\"...",QMessageBox::Cancel);
    } else {
      QMessageBox::warning(this,"Guaca is not slaved","Could not connect to \""+host+"\".",QMessageBox::Cancel);
    }
    servermode = false;
  }
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this,"About Guaca","Guaca v1.2\n\nGuaca is a front end for the data acquisition client mole. \n"""
                                        "To learn more about mole functionality, see \"mole --help\"\n\n"
                                        "Copyright J. Romualdez 2017-2020");
}

void MainWindow::on_actionAbout_Qt_triggered()
{
  QMessageBox::aboutQt(this,"About Qt");
}

void MainWindow::on_actionOptions_triggered()
{
    options->load_options();
    options->show();
}

void MainWindow::on_actionPurge_old_data_triggered()
{
    QString deletecmd = "find " + QString(archivedir) + "/. -type d -exec rm -r \"{}\" \\;";
    QMessageBox::StandardButton reply = QMessageBox::question(this,
                                                              "Delete archived data?",
                                                              "Are you sure you want to run '" + deletecmd + "'?",
                                                              QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        if (system(deletecmd.toLatin1().data()) < 0) {
          QMessageBox::information(
          this,
          tr("Guacamole!"),
          tr("Couldn't clear archived data."));
        }
    }
}

void MainWindow::on_hosts_activated(int index)
{
    if (change_remote_host(ui->hosts->itemText(index))) {
        saveConfig();
    }
    host_index = index;
    ui->hosts->setCurrentIndex(host_index);
}

void MainWindow::on_actionClear_remote_hosts_triggered()
{
    while (ui->hosts->count() > 2) {
        ui->hosts->removeItem(1);
    }
    host_index = 1;
    on_hosts_activated(1);
}

void MainWindow::on_linkSelect_activated(const QString &arg1)
{
    linkItem = arg1;
}

/*
 * Populates the linklist selector based on the linklists chosen from
 * the main linklist list widget.
 */

void MainWindow::on_multiLinkSelect_itemSelectionChanged()
{
    ui->linkSelect->clear();
    linkSelect.clear();

    QList<QListWidgetItem *> items = ui->multiLinkSelect->selectedItems();
    foreach(QListWidgetItem * T, items)
    {
      ui->linkSelect->addItem(T->text());
      linkSelect << T->text();
    }

    auto_select_link();
}


void MainWindow::on_computeFrameMap_clicked()
{

}
