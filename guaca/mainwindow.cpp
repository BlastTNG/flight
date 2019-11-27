#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

#include "../liblinklist/linklist.h"
#include "../liblinklist/linklist_connect.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#define VERBOSE 0
#define SELECTION_APPEND_TEXT " - Custom selection... -"

char archivedir[128] = "/data/mole";
char configdir[128] = "/data/mole";
char configfile[128] = "guaca.cfg";
char masterlink[128] = "/data/etc/mole.lnk";
char masterloglink[128] = "/data/etc/mole.log";

char *remote_hosts[] = {
  "zaphod.bit",
  NULL
};

void USAGE(void) {

  printf("\n\nGUI front-end for mole to conveniently grab telemetry data from server.\n\n"
      "Usage: guaca [host] [OPTION]...\n\n"
      " -h                 Display this message.\n"
      "\n");

    exit(0);
}

/*
 * This thread waits for slave guacas to connect and then serves
 * up the configuration file to the slave
 */

int server_active = 0;

void server_thread(void * arg)
{
  int sock;
  int client_sock , c;
  struct sockaddr_in server , client;

  struct GUACACONFIG * cfg = (struct GUACACONFIG *) arg;

  unsigned int theport = GUACAPORT;

  //Create socket
  int socket_desc = socket(AF_INET , SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (socket_desc == -1)
  {
    perror("socket could not create server socket");
    return;
  }

  //Prepare the sockaddr_in structure
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(theport);

  int tru = 1;
  setsockopt(socket_desc,SOL_SOCKET,SO_REUSEADDR,&tru,sizeof(int));

  //Bind
  if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
  {
    //print the error message
    perror("bind failed. Unable to start guacamole server: ");
    return;
  }

  //Listen
  listen(socket_desc , 3);


  //Accept and incoming connection
  if (VERBOSE) printf("Waiting for incoming connections...\n");
  c = sizeof(struct sockaddr_in);

  uint8_t configbuf[2048] = {0};

  while (server_active)
  {
    if ((client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) > 0)
    {
      sock = client_sock;
      if (VERBOSE) printf("Client request for config\n");

      memset(configbuf,0,2048);

      int loc = 0;

      configbuf[0] = 0xff; loc += 1; // header
      *((int *) (configbuf+loc)) = cfg->linkindex;    loc += 4;
      *((int *) (configbuf+loc)) = cfg->hostindex;    loc += 4;
      *((int *) (configbuf+loc)) = cfg->checksum;     loc += 4;
      *((int *) (configbuf+loc)) = cfg->server;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->backup;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->rewind;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->active;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->multilinknum; loc += 4;

      for (int i=0;i<MAX_NUM_LINKFILE; i++)
      {
        *((int *) (configbuf+loc)) = cfg->multilinkindex[i]; loc += 4;
      }

      if (send(sock, configbuf, loc, 0) <= 0)
      {
        printf("Unable to send client data\n");
      }

      close(sock);
    }
    else
    {
      usleep(200000);
    }
  }

  return;
}

static int one (const struct dirent *unused)
{
  if (unused) return 1;

  return 1;
}

char * get_linklist_name(char * filename)
{
  int length = strlen(filename);

  // the name needs to be long enough and of the right format
  if ((length < 20) || (filename[length-20] != '_')) return NULL;

  for (int i=length-1; i>=0; i--) {
     if (filename[i] == '.') { // has a file extension, so not what we're looking for
         return NULL;
     }
  }

  filename[length-20] = '\0'; // remove the date from the linklist name

  return filename;
}

int generate_linklist_listfiles()
{
  struct dirent **dir;
  int n = scandir(archivedir, &dir, one, alphasort);

  int num_types = 0;
  char type_names[MAX_NUM_LINKLIST_FILES][LINKLIST_SHORT_FILENAME_SIZE] = {""};
  FILE * type_fds[MAX_NUM_LINKLIST_FILES] = {0};
  char listfilename[LINKLIST_MAX_FILENAME_SIZE] = "";

  if (n < 0) {
      return 0;
  }

  for (int i=0; i<n; i++) {
    char * linklistname;
    if ((linklistname = get_linklist_name(dir[i]->d_name))) {
      int j = 0;
      for (j=0; j<num_types; j++) {
        if (!strcmp(linklistname, type_names[j])) break;
      }
      if (j == num_types) {
        sprintf(listfilename, "%s/%s.lst", archivedir, linklistname);
        type_fds[j] = fopen(listfilename, "w");
        strcpy(type_names[j], linklistname);
        num_types++;
      }
      linklistname[strlen(linklistname)] = '_';
      fprintf(type_fds[j], "%s/%s\n", archivedir, linklistname);
    }
  }
  for (int i=0; i<num_types; i++) {
    fflush(type_fds[i]);
    fclose(type_fds[i]);
  }

  return num_types;
}

void MainWindow::make_listfiles() {
  generate_linklist_listfiles();
}


/*
 * Connects to a server guaca and slaves to it by retrieving
 * the configuration file and applying those settings.
 * Configuration includes current state of mole options
 * as well as whether or not mole clients have been
 * initialized.
 */

int MainWindow::get_server_data()
{
  struct sockaddr_in server_info;
  struct hostent *he;
  int socket_fd;

  uint8_t buf[2048] = {0};

  if ((he = gethostbyname(gnd_ip))==NULL)
  {
    printf("Cannot get host name \"%s\"\n",gnd_ip);
    return -1;
  }
  if (VERBOSE) printf("Connecting to %s...\n",gnd_ip);
  if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0))== -1)
  {
    fprintf(stderr, "Socket Failure!!\n");
    return -1;
  }

  int tru = 1;
  setsockopt(socket_fd,SOL_SOCKET,SO_REUSEADDR,&tru,sizeof(int));

  memset(&server_info, 0, sizeof(server_info));
  server_info.sin_family = AF_INET;
  server_info.sin_port = htons(GUACAPORT);
  server_info.sin_addr = *((struct in_addr *)he->h_addr);

  if (::connect(socket_fd, (struct sockaddr *)&server_info, sizeof(struct sockaddr))<0)
  {
    printf("Client connection refused.\n");
    ::close(socket_fd);
    return -1;
  }

  int rsize = recv(socket_fd,buf,2048,0);
  ::close(socket_fd);

  int loc = 0;
  if ((buf[0] == 0xff) && (rsize > 0))
  {
    loc += 1;
    cfg.linkindex = *((int *) (buf+loc));    loc += 4;
    cfg.hostindex = *((int *) (buf+loc));    loc += 4;
    cfg.checksum = *((int *) (buf+loc));     loc += 4;
    cfg.server = *((int *) (buf+loc));      loc += 4;
    cfg.backup = *((int *) (buf+loc));       loc += 4;
    cfg.rewind = *((int *) (buf+loc));       loc += 4;
    cfg.active = *((int *) (buf+loc));       loc += 4;
    cfg.multilinknum = *((int *) (buf+loc)); loc += 4;

    for (int i=0;i<MAX_NUM_LINKFILE;i++)
    {
      cfg.multilinkindex[i] = *((int *) (buf+loc)); loc += 4;
    }


    if (VERBOSE) printf("Received config from master guaca\n");
    updateSettings();
  }
  else
  {
    printf("Received invalid server message\n");
    return -1;
  }

  return 1;
}

/*
 * A very important function that animates guaca when
 * appropriate. Also takes care of displaying logs in the
 * status bar.
 */

void MainWindow::dancing()
{
  still_dancing = true;
  QString msg = "";

  if (logfile == NULL)
  {
    logfile = fopen(masterloglink,"r");
  }

  if (logfile != NULL)
  {
    fseek(logfile, -MAXLINELENGTH, SEEK_END);

    ssize_t len = fread(buf, 1, MAXLINELENGTH-1, logfile);
    buf[len] = '\0';
    char *last_newline = buf;

    unsigned int i = 1;
    while (strncmp("Frame",last_newline,5) != 0)
    {
      if (i>(MAXLINELENGTH-5))
      {
        last_newline[0] = 0;
        break;
      }
      last_newline = buf+i;
      i++;
    }

    i = 0;
    while (last_newline[i] && (i<strlen(last_newline)))
    {
      if (last_newline[i] == '\r')
      {
        last_newline[i] = 0;
        break;
      }
      i++;
    }

    // msg = QString(last_newline);
    unsigned int current_index = ui->linkSelect->currentIndex();
    if (mole_logs.size() > current_index) {
        msg = mole_logs[current_index]->get_latest_str();
    }
    fclose(logfile);
    logfile = NULL;
  }
  fflush(stdout);

  int togglethemole = 0;

  int thetime = time(0);

  // get file size
  if ((thetime-logend)>1)
  {
    getSettings();
    if (servermode) syncstate = get_server_data();
    else
    {
      syncstate = 0;
    }

    uint64_t file_size = 0;

    if (statfile == NULL)
    {
      statfile = fopen(options->stat_field.toLatin1().data(), "r");
    }

    if (statfile != NULL)
    {
      fseek(statfile,0,SEEK_END);
      file_size = ftell(statfile);

      if (file_size == prev_size)
      {
        data_incoming = 0;
      }
      else
      {
        data_incoming = 1;
      }
      fclose(statfile);
      statfile = NULL;
      has_warned = false;
    }
    else
    {
      if (!has_warned) QMessageBox::warning(this,"Invalid stat file",
                            "Reference field "+ options->stat_field + " is invalid.\n Change under File->Options->Advanced->Reference Field.");
      has_warned = true;
      data_incoming = 0;
    }

    prev_size = file_size;

    logend = thetime;
    //printf("%s\n",temp);
    togglethemole = 1;
  }

  if (!data_incoming) msg = "Mole not receiving data.\n";

  int isrun = system("pidof -x mole > /dev/null");
  if(!isrun) // mole is running on the system
  {
    mole_active = 1;
    freeze();
    if (!cfg.active && servermode && togglethemole) // slaved and shouldn't be running
    {
      on_toggleMole_clicked();
    }
  }
  else // mole is not running on the system
  {
    mole_active = 0;
    unfreeze();

    if (cfg.active && servermode && togglethemole) // saved and should be running
    {
      on_toggleMole_clicked();
    }
  }

  cfg.active = mole_active;
  ui->statusBar->showMessage(msg);

  if (mole_active)
  {
    if (data_incoming)
    {
      if (image_i==IMAGE_LOOP_HIGH) inc = -1;
      else if (image_i==IMAGE_LOOP_LOW) inc = 1;
    }
    else inc = 1;
  }
  else
  {
    inc = 1;
    if  (image_i==(IMAGE_IND_OFF+1)%IMAGE_TOTAL) image_i = IMAGE_IND_OFF;
  }

  int ss = qMin(ui->dance->width(),ui->dance->height())*15/8;
  qs = QSize(ss,ss);
  ui->dance->setPixmap(qi[image_i].pixmap(qs).scaled(ss,ss,Qt::KeepAspectRatio));
  ui->dance->show();

  image_i = (image_i+inc)%IMAGE_TOTAL;

  still_dancing = false;
}


/*
 * Updates the mole setting based on the configuration file
 */
void MainWindow::updateSettings()
{
  ui->numRewind->setText(QString::number(cfg.rewind));
  options->server = cfg.server;
  options->backup = cfg.backup;
  options->no_checksums = cfg.checksum;

  //printf("set to %d\n",cfg.linkindex);
}

/*
 * Gets the settings from the GUI and saves them to the configuration file.
 */
void MainWindow::getSettings()
{
  cfg.rewind = ui->numRewind->text().toInt();
  cfg.server = options->server;
  cfg.backup = options->backup;
  cfg.checksum = options->no_checksums;

  //printf("saved to %d\n",cfg.linkindex);

}

/*
 * Main constructor: looks in the linklist directory for all the available
 * linklists and makes them selectable in the main link widget. Also starts
 * the server thread if necessary and the animation/log checking thread.
 */

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    settings("SuperBIT", "guaca")
{
  ui->setupUi(this);

  ui->numRewind->setValidator(new QIntValidator(0, 1e8, this) );
  ui->startFrame->setValidator(new QIntValidator(0, INT32_MAX, this) );
  ui->endFrame->setValidator(new QIntValidator(0, INT32_MAX, this) );

  logfile = NULL;

  memset((void *) &cfg,0,sizeof(struct GUACACONFIG));
  cfg.rewind = 100;

  // open the configuration directory and check previous configuration
  mkdir(configdir,00755);
  char fname[128] = {0};
  sprintf(fname,"%s/%s",configdir,configfile);

  if (access(fname,F_OK) == 0)
  {
    // get previous config
    FILE * cfgfile = fopen(fname,"rb");
    uint8_t overflow;

    if ((fread(&cfg,1,sizeof(struct GUACACONFIG),cfgfile) != sizeof(struct GUACACONFIG)) || (fread(&overflow,1,1,cfgfile) != 0))
    {
      memset(&cfg,0,sizeof(struct GUACACONFIG));
    }

    fclose(cfgfile);
  }

  generate_linklist_listfiles();

  options = new Options(this);

  updateSettings();

  if (settings.contains("mainwindow/customConfig")) {
      qDebug() << "Restoring saved options";
      loadConfig();
  } else {
      qDebug() << "Using default options";
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
          host_index = add_a_host(args.at(1));
          ui->hosts->setCurrentIndex(host_index);
      } else {
          USAGE();
      }
  }

  // Query remote host
  change_remote_host(ui->hosts->currentText());

  QString base = ":/images/guaca-";
  QString ext = ".svg";

  for (int i=0; i<IMAGE_TOTAL; i++)
  {
    qi[i] = QIcon(base+QString::number(i)+ext);
  }
  qs = QSize(300,300);

  ui->dance->setAlignment(Qt::AlignCenter);
  ui->dance->setPixmap(qi[IMAGE_IND_OFF].pixmap(qs));
  ui->dance->show();
  image_i = IMAGE_IND_OFF;
  inc = 1;
  srand(time(0));

  mole_active = 0;
  data_incoming = 1;
  syncstate = 0;

  logend = time(0);
  statfile = fopen(options->stat_field.toLatin1().data() , "r");

  server_active = 1;
  f1 = QtConcurrent::run(server_thread, &cfg);
  servermode = 0;

  _ut = new QTimer(this);
  _ut->setInterval(90);
  connect(_ut,SIGNAL(timeout()),this,SLOT(dancing()));
  _ut->start();

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

  if (logfile) fclose(logfile);
  if (!servermode) {
    f1.cancel();
  }
  server_active = 0;

  if (options) delete options;
  options = NULL;
  if (ui) delete ui;
  ui = NULL;
}

void MainWindow::closeEvent(QCloseEvent *event) {
    savePosition();
    event->accept();
    _ut->stop();
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
    for (int i = 1; i < ui->hosts->count()-1; i++) {
        hosts << ui->hosts->itemText(i);
    }
    settings.setValue("mainwindow/host_list", hosts);
    settings.setValue("mainwindow/host_index", ui->hosts->currentIndex());
    settings.setValue("mainwindow/linkItem", linkItem);
    settings.setValue("mainwindow/linkSelect", linkSelect);

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
    linkSelect = QVariant(settings.value("mainwindow/linkSelect")).toStringList();

}

void MainWindow::defaultConfig() {
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
                          "-p "+QString::number(options->client_port)+" -P "+QString::number(options->server_port)+" ";

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
  }

  if (options->no_checksums) mole_cmd += " --no-check";
  if (options->backup) mole_cmd += " --backup";
  if (options->server) mole_cmd += " --server";

  // update configuration file
  getSettings();
  char fname[128];
  sprintf(fname,"%s/%s",configdir,configfile);
  FILE * cfgfile = fopen(fname,"wb");
  fwrite(&cfg,sizeof(GUACACONFIG),1,cfgfile);
  fflush(cfgfile);
  fclose(cfgfile);

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
  stop_all_moles();

  if (mole_active)
  {
    unfreeze();
    data_incoming = 1;
  }
  else
  {
    freeze();
    for (int i=0;i<ui->linkSelect->count();i++) start_a_mole(i);

    if (ui->linkSelect->count() == 0) QMessageBox::warning(this,"No Link Selected","No primary link selected\nPlease make a linklist(s) selection and then select primary link.");

    on_linkSelect_currentIndexChanged(ui->linkSelect->currentText());
    //qDebug(ui->linkSelect->currentText().toLatin1());

    options->show_helpers();

    data_incoming = 1;
    logend = time(0);
  }

  mole_active = 1-mole_active;
  cfg.active = mole_active;
}


/*
 * Forms symlinks for a given linklist when the option is changed
 * in the linklist selector.
 */
void MainWindow::on_linkSelect_currentIndexChanged(const QString &arg1)
{

  if (arg1.length() == 0) return;

  QString aname = arg1;
  QString linkname = QString(configdir)+"/"+aname+".lnk";

  QByteArray ba = linkname.toLatin1();

  printf("%s -> %s\n",masterlink,ba.data());

  unlink(masterlink);
  if (symlink(ba.data(),masterlink) < 0)
  {
    printf("Symlink failed (%s->%s)\n",masterlink,ba.data());
  }

  aname = arg1;
  linkname = QString(configdir)+"/"+aname+".log";
  ba = linkname.toLatin1();

  printf("%s -> %s\n",masterloglink,ba.data());

  unlink(masterloglink);
  if (symlink(ba.data(),masterloglink) < 0)
  {
    printf("Symlink failed (%s->%s)\n",masterloglink,ba.data());
  }
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
  ui->multiLinkSelect->clear();


  printf("Attempting to connect to %s...\n", tcpconn.ip);
  if ((numlink = request_server_archive_list(&tcpconn,names)) > 0)// made a connection with the server
  {
    printf("Got server list\n");

    QStringList storedLinkSelect = linkSelect;

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
        tr("No archive found.") );
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
  bool ok;
  QString thehost;
  QString host = QInputDialog::getText(this, "Choose Master", "What master server should guaca connect to?", QLineEdit::Normal, thehost, &ok);

  if (!ok)
  {
    return;
  }
  else
  {
    QByteArray ba = host.toLatin1();
    const char *c_str2 = ba.data();
    strcpy(gnd_ip,c_str2);
    servermode = 1;
    while (syncstate == 0) QTest::qWait(200);
    if (syncstate > 0)
    {
      QMessageBox::information(this,"Guaca is slaved","Guaca slaved to \""+host+"\"...",QMessageBox::Cancel);
      servermode = 0;
    }
    else
    {
      QMessageBox::warning(this,"Guaca is not slaved","Could not connect to \""+host+"\".",QMessageBox::Cancel);
      servermode = 0;
    }
  }
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this,"About Guaca","Guaca v1.0\n\nGuaca is a front end for the data acquisition client mole. \n"""
                                        "To learn more about mole functionality, see \"mole --help\"\n\n"
                                        "Copyright J. Romualdez 2017-2019");
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

int MainWindow::add_a_host(const QString &thehost)
{
    int index = ui->hosts->count()-1;

    int existing_index = ui->hosts->findText(thehost);
    if (existing_index < 0) {
        // add the new host
        ui->hosts->insertItem(index, thehost);
        return index;
    } else {
        return existing_index;
    }
}

void MainWindow::on_hosts_activated(int index)
{
    bool reconnect = host_index != index;
    bool newitem = false;

    // the last item was selected, which is the add host option
    if (index == (ui->hosts->count()-1)) {
      bool ok;
      QString thehost = QInputDialog::getText(this, "Add remote host", "What remote host should mole connect to?", QLineEdit::Normal, NULL, &ok);
      if (ok) {
          newitem = add_a_host(thehost) == index;
          reconnect = true;
      } else {
          // return to previously selected
          ui->hosts->setCurrentIndex(host_index);
          reconnect = false;
      }
    }
    // reconnect to the server if host has actually changed
    if (reconnect) {
        if (change_remote_host(ui->hosts->itemText(index))) {
            host_index = index;
        } else if (newitem) {
            ui->hosts->removeItem(index);
            change_remote_host(ui->hosts->itemText(host_index));
        }
        ui->hosts->setCurrentIndex(host_index);
    }
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
