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

#include "../external_libs/linklist/linklist.h"
#include "../external_libs/linklist/linklist_connect.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#define VERBOSE 0
#define SELECTION_APPEND_TEXT " - Custom selection... -"

char configdir[128] = "/data/mole";
char configfile[128] = "guaca.cfg";
char molestat[128] = "/data/etc/mole.lnk/time";
char masterlink[128] = "/data/etc/mole.lnk";
char masterloglink[128] = "/data/etc/mole.log";

char *remote_hosts[] = {
  "cacofonix",
  NULL
};

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
      *((int *) (configbuf+loc)) = cfg->server;      loc += 4;
      *((int *) (configbuf+loc)) = cfg->backup;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->rewind;       loc += 4; 
      *((int *) (configbuf+loc)) = cfg->active;       loc += 4;
      *((int *) (configbuf+loc)) = cfg->multilinknum; loc += 4;

      for (int i=0;i<MAX_NUM_LINKFILE; i++)
      {
        *((int *) (configbuf+loc)) = cfg->multilinkindex[i]; loc += 4;
      } 
      strcpy((char *) (configbuf+loc),cfg->customhost); loc += 64;

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
    strcpy(cfg.customhost,(char *) (buf+loc));


    if (VERBOSE) printf("Received config from master guaca\n");
    //printf("%d %d %d %d %d %d %d %s\n",cfg.linkindex,cfg.hostindex,cfg.checksum,cfg.server,cfg.backup,cfg.rewind,cfg.active,cfg.customhost);
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

    //printf("%s\n",last_newline);
    msg = QString(last_newline);

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
      statfile = fopen(molestat,"r");
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
    }
    else
    {
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

}


/*
 * Updates the mole setting based on the configuration file
 */
void MainWindow::updateSettings()
{
  ui->remoteHost->setCurrentIndex(cfg.hostindex);
  ui->numRewind->setText(QString::number(cfg.rewind));
  if (cfg.checksum) ui->checksum->setChecked(true);
  else  ui->checksum->setChecked(false);
  if (cfg.backup) ui->backup->setChecked(true);
  else  ui->backup->setChecked(false);
  if (cfg.server) ui->server->setChecked(true);
  else  ui->server->setChecked(false);

  ui->multiLinkSelect->clearSelection();


  for (int i=0;i<cfg.multilinknum;i++)
  {
    //printf("%d\n",cfg.multilinkindex[i]);
    if (cfg.multilinkindex[i] < ui->multiLinkSelect->count()) ui->multiLinkSelect->item(cfg.multilinkindex[i])->setSelected(true);
  }
  ui->linkSelect->setCurrentIndex(cfg.linkindex);
  ui->remoteHost->setItemText(ui->remoteHost->count()-1,QString(cfg.customhost)+SELECTION_APPEND_TEXT);
 

  //printf("set to %d\n",cfg.linkindex);
}

/*
 * Gets the settings from the GUI and saves them to the configuration file.
 */
void MainWindow::getSettings()
{
  cfg.hostindex = ui->remoteHost->currentIndex();
  cfg.checksum = (ui->checksum->checkState() == Qt::Checked);
  cfg.server = (ui->server->checkState() == Qt::Checked);
  cfg.backup = (ui->backup->checkState() == Qt::Checked);
  cfg.rewind = ui->numRewind->text().toInt();

  QModelIndexList indexes = ui->multiLinkSelect->selectionModel()->selectedIndexes();

  int i = 0;
  foreach(QModelIndex index, indexes)
  {
      cfg.multilinkindex[i] = index.row();
      //printf("%d %d\n",i,index.row());
      i++;
  }
  cfg.multilinknum = i;
  cfg.linkindex = ui->linkSelect->currentIndex();
  //printf("saved to %d\n",cfg.linkindex);
  QString customhost = ui->remoteHost->itemText(ui->remoteHost->count()-1).split(QRegExp("\\s+-"), QString::SkipEmptyParts)[0];

  strcpy(cfg.customhost,customhost.toLatin1().data());

}

/*
 * Main constructor: looks in the linklist directory for all the available
 * linklists and makes them selectable in the main link widget. Also starts
 * the server thread if necessary and the animation/log checking thread.
 */

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  //on_remoteHost_currentIndexChanged("localhost");

  ui->numRewind->setValidator(new QIntValidator(0, 1e8, this) );

  logfile = NULL;

  memset((void *) &cfg,0,sizeof(struct GUACACONFIG));
  cfg.rewind = 100;

  // file the remote hosts selection box
  for (int i = 0; remote_hosts[i]; i++)
  {
    ui->remoteHost->addItem(QString(remote_hosts[i]));
  }
  ui->remoteHost->addItem(QString(" - Custom selection - "));

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

  updateSettings();
  change_remote_host(ui->remoteHost->itemText(cfg.hostindex));

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
  statfile = fopen(molestat,"r");

  if (QCoreApplication::arguments().count() > 1) // slave mode
  {
    QByteArray ba = QCoreApplication::arguments().at(1).toLatin1();
    const char *c_str2 = ba.data();
    strcpy(gnd_ip,c_str2);
    printf("Guaca is slaved to \"%s\"\n",gnd_ip);
    //servermode = 1;
  }
  else
  {
    server_active = 1;
    f1 = QtConcurrent::run(server_thread, &cfg);
    servermode = 0;
  }

  _ut = new QTimer(this);
  _ut->setInterval(90);
  connect(_ut,SIGNAL(timeout()),this,SLOT(dancing()));
  _ut->start();
}

MainWindow::~MainWindow()
{
  if (logfile) fclose(logfile);
  if (!servermode)
  {
    f1.cancel();
  }
  else
  {
    int ret = system("pkill mole");
    if (ret < 0) printf("Unable to kill mole in slave mode\n");
  }
  server_active = 0;
  delete ui;
}

/*
 * Unfreezes all user selectable widgets in the GUI
 */
void MainWindow::unfreeze()
{
  if (!servermode)
  {
    ui->toggleMole->setText("Start MOLE");
    ui->checksum->setEnabled(true);
    ui->server->setEnabled(true);;
    ui->multiLinkSelect->setEnabled(true);
    ui->numRewind->setEnabled(true);
    ui->remoteHost->setEnabled(true);
    ui->backup->setEnabled(true);
    ui->toggleMole->setEnabled(true);
    ui->linkSelect->setEnabled(true);
  }
  else
  {
    ui->checksum->setDisabled(true);
    ui->server->setDisabled(true);;
    ui->multiLinkSelect->setDisabled(true);
    ui->numRewind->setDisabled(true);
    ui->remoteHost->setDisabled(true);
    ui->backup->setDisabled(true);
    ui->toggleMole->setDisabled(true);
    ui->linkSelect->setDisabled(true);
  }
}

/*
 * Freezes all user selectable widgets in the GUI. Leaves the
 * link selector unfrozen to allow for mole.lnk changes while
 * running.
 */
void MainWindow::freeze()
{
  ui->checksum->setDisabled(true);
  ui->server->setDisabled(true);
  ui->multiLinkSelect->setDisabled(true);
  ui->numRewind->setDisabled(true);
  ui->remoteHost->setDisabled(true);
  ui->backup->setDisabled(true);
  ui->toggleMole->setText("Stop MOLE");

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
                          "@"+ui->remoteHost->currentText().split(QRegExp("\\s+-"), QString::SkipEmptyParts)[0];

  if (!ui->numRewind->text().isEmpty()) mole_cmd += " --rewind " +ui->numRewind->text();
  else
  {
    mole_cmd += " --rewind=0";
    ui->numRewind->setText(QString::number(0));
  }

  if (ui->checksum->checkState() == Qt::Checked) mole_cmd += " --no-check";
  if (ui->backup->checkState() == Qt::Checked) mole_cmd += " --backup";
  if (ui->server->checkState() == Qt::Checked) mole_cmd += " --server";


  QString aname = ui->linkSelect->itemText(index);
  QString linkname = QString(configdir)+"/"+aname+".log";

  mole_cmd += " > "+linkname+" &";
  getSettings();

  // update configuration file
  char fname[128];
  sprintf(fname,"%s/%s",configdir,configfile);
  FILE * cfgfile = fopen(fname,"wb");
  fwrite(&cfg,sizeof(GUACACONFIG),1,cfgfile);
  fflush(cfgfile);
  fclose(cfgfile);

  /*
  QMessageBox::information(
      this,
      tr("Application Name"),
      mole_cmd);
  */

  QByteArray ba = mole_cmd.toLatin1();
  int ret = system(ba.data());
  printf("%s\n",ba.data());
  if (ret < 0)
  {
    QMessageBox::information(
    this,
    tr("Guacamole!"),
    tr("Couldn't start mole."));
  }
  //printf("Ran %s\n",ba.data());
  QTest::qWait(50);

}

/*
 * Toggles whether or not mole clients are active
 */
void MainWindow::on_toggleMole_clicked()
{
  int ret = system("pkill mole");

  if (ret < 0)
  {
    QMessageBox::information(
      this,
      tr("Guacamole!"),
      tr("Couldn't kill mole."));
  }

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

    data_incoming = 1;
    logend = time(0);
  }

  mole_active = 1-mole_active;
  cfg.active = mole_active;
}

/*
 * Populates the linklist selector based on the linklists chosen from
 * the main linklist list widget.
 */
void MainWindow::on_multiLinkSelect_itemSelectionChanged()
{
  ui->linkSelect->clear();

  QList<QListWidgetItem *> items = ui->multiLinkSelect->selectedItems();
  foreach(QListWidgetItem * T, items)
  {
    ui->linkSelect->addItem(T->text());
  }
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
void MainWindow::on_remoteHost_activated(const int &arg1)
{
  QString hostname = ui->remoteHost->itemText(arg1);

  if (arg1 == (ui->remoteHost->count()-1)) // last item is the custom item
  {
    QString thehost;
    bool ok;
    hostname = QInputDialog::getText(this, "Choose Remote Host", "What remote host should mole connect to?", QLineEdit::Normal, thehost, &ok);
    if (!ok)
    {
      printf("No host to connect to. Exiting.\n");
      exit(0);
    }
    hostname += SELECTION_APPEND_TEXT;
    ui->remoteHost->setItemText(arg1,hostname);
  }

  change_remote_host(hostname);
}


void MainWindow::change_remote_host(const QString &arg)
{
  QString hostname = arg.split(QRegExp("\\s+-"), QString::SkipEmptyParts)[0];
  struct TCPCONN tcpconn;
  strcpy(tcpconn.ip,hostname.toLatin1().data());
  tcpconn.flag |= TCPCONN_NOLOOP;

  char names[LINKLIST_MAX_FILENAME_SIZE][LINKLIST_SHORT_FILENAME_SIZE] = {{0}};
  int numlink = 0;

  num_linkfile = 0;
  //for (int i=0;i<ui->multiLinkSelect->count();i++) ui->multiLinkSelect->takeItem(i);
  ui->multiLinkSelect->clear();


  printf("Attempting to connect to %s...\n", tcpconn.ip);
  if ((numlink = request_server_archive_list(&tcpconn,names)) > 0)// made a connection with the server
  {
    printf("Got server list\n");
    int numactive = 0;

    //ui->multiLinkSelect->clear();
    for (int i=0;i<numlink;i++)
    {
      bool active = 0;

      ui->multiLinkSelect->addItem(names[i]);
      if (active)
      {
          ui->multiLinkSelect->item(i)->setSelected(true);
          numactive++;
      }
      num_linkfile++;
    }

    ui->linkDir->setText("Available telemetry streams from "+QString(hostname.data()));
  }
  else
  {
    printf("No archive found at %s\n", tcpconn.ip);
    ui->remoteHost->setCurrentIndex(ui->remoteHost->count()-1);
    on_remoteHost_activated(ui->remoteHost->count()-1);
  }
  close_connection(&tcpconn);
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
