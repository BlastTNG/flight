// ***************************************************
// *  Programmed by Adam Hincks                      *
// *                                                 *
// *  Comments on classes & functions in .cpp file   *
// ***************************************************

#ifndef NARSIL_H
#define NARSIL_H

#include "commands.h"

#include <qvariant.h>
#include <qdialog.h>
#include <qspinbox.h>

#include <sys/types.h>

#define SMALL_POINT_SIZE 10
#define LARGE_POINT_SIZE 12

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QButtonGroup;
class QFrame;
class QLabel;
class QLineEdit;
class QListBox;
class QListBoxItem;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QPixmap;
class QCheckBox;
class QComboBox;
class QMultiLineEdit;

class KstFile;
class DoubleEntry;

extern double defaults[N_MCOMMANDS][MAX_N_PARAMS];

class MainForm : public QDialog
{
  Q_OBJECT

public:
  MainForm(char *cf, QWidget* parent = 0, const char* name = 0,
			     bool modal = FALSE, WFlags fl = 0);
  ~MainForm();

  QColorGroup *NColorGroup;
  QColorGroup *NColorGroup2;
  QFrame *NTopFrame;
  QFrame *NBotFrame;
  QLabel *NCurveFileCaption;
  QLineEdit *NCurveFile;
  QButtonGroup *NGroupsBox;
  QRadioButton *NGroups[N_GROUPS];
  QPushButton *NSendButton;
  QPushButton *NSettingsButton;
  QPushButton *NCurveFileButton;
  QPushButton *QuitButton;
  QListBox *NCommandList;
  QLabel *NAboutLabel;
  QLabel *NParamLabels[MAX_N_PARAMS];
  DoubleEntry *NParamFields[MAX_N_PARAMS];
  QLabel *NWaitImage;
  QCheckBox *NVerbose;
  QComboBox *NSendMethod;
  QComboBox *NSendRoute;
  QDialog *NSettingsWindow;
  QMultiLineEdit *NLog;
  QPushButton *NCloseSettingsWindow;

  void TurnOn(QTimer *t);
  void TurnOff(QTimer *t);
  QTimer *timer;

protected:
  QGridLayout *NGroupsLayout;
  QVBoxLayout *NSettingsLayout;
  QHBoxLayout *NTSettingsLayout;
  QHBoxLayout *NMSettingsLayout;
  QHBoxLayout *NBSettingsLayout;

private:
  int GroupSIndexes(int group, int *indexes);
  int GroupMIndexes(int group, int *indexes);
  int GetGroup();
  int SIndex(QString cmd);
  int MIndex(QString cmd);
  int LongestParam();
  void ReadLog(QMultiLineEdit *dest);
  void WriteLog(QMultiLineEdit *dest, char *args[]);
  void WriteLog(QMultiLineEdit *dest, int retstatus);

  int lastmcmd;
  QString curvefile;
  KstFile *DataSource;
  int fid;
  bool sending;
  pid_t sendingpid;

  char framenum;
  char numframes;
  char dir;
  QPixmap *Images[4];

public slots:
  void ChangeCommandList();
  void ChooseCommand();
  void Quit();
  void SendCommand();
  void ChangeImage();
  void ChangeCurFile();
  void ShowSettings();
};

#endif
